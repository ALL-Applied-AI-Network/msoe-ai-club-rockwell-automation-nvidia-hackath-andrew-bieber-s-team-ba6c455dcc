"""Simulation-assisted overhead camera; poses measured from rendered masks only.
Semantic identities are simulator supplied. NOT real RGB object recognition.
"""
import json
from pathlib import Path
import numpy as np
import scene_config as C

WIDTH=1.20
CENTER=np.array([.30,0.])
RES=3072


def angle_error(a,b,kind):
    period=90 if kind=='cube' else 180
    return 0. if kind=='cap' else abs((a-b+period/2)%period-period/2)


def screw_outline(xy,yaw_deg,head_first):
    """Top-down screw silhouette as convex XY polygons: head trapezoid + shank."""
    a=np.radians(yaw_deg); u=np.array([np.cos(a),np.sin(a)])*(1 if head_first else -1)
    v=np.array([-u[1],u[0]]); c=np.asarray(xy); L=C.SCREW_LENGTH
    hr,sr,hh=C.SCREW_HEAD_D/2,C.SCREW_SHANK_D/2,C.SCREW_HEAD_H
    s0=c-u*L/2
    head=[s0+v*hr,s0-v*hr,s0+u*hh-v*sr,s0+u*hh+v*sr]
    shank=[s0+u*hh+v*sr,s0+u*hh-v*sr,c+u*L/2-v*sr,c+u*L/2+v*sr]
    return [np.array(head),np.array(shank)]


def fit_mask(mask,kind,size=None):
    import cv2
    rows,cols=np.nonzero(mask)
    if len(rows)<30: raise RuntimeError(f'{kind}: mask missing or too small')
    if min(rows.min(),cols.min())<=1 or max(rows.max(),cols.max())>=RES-2:
        raise RuntimeError(f'{kind}: clipped by image edge')
    # Convert calibrated orthographic pixel centres to table XY.
    points=np.column_stack((CENTER[0]+((cols+.5)/RES-.5)*WIDTH,
                            CENTER[1]+(.5-(rows+.5)/RES)*WIDTH)).astype(np.float32)
    rect=cv2.minAreaRect(points)
    xy,(w,h),degrees=rect
    long,short=max(w,h),min(w,h)
    yaw=degrees if w>=h else degrees+90
    # Geometry gates follow the selected simulated object dimensions. Labels still
    # come from the simulator; these ranges do not turn this into RGB recognition.
    vial_range = (C.VIAL_LENGTH-.012, C.VIAL_LENGTH+.010,
                  2*C.VIAL_RADIUS-.005, 2*C.VIAL_RADIUS+.007)
    edge = size or C.CUBE_EDGE
    cube_range = (edge-.003, edge+.003)*2
    screw_range = (C.SCREW_LENGTH-.004, C.SCREW_LENGTH+.004,
                   C.SCREW_HEAD_D-.003, C.SCREW_HEAD_D+.003)
    ranges={'cube':cube_range,'vial':vial_range,'cap':(.028,.035,.028,.035),
            'screw':screw_range}
    lo,hi,slo,shi=ranges[kind]
    if not lo<=long<=hi or not slo<=short<=shi:
        raise RuntimeError(f'{kind}: incomplete/occluded mask, envelope {long*1000:.1f} x {short*1000:.1f} mm')
    # Reject disconnected silhouettes; pixels may have small render holes.
    # Work on a padded ROI, retaining full-image coordinates for calibration.
    # Padding covers the fitted silhouette beyond the observed pixel bounds.
    pad=max(int(rows.max()-rows.min()),int(cols.max()-cols.min()))+4
    y0=max(0,int(rows.min())-pad); y1=min(RES,int(rows.max())+pad+1)
    x0=max(0,int(cols.min())-pad); x1=min(RES,int(cols.max())+pad+1)
    cropped=mask[y0:y1,x0:x1]
    count,_,stats,_=cv2.connectedComponentsWithStats(cropped.astype(np.uint8),8)
    if count<2 or stats[1:,cv2.CC_STAT_AREA].max()/len(rows)<.90:
        raise RuntimeError(f'{kind}: fragmented segmentation')
    expected=np.zeros(cropped.shape,np.uint8)
    corners=cv2.boxPoints(rect)
    to_px=lambda pts:np.column_stack(((pts[:,0]-CENTER[0])/WIDTH*RES+RES/2-.5,
                                      RES/2-(pts[:,1]-CENTER[1])/WIDTH*RES-.5))
    pixels=to_px(corners)
    if kind=='screw':
        # Head at either end; keep the better match.
        best=None
        for head_first in (True,False):
            trial=np.zeros(cropped.shape,np.uint8)
            for poly in screw_outline(xy,yaw,head_first):
                cv2.fillConvexPoly(trial,np.round(to_px(poly)).astype(int)-np.array([x0,y0]),1)
            score=np.logical_and(cropped,trial).sum()/max(1,np.logical_or(cropped,trial).sum())
            if best is None or score>best[0]: best=(score,trial)
        expected=best[1]
    elif kind=='cap':
        pixel_center=((np.array(xy)-CENTER)/WIDTH*RES*np.array([1,-1])+RES/2-.5).round().astype(int)
        cv2.circle(expected,tuple(pixel_center-np.array([x0,y0])),round((long+short)/4/WIDTH*RES),1,-1)
    else:
        cv2.fillConvexPoly(expected,np.round(pixels).astype(int)-np.array([x0,y0]),1)
    iou=float(np.logical_and(cropped,expected).sum()/max(1,np.logical_or(cropped,expected).sum()))
    if iou<.85: raise RuntimeError(f'{kind}: silhouette fit IoU {iou:.3f} below .85')
    if kind=='cube': yaw=(yaw+45)%90-45
    elif kind in ('vial','screw'): yaw=(yaw+90)%180-90
    else: yaw=0.
    return dict(xy=list(map(float,xy)),yaw_deg=float(yaw),pixels=int(len(rows)),silhouette_iou=iou,
                measured_length_mm=float(long*1000),measured_width_mm=float(short*1000))


def mask_centroid(mask, min_pixels=30):
    """Lenient top-down read of one object: is it visible, and where (table XY)?
    Unlike fit_mask it accepts any pose -- a cube tumbled against a bin wall,
    a cap on its side -- because verification only needs the location."""
    rows, cols = np.nonzero(mask)
    if len(rows) < min_pixels:
        return dict(found=False, pixels=int(len(rows)))
    x = CENTER[0]+((cols.mean()+.5)/RES-.5)*WIDTH
    y = CENTER[1]+(.5-(rows.mean()+.5)/RES)*WIDTH
    return dict(found=True, pixels=int(len(rows)), xy=[float(x), float(y)])


def verify_placement(observation, pick):
    """Camera verdict for one placed object: (placed?, reason).
    Bin objects: visible inside the right bin's pocket. Floor slots (vials):
    centroid within 6 mm of the slot."""
    import scan_sort_bins as B
    if not observation.get('found'):
        return False, 'not visible to the overhead camera'
    xy = np.asarray(observation['xy'])
    if pick.get('bin'):
        (bx, by), _ = B.REAL_BINS[pick['bin']]
        half = B.BIN_POCKET/2 - .002
        if abs(xy[0]-bx) < half and abs(xy[1]-by) < half:
            return True, f"seen inside the {pick['bin']} bin"
        return False, (f"seen at ({xy[0]:.3f}, {xy[1]:.3f}) m, outside the {pick['bin']} bin")
    error = float(np.linalg.norm(xy-np.asarray(pick['place_xy'])))
    if error < .006:
        return True, f'seen {error*1000:.1f} mm from its slot'
    return False, f'seen {error*1000:.1f} mm from its slot'


class Scanner:
    def __init__(self,stage,items,directory,save_images=False):
        from pxr import UsdGeom,Gf,Usd
        from isaacsim.core.utils.semantics import add_labels
        import omni.replicator.core as rep
        self.save_images=save_images
        self.rep=rep; self.items=items; self.directory=Path(directory);self.number=0
        self.directory.mkdir(parents=True,exist_ok=True)
        camera=UsdGeom.Camera.Define(stage,'/World/RescanCamera')
        camera.CreateProjectionAttr('orthographic')
        camera.CreateHorizontalApertureAttr(WIDTH*10)
        camera.CreateVerticalApertureAttr(WIDTH*10)
        camera.CreateClippingRangeAttr(Gf.Vec2f(.01,10.))
        UsdGeom.Xformable(camera).AddTranslateOp().Set(Gf.Vec3d(CENTER[0],CENTER[1],2.1))
        self.tags={p['prim']:f"scan_{p['category']}_{p['index']}" for p in items}
        for pick in items:
            for prim in Usd.PrimRange(stage.GetPrimAtPath(pick['prim'])):
                add_labels(prim,labels=[self.tags[pick['prim']]],instance_name='class')
        self.product=rep.create.render_product(str(camera.GetPath()),(RES,RES))
        self.seg=rep.AnnotatorRegistry.get_annotator('semantic_segmentation',init_params={'colorize':False})
        self.rgb=rep.AnnotatorRegistry.get_annotator('rgb') if save_images else None
        self.seg.attach(self.product)
        if self.rgb is not None: self.rgb.attach(self.product)
        self.product.hydra_texture.set_updates_enabled(False)
        (self.directory/'camera.json').write_text(json.dumps(dict(projection='orthographic',
            width_m=WIDTH,center_xy_m=CENTER.tolist(),resolution=[RES,RES],
            labels='simulator supplied; geometry fitted to rendered masks'),indent=2))

    def capture(self,required,save=True):
        # The high-resolution camera must not render on every viewport update.
        self.product.hydra_texture.set_updates_enabled(True)
        try:
            return self._capture(required,save)
        finally:
            self.product.hydra_texture.set_updates_enabled(False)

    def _frame(self):
        # Render without advancing physics; leave timeline playing.
        self.rep.orchestrator.step(delta_time=0.0,pause_timeline=False)
        self.rep.orchestrator.step(delta_time=0.0,pause_timeline=False)
        raw=self.seg.get_data()
        if not isinstance(raw,dict) or 'data' not in raw: raise RuntimeError('Segmentation camera returned no frame')
        data=np.asarray(raw['data']).squeeze()
        if data.shape!=(RES,RES): raise RuntimeError(f'Unexpected segmentation frame shape {data.shape}')
        return data,raw.get('info',{}).get('idToLabels',{})

    def _mask(self,data,labels,prim):
        tag=self.tags[prim]
        ids=[]
        for identity,values in labels.items():
            value=values.get('class','') if isinstance(values,dict) else values
            tokens=value if isinstance(value,list) else str(value).split(',')
            if tag in [str(v).strip() for v in tokens]: ids.append(int(identity))
        return np.isin(data,ids)

    def locate(self,items):
        """One overhead frame; lenient location of each object (see mask_centroid)."""
        self.product.hydra_texture.set_updates_enabled(True)
        try:
            data,labels=self._frame()
        finally:
            self.product.hydra_texture.set_updates_enabled(False)
        output={p['prim']:mask_centroid(self._mask(data,labels,p['prim'])) for p in items}
        self.number+=1
        Path(str(self.directory/f'verify_{self.number:04d}')+'.json').write_text(json.dumps(output,indent=2))
        return output

    def _capture(self,required,save=True):
        import cv2
        data,labels=self._frame()
        output={};errors={}
        for p in required:
            try: output[p['prim']]=fit_mask(self._mask(data,labels,p['prim']),p['category'],p.get('size'))
            except RuntimeError as exc: errors[p['prim']]=str(exc)
        self.number+=1
        if save:
            stem=self.directory/f'scan_{self.number:04d}'
            if self.save_images:
                rgba=np.asarray(self.rgb.get_data())
                if rgba.ndim==3: cv2.imwrite(str(stem)+'.png',cv2.cvtColor(rgba[:,:,:3],cv2.COLOR_RGB2BGR))
                np.savez_compressed(str(stem)+'_mask.npz',ids=data)
            Path(str(stem)+'.json').write_text(json.dumps(dict(observations=output,errors=errors),indent=2))
        if errors: raise RuntimeError('Scan rejected: '+json.dumps(errors))
        return output

    def stable(self,required,target,tick,q,timeout=20.,samples=5,gap_ticks=15):
        """Require `samples` observations, gap_ticks/30 s apart, spanning <=0.8 mm
        and <=1 degree. Default 5 x 0.5 s; the normal speed profile uses 3 x 1/3 s
        (objects no longer sleep, and the pre-grasp guard still catches a move)."""
        history=[]
        for attempt in range(int(timeout*30/gap_ticks)+1):
            for _ in range(gap_ticks): tick(q,.006)
            observations=self.capture(required,save=(attempt%4==0))
            history.append(observations[target['prim']]);history=history[-samples:]
            if len(history)==samples:
                xy=np.asarray([o['xy'] for o in history])
                travel=float(np.max(np.linalg.norm(xy[:,None,:]-xy[None,:,:],axis=2)))
                rotation=max(angle_error(a['yaw_deg'],b['yaw_deg'],target['category']) for a in history for b in history)
                if travel<=.0008 and rotation<=1.:
                    return observations
            print(f"[scan] Waiting for {target['prim']} to settle ({attempt*gap_ticks/30:.1f}s)",flush=True)
        raise RuntimeError(f"{target['prim']} still moving after {timeout:g}s; no grasp attempted")

    def close(self):
        self.seg.detach(self.product)
        if self.rgb is not None: self.rgb.detach(self.product)
        self.product.destroy()
