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


def fit_mask(mask,kind):
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
    cube_range = (C.CUBE_EDGE-.003, C.CUBE_EDGE+.003)*2
    ranges={'cube':cube_range,'vial':vial_range,'cap':(.028,.035,.028,.035)}
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
    pixels=np.column_stack(((corners[:,0]-CENTER[0])/WIDTH*RES+RES/2-.5,
                            RES/2-(corners[:,1]-CENTER[1])/WIDTH*RES-.5))
    if kind=='cap':
        pixel_center=((np.array(xy)-CENTER)/WIDTH*RES*np.array([1,-1])+RES/2-.5).round().astype(int)
        cv2.circle(expected,tuple(pixel_center-np.array([x0,y0])),round((long+short)/4/WIDTH*RES),1,-1)
    else:
        cv2.fillConvexPoly(expected,np.round(pixels).astype(int)-np.array([x0,y0]),1)
    iou=float(np.logical_and(cropped,expected).sum()/max(1,np.logical_or(cropped,expected).sum()))
    if iou<.85: raise RuntimeError(f'{kind}: silhouette fit IoU {iou:.3f} below .85')
    if kind=='cube': yaw=(yaw+45)%90-45
    elif kind=='vial': yaw=(yaw+90)%180-90
    else: yaw=0.
    return dict(xy=list(map(float,xy)),yaw_deg=float(yaw),pixels=int(len(rows)),silhouette_iou=iou,
                measured_length_mm=float(long*1000),measured_width_mm=float(short*1000))


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

    def _capture(self,required,save=True):
        import cv2
        # Render without advancing physics; leave timeline playing.
        self.rep.orchestrator.step(delta_time=0.0,pause_timeline=False)
        self.rep.orchestrator.step(delta_time=0.0,pause_timeline=False)
        raw=self.seg.get_data()
        if not isinstance(raw,dict) or 'data' not in raw: raise RuntimeError('Segmentation camera returned no frame')
        data=np.asarray(raw['data']).squeeze()
        if data.shape!=(RES,RES): raise RuntimeError(f'Unexpected segmentation frame shape {data.shape}')
        labels=raw.get('info',{}).get('idToLabels',{})
        output={};errors={}
        for p in required:
            tag=self.tags[p['prim']]
            ids=[]
            for identity,values in labels.items():
                value=values.get('class','') if isinstance(values,dict) else values
                tokens=value if isinstance(value,list) else str(value).split(',')
                if tag in [str(v).strip() for v in tokens]: ids.append(int(identity))
            try: output[p['prim']]=fit_mask(np.isin(data,ids),p['category'])
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

    def stable(self,required,target,tick,q,timeout=20.):
        """Require 2 s of observations spanning <=0.8 mm and <=1 degree."""
        history=[]
        for attempt in range(int(timeout*2)+1):
            for _ in range(15): tick(q,.006)
            observations=self.capture(required,save=(attempt%4==0))
            history.append(observations[target['prim']]);history=history[-5:]
            if len(history)==5:
                xy=np.asarray([o['xy'] for o in history])
                travel=float(np.max(np.linalg.norm(xy[:,None,:]-xy[None,:,:],axis=2)))
                rotation=max(angle_error(a['yaw_deg'],b['yaw_deg'],target['category']) for a in history for b in history)
                if travel<=.0008 and rotation<=1.:
                    return observations
            print(f"[scan] Waiting for {target['prim']} to settle ({attempt/2:.1f}s)",flush=True)
        raise RuntimeError(f"{target['prim']} still moving after {timeout:g}s; no grasp attempted")

    def close(self):
        self.seg.detach(self.product)
        if self.rgb is not None: self.rgb.detach(self.product)
        self.product.destroy()
