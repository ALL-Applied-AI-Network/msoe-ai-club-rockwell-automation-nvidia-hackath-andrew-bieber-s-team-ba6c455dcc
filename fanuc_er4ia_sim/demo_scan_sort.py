"""Rescan before each pickup, cubes -> vials -> caps, five enclosed areas.
Simulation-assisted camera segmentation, NOT learned RGB perception.
Run demo_scan_sort_spaced.py from fanuc_er4ia_sim for the separated layout.
"""
import argparse,json,time,traceback,os
from pathlib import Path
from datetime import datetime
from collections import Counter
os.environ.setdefault('FINGER_SET', 'dual')
import numpy as np
import scene_config as C
import dual_zone as D
import fk,ik
import scan_sort_motion as motion
from scan_sort_motion import PARK_XYZ,PARK_SPREAD,trajectory,run_pick,pose_metrics,object_box,boxes_overlap,finger_boxes
from scan_sort_camera import Scanner
import scan_sort_bins as bins
SLOTS=bins.SLOTS

def inventory():
    used=Counter();items=[]
    for kind,count in [('cube',len(C.SORT_CUBES)),('vial',len(C.SORT_VIALS)),('cap',len(C.SORT_CAPS))]:
        for index in range(count):
            p=D.pick_spec(kind,index)
            p['color']=C.SORT_CUBES[index][2] if kind=='cube' else None
            p['group']='cube_'+p['color'] if kind=='cube' else kind
            p['place_xy']=list(SLOTS[p['group']][used[p['group']]])
            used[p['group']]+=1;items.append(p)
    return items

def park_pose():
    ik.set_tool_axis(C.TOOL_AXIS);ik.set_spread_axis(C.FINGER_SPREAD_AXIS)
    limits=np.deg2rad([C.JOINT_LIMITS_DEG[f'J{i}'] for i in range(1,7)])
    q=np.deg2rad([C.HOME_POSE_DEG[f'J{i}'] for i in range(1,7)])
    q,ok=ik.solve(q,np.array(PARK_XYZ),np.array([0.,0.,-1.]),limits,
                  tool_offset=D.tool_offset('cube'),spread_dir=np.array(PARK_SPREAD),iters=500)
    if not ok: raise RuntimeError('Scan parking pose IK failed')
    spread=fk.fk(q)[0]@np.asarray(C.FINGER_SPREAD_AXIS); spread[2]=0
    motion.set_park_spread(spread/np.linalg.norm(spread))
    return q

def grasp_directions(pick):
    """Finger directions tried for a grasp, in order (same set at startup and runtime)."""
    directions=[pick['spread_dir'],-pick['spread_dir']]
    # Round cap plugs permit an additional pair of perpendicular grasps.
    if pick['category']=='cap':
        directions+=[np.array([1.,0.,0.]),np.array([-1.,0.,0.])]
    return directions

def plan_any(pick,previous):
    """Startup plan: preferred direction first, then the runtime fallbacks."""
    try: return trajectory(pick,previous)
    except RuntimeError as first:
        for direction in grasp_directions(pick):
            try: return trajectory(dict(pick,force_spread=True,spread_dir=direction.copy()),previous)
            except RuntimeError: pass
        raise first

def from_scan(template,observation):
    p=dict(template)
    p['xy']=list(observation['xy'])
    p['scan_yaw_deg']=observation['yaw_deg']
    yaw=np.deg2rad(p['scan_yaw_deg'])
    p['spread_dir']=np.array([-np.sin(yaw),np.cos(yaw),0.])
    return p

def check_path(plan,obstacles):
    walls=[(f'{g}/{name}',(center,np.eye(3),size/2)) for g,name,center,size,_ in bins.walls()]
    obstacles=[(p['prim'],object_box(p)) for p in obstacles]+walls
    # Cache obstacle AABBs; run the unchanged OBB test only for nearby pairs.
    obstacle_centres=np.array([box[0] for _,box in obstacles]).reshape(-1,3)
    obstacle_extents=np.array([np.abs(box[1])@box[2] for _,box in obstacles]).reshape(-1,3)
    initial,phases,pick,_=plan
    close_q=next(steps[-1][0] for name,steps in phases if name=='close')
    r0,p0=fk.fk(close_q)
    oc,orr,oh=object_box(pick)
    offset=r0.T@(oc-p0);relative=r0.T@orr
    for phase,steps in phases:
        for q,jaw,_ in steps:
            boxes=list(finger_boxes(q,jaw))
            if phase in ('lift','carry','lower'):
                r,p=fk.fk(q)
                boxes.append(('carried object',(p+r@offset,r@relative,oh)))
            for name,box in boxes:
                centre,rotation,half=box
                extent=np.abs(rotation)@half
                nearby=np.flatnonzero(np.all(np.abs(obstacle_centres-centre)
                    <=obstacle_extents+extent+0.002,axis=1))
                for index in nearby:
                    label,other=obstacles[index]
                    if boxes_overlap(box,other):
                        raise RuntimeError(f'Path rejected: {pick["prim"]} {phase}: {name} near {label}')



# BLOCKER_FIRST_SORT_V1
class BlockedGrasp(RuntimeError):
    def __init__(self, failures):
        super().__init__('; '.join(failures))
        self.failures = failures


def choose_next(pending, attempt, events):
    """Try preferred target, then its unsorted blockers; detect dependency cycles.
    attempt must rescan/stabilize and validate both IK and clearance for each
    candidate. Only a collision rejection permits changing the object order.
    """
    import re
    lookup = {p['prim']: p for p in pending}
    rejected = {}
    def visit(target, ancestors):
        key = target['prim']
        if key in ancestors or key in rejected:
            return None
        try:
            return attempt(target)
        except BlockedGrasp as exc:
            rejected[key] = exc.failures
            blockers = []
            for failure in exc.failures:
                for blocker in re.findall(r'near (/World/Sort(?:Cube|Vial|Cap)\d+)', failure):
                    if blocker in lookup and blocker not in blockers:
                        blockers.append(blocker)
            for blocker in blockers:
                event = dict(blocked=key, try_first=blocker, reasons=exc.failures)
                events.append(event)
                print(f'[schedule] {key} blocked; checking {blocker} first', flush=True)
                result = visit(lookup[blocker], ancestors | {key})
                if result is not None:
                    return result
            return None
    result = visit(pending[0], set())
    if result is None:
        raise RuntimeError('No clear grasp for preferred object or its blockers: ' + str(rejected))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--viewport-fps',type=int,choices=(10,15,30),default=15,
                        help='Viewport refresh only; physics stays at 60 Hz')
    parser.add_argument('--save-scan-images',action='store_true',
                        help='Save RGB PNG and mask NPZ diagnostics; JSON is always retained')
    parser.add_argument('--check-only',action='store_true')
    parser.add_argument('--exit-on-finish',action='store_true')
    parser.add_argument('--start-delay',type=float,default=10.)
    parser.add_argument('--only',choices=('cube','vial','cap'),
                        help='Pick only this category (quick test); other objects stay as obstacles')
    args=parser.parse_args()
    if not np.isfinite(args.start_delay) or args.start_delay<0:
        parser.error('--start-delay must be finite and nonnegative')
    plans=[]; previous=park_pose()
    for pick in inventory():
        plan=plan_any(pick,previous)
        previous=plan[1][-1][1][-1][0].copy()
        plans.append(plan)
        print(f"[sort] PLAN {pick['category']} {pick['index']} -> {pick['group']} {pick['place_xy']}; "
              f"IK error {plan[3]*1000:.3f} mm",flush=True)
    for i,plan in enumerate(plans):
        obstacles=[dict(p,xy=p['place_xy'],scan_yaw_deg=0.) if j<i else p
                   for j,(_,_,p,_) in enumerate(plans) if j!=i]
        check_path(plan,obstacles)
    print('[scan] Nominal IK and wall/object clearance checks passed.',flush=True)
    if args.check_only:
        print('[sort] Host checks passed; physics and full-arm collisions remain unverified.',flush=True)
        return 0
    directory=Path(__file__).resolve().parent/'runs'/('scan_sort_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    directory.mkdir(parents=True)
    report=dict(test='continuous_synthetic_camera_sort',perception_used=True,
                classification_source='simulator semantic labels',pose_source='rendered mask fit',complete=False,error=None,objects=[],
                expected_objects=len(plans),final_audit=[],destinations=SLOTS,
                performance=dict(viewport_fps=0 if args.headless else args.viewport_fps,
                                 viewport_size=[960,540],scan_resolution=3072,
                                 scan_images=args.save_scan_images,physics_hz=60))
    path=directory/'sort_report.json'
    def save(): path.write_text(json.dumps(report,indent=2)+'\n')
    save()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':args.headless,'limit_cpu_threads':2,
                       'renderer':'RaytracedLighting','width':960,'height':540,
                       'disable_viewport_updates':args.headless})
    world=None; scanner=None
    try:
        import omni.usd
        from pxr import UsdPhysics
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation,SingleXFormPrim
        from isaacsim.core.utils.types import ArticulationAction
        import cell
        world=World(stage_units_in_meters=1.,physics_dt=1/60,rendering_dt=1/30)
        stage=omni.usd.get_context().get_stage()
        cell.build(stage)
        if not stage.GetPrimAtPath('/World/SolidWorkSurface').IsValid():
            raise RuntimeError('Install fix_work_surface.py before running this demo')
        if not np.isclose(cell.tip_offset(),C.TOOL_LENGTH): raise RuntimeError('TCP mismatch')
        bins.build(stage)
        scanner=Scanner(stage,[p for _,_,p,_ in plans],directory/"scans",save_images=args.save_scan_images)
        if not args.headless:
            from isaacsim.core.utils.viewports import set_camera_view
            set_camera_view(eye=np.array([1.05,.80,1.65]),target=np.array([.25,.04,1.04]))
        bodies=[p for p in stage.Traverse() if str(p.GetPath()).startswith(
            ('/World/SortCube','/World/SortCap','/World/SortVial')) and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        for p in bodies: UsdPhysics.RigidBodyAPI(p).CreateKinematicEnabledAttr().Set(True)
        cell.enable_ccd(world)
        world.reset()  # Exactly once, before the first pickup.
        arm=SingleArticulation(prim_path=C.ROBOT_PRIM,name='sort_er4ia')
        arm.initialize()
        names=list(arm.dof_names)
        arm_idx=[names.index(f'J{i}') for i in range(1,7)]
        joints={name:i for i,name in enumerate(names)}
        handles={pick['prim']:SingleXFormPrim(pick['prim']) for _,_,pick,_ in plans}
        def measured():
            values=arm.get_joint_positions()
            if values is None: raise RuntimeError('Physics view unavailable; do not press Stop during sorting')
            return values.copy()
        def command(q,jaw):
            values=measured(); values[arm_idx]=q
            for name,value in cell.jaw_targets(jaw).items(): values[joints[name]]=value
            arm.apply_action(ArticulationAction(joint_positions=values))
            return values
        tick_count=0
        render_every=30//args.viewport_fps
        def tick(q,jaw):
            nonlocal tick_count
            if not app.is_running() or not world.is_playing(): raise RuntimeError('Sorting interrupted')
            start=time.monotonic()
            command(q,jaw)
            tick_count+=1
            world.step(render=False)
            world.step(render=(not args.headless and tick_count%render_every==0))
            if not args.headless: time.sleep(max(0,1/30-(time.monotonic()-start)))
        initial=plans[0][0]
        arm.set_joint_positions(command(initial,C.JAW_OPEN))  # Initialization only.
        for _ in range(60): tick(initial,C.JAW_OPEN)
        for p in bodies: UsdPhysics.RigidBodyAPI(p).CreateKinematicEnabledAttr().Set(False)
        for _ in range(max(30,round(args.start_delay*30))): tick(initial,C.JAW_OPEN)
        baseline={pick['prim']:D.centre(*handles[pick['prim']].get_world_pose(),pick['category'])
                  for _,_,pick,_ in plans}
        items=[p for _,_,p,_ in plans]
        plans.clear()
        executed=[]
        current_q=initial.copy()
        pending=[p for p in items if args.only in (None,p['category'])]
        object_count=len(pending); report['expected_objects']=object_count
        report['scheduling_events']=[]
        while pending:
            number=len(executed)+1
            def attempt(template):
                observations=scanner.stable(items,template,tick,current_q)
                snapshot=[from_scan(p,observations[p['prim']]) for p in items]
                pick=next(p for p in snapshot if p['prim']==template['prim'])
                obstacles=[p for p in snapshot if p['prim']!=pick['prim']]
                failures=[]
                for direction in grasp_directions(pick):
                    candidate=dict(pick,force_spread=True,spread_dir=direction.copy())
                    try:
                        plan=trajectory(candidate,current_q)
                        check_path(plan,obstacles)
                        return plan,observations
                    except RuntimeError as exc: failures.append(str(exc))
                raise BlockedGrasp(failures)
            plan,observations=choose_next(pending,attempt,report['scheduling_events'])
            pick=plan[2]
            print(f"\n[scan] OBJECT {number}/{object_count}: {pick['group']} {pick['index']}",flush=True)
            save()
            print(f"[scan] Fresh target XY={pick['xy']}, yaw={pick['scan_yaw_deg']:.2f} deg",flush=True)
            def phase_guard(phase):
                # Evaluation-only GT guard: can abort, never correct a command.
                if phase in ('descend','close'):
                    position,quat=handles[pick['prim']].get_world_pose()
                    centre=D.centre(position,quat,pick['category'])
                    if np.linalg.norm(centre[:2]-pick['xy'])>.0015:
                        raise RuntimeError('Object moved after scan; stop and rescan on a new run')
            item=run_pick(app,world,arm,arm_idx,tick,measured,plan,
                          directory/f"{number:02d}_{pick['category']}_{pick['index']}.json",phase_hook=phase_guard)
            item['scan']=observations[pick['prim']]
            executed.append(pick)
            current_q=plan[1][-1][1][-1][0].copy()
            report['objects'].append(item); save()
            if not item['complete']: raise RuntimeError(f"Pickup/placement failed for {pick['prim']}")
            pending=[p for p in pending if p['prim']!=pick['prim']]
            # Later motions must not knock previously sorted objects out of place.
            audit=[]
            for p in executed:
                metrics=pose_metrics(handles[p['prim']],p,baseline[p['prim']][2])
                audit.append(dict(object=p['category'],index=p['index'],group=p['group'],**metrics))
            report['final_audit']=audit; save()
            if not all(a['verified'] for a in audit): raise RuntimeError('Previously sorted object moved out of tolerance')
        # Additional settling and a final audit after every object has been moved.
        for _ in range(60): tick(current_q,C.JAW_OPEN)
        report['final_audit']=[dict(object=p['category'],index=p['index'],group=p['group'],
            **pose_metrics(handles[p['prim']],p,baseline[p['prim']][2])) for p in executed]
        report['complete']=bool(len(report['objects'])==object_count
            and all(r['complete'] for r in report['objects'])
            and all(r['verified'] for r in report['final_audit']))
    except Exception as exc:
        report['error']=str(exc); traceback.print_exc()
    finally:
        save()
        print(f"[sort] {'PASS' if report['complete'] else 'FAIL'}: "
              f"{sum(r['complete'] for r in report['objects'])}/{report['expected_objects']} individual placements",flush=True)
        print(f'[sort] Report: {path}',flush=True)
        try:
            if world is not None and app.is_running(): world.pause()
            if not args.headless and not args.exit_on_finish and app.is_running():
                print('[sort] Scene retained. Close the window when finished.',flush=True)
                while app.is_running(): app.update(); time.sleep(.02)
        finally:
            try:
                if scanner is not None: scanner.close()
            finally: app.close()
    return 0 if report['complete'] else 1


if __name__=='__main__': raise SystemExit(main())
