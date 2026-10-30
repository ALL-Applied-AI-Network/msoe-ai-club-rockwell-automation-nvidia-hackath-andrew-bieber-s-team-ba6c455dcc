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
from scan_sort_motion import recovery_steps,PARK_XYZ,PARK_SPREAD,trajectory,run_pick,pose_metrics,object_box,boxes_overlap,finger_boxes
from scan_sort_camera import Scanner
import scan_sort_bins as bins
SLOTS=bins.SLOTS

def inventory():
    used=Counter();items=[]
    for kind,count in [('screw',len(C.SORT_SCREWS)),('cube',len(C.SORT_CUBES)),
                       ('vial',len(C.SORT_VIALS)),('cap',len(C.SORT_CAPS))]:
        for index in range(count):
            p=D.pick_spec(kind,index)
            p['color']=C.SORT_CUBES[index][2] if kind=='cube' else None
            p['group']=('cube_large' if p['size']!=C.CUBE_EDGE else 'cube_'+p['color']) if kind=='cube' else kind
            p['place_xy']=list(SLOTS[p['group']][used[p['group']]])
            p['bin']=bins.bin_for(p['group'],used[p['group']])
            if p['bin']: p['place_z']=C.SURFACE_DZ+bins.release_tip_height()
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

def grasp_candidates(pick):
    """Grasp variants tried in order (same set at startup and at runtime).

    Each varies the finger direction at pickup and at placement. The wide-zone
    rail reaches up to 87 mm sideways, so the arm may need to turn the fingers
    to keep it inside a sorting area's walls.
    """
    spread=np.asarray(pick['spread_dir'],float)
    spreads=[spread,-spread]
    if pick['category']=='cap':  # round plug: any direction
        spreads+=[np.array([1.,0.,0.]),np.array([-1.,0.,0.])]
    elif pick['category']=='cube':  # square: the other face pair works too
        perp=np.array([-spread[1],spread[0],0.])
        spreads+=[perp,-perp]
    variants=[dict(pick,force_spread=True,spread_dir=a.copy()) for a in spreads]
    if pick['category'] in ('cube','cap','screw'):  # vials must lie along their slot
        # Explicit placement directions keep their sign (force_place), so the
        # long rail can be pointed toward whichever side has room.
        for a in spreads:
            for b in ((1.,0.,0.),(-1.,0.,0.),(0.,1.,0.),(0.,-1.,0.)):
                variants.append(dict(pick,force_spread=True,spread_dir=a.copy(),
                                     place_spread_dir=np.array(b),force_place=True))
    return variants

def plan_any(pick,previous,obstacles=None):
    """Startup plan: preferred grasp first, then the runtime fallbacks.
    With obstacles, a plan must also pass the wall/object clearance check."""
    failures=[]
    for candidate in [dict(pick)]+grasp_candidates(pick):
        try:
            plan=trajectory(candidate,previous)
            if obstacles is not None: check_path(plan,obstacles)
            return plan
        except RuntimeError as exc: failures.append(str(exc))
    raise RuntimeError(f"No valid grasp for {pick['prim']}: {failures[0]}")

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
                        help='Viewport refresh only; does not change the physics rate')
    parser.add_argument('--physics-hz',type=int,choices=(60,120,240),default=240,
                        help='Physics steps per second (motion timing is unchanged). '
                             '60 Hz let held vials creep out of the grip during long carries')
    parser.add_argument('--save-scan-images',action='store_true',
                        help='Save RGB PNG and mask NPZ diagnostics; JSON is always retained')
    parser.add_argument('--check-only',action='store_true')
    parser.add_argument('--exit-on-finish',action='store_true')
    parser.add_argument('--start-delay',type=float,default=10.)
    parser.add_argument('--force-drop',action='store_true',
                        help='Deliberately drop the first object mid-carry to demonstrate recovery')
    parser.add_argument('--max-recoveries',type=int,default=3,
                        help='Times one object may be dropped/missed and retried before it is '
                             'reported failed (the sort then continues with the other objects)')
    parser.add_argument('--only',choices=('screw','cube','vial','cap'),
                        help='Pick only this category (quick test); other objects stay as obstacles')
    args=parser.parse_args()
    if not np.isfinite(args.start_delay) or args.start_delay<0:
        parser.error('--start-delay must be finite and nonnegative')
    plans=[]; previous=park_pose()
    items=inventory()
    for i,pick in enumerate(items):
        # Earlier objects are already in their destinations; later ones are still out.
        # Objects dropped into a bin are covered by the bin's own walls.
        obstacles=[dict(p,xy=p['place_xy'],scan_yaw_deg=0.) if j<i else p
                   for j,p in enumerate(items) if j!=i and not (j<i and p.get('bin'))]
        plan=plan_any(pick,previous,obstacles)
        previous=plan[1][-1][1][-1][0].copy()
        plans.append(plan)
        print(f"[sort] PLAN {pick['category']} {pick['index']} -> {pick['group']} {pick['place_xy']}; "
              f"IK error {plan[3]*1000:.3f} mm",flush=True)
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
                                 scan_images=args.save_scan_images,physics_hz=args.physics_hz))
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
        # rendering_dt == physics_dt: a rendered step then advances exactly one physics step.
        world=World(stage_units_in_meters=1.,physics_dt=1/args.physics_hz,rendering_dt=1/args.physics_hz)
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
        handles={pick['prim']:SingleXFormPrim(pick['prim'],reset_xform_properties=False) for _,_,pick,_ in plans}
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
            # One motion frame = 1/30 s of physics, split into physics_hz/30 steps.
            substeps=args.physics_hz//30
            for k in range(substeps):
                world.step(render=(k==substeps-1 and not args.headless and tick_count%render_every==0))
            if not args.headless: time.sleep(max(0,1/30-(time.monotonic()-start)))
        initial=plans[0][0]
        arm.set_joint_positions(command(initial,C.JAW_OPEN))  # Initialization only.
        for _ in range(60): tick(initial,C.JAW_OPEN)
        for p in bodies: UsdPhysics.RigidBodyAPI(p).CreateKinematicEnabledAttr().Set(False)
        for _ in range(max(30,round(args.start_delay*30))): tick(initial,C.JAW_OPEN)
        baseline={pick['prim']:D.centre(*handles[pick['prim']].get_world_pose(),pick['category'],pick.get('size'))
                  for _,_,pick,_ in plans}
        items=[p for _,_,p,_ in plans]
        plans.clear()
        executed=[]
        park_q=initial.copy()  # the scan pose every pick starts from and returns to
        current_q=park_q.copy()
        pending=[p for p in items if args.only in (None,p['category'])]
        object_count=len(pending); report['expected_objects']=object_count
        report['scheduling_events']=[]
        report['recoveries']=[]; report['failed_objects']=[]
        failures_by_object=Counter(); abandoned=set()
        number=0

        def recover(prim,category,reason):
            '''Count a failure; back off to the scan pose; give up after the limit.'''
            nonlocal current_q,pending
            failures_by_object[prim]+=1
            count=failures_by_object[prim]
            give_up=count>args.max_recoveries
            report['recoveries'].append(dict(object=prim,failure=count,reason=reason,gave_up=give_up))
            print(f"[recover] {prim}: {reason}",flush=True)
            if give_up:
                print(f"[recover] {prim}: failed {count} times; giving up on it and "
                      f"continuing with the other objects",flush=True)
                abandoned.add(prim); report['failed_objects'].append(prim)
                pending=[p for p in pending if p['prim']!=prim]
            else:
                print(f"[recover] Backing off to the scan pose to rescan and retry "
                      f"({count}/{args.max_recoveries})",flush=True)
            for q,jaw in recovery_steps(measured()[arm_idx],park_q,category): tick(q,jaw)
            for _ in range(30): tick(park_q,C.JAW_OPEN)  # let a dropped object settle
            current_q=park_q.copy(); save()

        while pending:
            number+=1
            # Leave out objects given up on (poses the scan may not read) and objects
            # already dropped into a bin (walls can hide them; the bin is the obstacle).
            binned={p['prim'] for p in executed if p.get('bin')}
            active=[p for p in items if p['prim'] not in abandoned and p['prim'] not in binned]
            def attempt(template):
                observations=scanner.stable(active,template,tick,current_q)
                snapshot=[from_scan(p,observations[p['prim']]) for p in active]
                pick=next(p for p in snapshot if p['prim']==template['prim'])
                obstacles=[p for p in snapshot if p['prim']!=pick['prim']]
                failures=[]
                for candidate in grasp_candidates(pick):
                    try:
                        plan=trajectory(candidate,current_q)
                        check_path(plan,obstacles)
                        return plan,observations
                    except RuntimeError as exc: failures.append(str(exc))
                raise BlockedGrasp(failures)
            try:
                plan,observations=choose_next(pending,attempt,report['scheduling_events'])
            except RuntimeError as exc:
                # A dropped object may not settle, or land in a pose the scan cannot read.
                # Count that as a failed attempt for objects that have already failed once.
                text=str(exc)
                culprits=[p for p in pending if failures_by_object[p['prim']] and p['prim'] in text]
                if not culprits: raise
                for p in culprits: recover(p['prim'],p['category'],'not found by the rescan: '+text[:160])
                continue
            pick=plan[2]
            print(f"\n[scan] OBJECT {number}/{object_count}: {pick['group']} {pick['index']}",flush=True)
            save()
            print(f"[scan] Fresh target XY={pick['xy']}, yaw={pick['scan_yaw_deg']:.2f} deg",flush=True)
            def phase_guard(phase):
                # Evaluation-only GT guard: can abort, never correct a command.
                if phase in ('descend','close'):
                    position,quat=handles[pick['prim']].get_world_pose()
                    centre=D.centre(position,quat,pick['category'],pick.get('size'))
                    delta=(centre[:2]-np.asarray(pick['xy']))*1000
                    if np.linalg.norm(delta)>1.5:
                        # Evaluation only: which objects moved since the start, and by how much.
                        moved=[]
                        for p in items:
                            c=D.centre(*handles[p['prim']].get_world_pose(),p['category'],p.get('size'))
                            shift=(c-baseline[p['prim']])*1000
                            if np.linalg.norm(shift[:2])>1.0:
                                moved.append(f"{p['prim']} dx={shift[0]:.1f} dy={shift[1]:.1f} mm")
                        rotation,flange=fk.fk(measured()[arm_idx])
                        print('[guard] Objects moved since start: '+('; '.join(moved) or 'none'),flush=True)
                        print(f'[guard] Flange now {np.round(flange,3).tolist()}, '
                              f'tool axis {np.round(rotation@np.asarray(C.TOOL_AXIS),3).tolist()}',flush=True)
                        raise RuntimeError(f'Object moved after scan (before {phase}): true centre '
                            f'{np.round(centre[:2],4).tolist()} vs scanned {np.round(pick["xy"],4).tolist()}, '
                            f'off by dx={delta[0]:.2f} dy={delta[1]:.2f} mm; rescanning')
            item=run_pick(app,world,arm,arm_idx,tick,measured,plan,
                          directory/f"{number:02d}_{pick['category']}_{pick['index']}.json",phase_hook=phase_guard,
                          force_drop=args.force_drop and number==1)
            item['scan']=observations[pick['prim']]
            item['attempt']=failures_by_object[pick['prim']]+1
            report['objects'].append(item); save()
            if not item['complete']:
                # Simulator pose checks only TRIGGER the retry; the new grasp target
                # always comes from a fresh camera scan.
                recover(pick['prim'],pick['category'],item.get('error') or 'pickup/placement not verified')
                continue
            current_q=plan[1][-1][1][-1][0].copy()
            executed.append(pick)
            pending=[p for p in pending if p['prim']!=pick['prim']]
            # Later motions (or a dropped object) may knock sorted objects out of place:
            # put any that moved back in the queue to be sorted again.
            audit=[]
            for p in list(executed):
                metrics=pose_metrics(handles[p['prim']],p,baseline[p['prim']][2])
                audit.append(dict(object=p['category'],index=p['index'],group=p['group'],**metrics))
                if not metrics['verified']:
                    print(f"[recover] Sorted {p['prim']} was knocked out of its slot; re-queueing it",flush=True)
                    executed.remove(p); pending.append(p)
                    failures_by_object[p['prim']]+=1
                    report['recoveries'].append(dict(object=p['prim'],failure=failures_by_object[p['prim']],
                                                     reason='knocked out of its slot',gave_up=False))
            report['final_audit']=audit; save()
        # Additional settling and a final audit after every object has been moved.
        for _ in range(60): tick(current_q,C.JAW_OPEN)
        report['final_audit']=[dict(object=p['category'],index=p['index'],group=p['group'],
            **pose_metrics(handles[p['prim']],p,baseline[p['prim']][2])) for p in executed]
        report['placed_objects']=len(executed)
        report['complete']=bool(len(executed)==object_count and not report['failed_objects']
            and all(r['verified'] for r in report['final_audit']))
    except Exception as exc:
        report['error']=str(exc); traceback.print_exc()
    finally:
        save()
        placed=report.get('placed_objects',sum(r['complete'] for r in report['objects']))
        print(f"[sort] {'PASS' if report['complete'] else 'FAIL'}: "
              f"{placed}/{report['expected_objects']} objects placed; "
              f"{len(report.get('recoveries',[]))} recoveries",flush=True)
        if report.get('failed_objects'):
            print(f"[sort] Gave up on: {', '.join(report['failed_objects'])}",flush=True)
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
