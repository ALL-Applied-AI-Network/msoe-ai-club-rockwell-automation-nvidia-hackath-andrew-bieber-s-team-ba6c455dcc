"""Continuous known-position sorting: one scene, ten objects, five groups.
Colors/types come from scene configuration, NOT camera recognition.
Run with Isaac Python; --check-only runs host planning/clearance checks only.
"""
import argparse
import json
import os
import time
import traceback
from collections import Counter
from datetime import datetime
from pathlib import Path
os.environ.setdefault("FINGER_SET", "dual")
import numpy as np
import scene_config as C
import dual_zone as D
import fk
import ik
from demo_dual_zone import ease, turn, segments as original_segments

# Two separate slots per group, metres in robot-base XY.
from scan_sort_bins import SLOTS

# Scan park pose: tool 300 mm above (0.08, 0), fingers closing along X.
# Keeps the arm and wrist camera out of the overhead camera's view of every
# object, slot and sorting area (the old pose hid one end of the 203 mm vial).
PARK_XYZ = (0.08, 0.0, 0.30)
PARK_SPREAD = (1.0, 0.0, 0.0)  # demo_scan_sort replaces this with the solved sign


def set_park_spread(direction):
    """Pin the exact park finger direction. Every pick returns to it, so the
    arm (and an off-centre vial TCP) ends at the same park pose each time."""
    global PARK_SPREAD
    PARK_SPREAD = tuple(float(v) for v in direction)

COLORS = {"cube_yellow": (0.95,0.8,0.05), "cube_red": (0.9,0.12,0.1),
          "cube_blue": (0.12,0.35,0.9), "vial": (0.1,0.8,0.8), "cap": (0.8,0.3,0.8),
          "screw": (0.35,0.35,0.38), "cube_large": (0.2,0.5,0.9)}


# Peak carry acceleration limit (m/s^2). A cap carried 0.6 m in 2.8 s
# (~0.46 m/s^2) tipped out of its point-contact grip.
CARRY_MAX_ACCEL = 0.10


def segments(pick, home):
    for phase in original_segments(pick, home):
        name,p0,p1,duration,j0,j1,direction = phase
        if pick['category'] == 'cap':
            # Pads touch the round plug at a point: gentle close, dwell and lift.
            duration = {'close':1.0, 'dwell':1.0, 'lift':1.6}.get(name,duration)
        if pick['category'] == 'vial':
            # trajectory uses two simulation seconds per duration unit.
            # Slower close/dwell lets grip force build before the vial is loaded;
            # a slower lift halves peak acceleration on the long, off-centre vial.
            # Carry 10 s (was 6 s): slip began at peak carry acceleration.
            duration = {'close':1.0, 'dwell':1.0, 'lift':1.6, 'carry':5.0,
                        'lower':1.2, 'release':1.2, 'settle':3.0}.get(name,duration)
        if name == 'carry':
            # Smoothstep peak acceleration is 6 D / T^2; stretch T to respect the limit.
            # (Two simulation seconds per duration unit.)
            distance = float(np.linalg.norm(np.asarray(p1)-np.asarray(p0)))
            duration = max(duration, np.sqrt(6*distance/CARRY_MAX_ACCEL)/2)
        if name == 'home':
            # Return with the wrist in the park orientation so the camera stays clear.
            direction = np.array(PARK_SPREAD)
        yield name,p0,p1,duration,j0,j1,direction


def inventory():
    used = Counter()
    result = []
    for kind, count in (("screw",len(C.SORT_SCREWS)), ("cube",len(C.SORT_CUBES)),
                        ("vial",len(C.SORT_VIALS)), ("cap",len(C.SORT_CAPS))):
        for index in range(count):
            pick = D.pick_spec(kind,index)
            group = (("cube_large" if pick["size"] != C.CUBE_EDGE else "cube_"+C.SORT_CUBES[index][2])
                     if kind == "cube" else kind)
            pick["group"] = group
            pick["color"] = C.SORT_CUBES[index][2] if kind == "cube" else None
            pick["place_xy"] = list(SLOTS[group][used[group]])
            used[group] += 1
            result.append(pick)
    return result


def trajectory(pick, previous=None):
    category = pick["category"]
    ik.set_tool_axis(C.TOOL_AXIS)
    ik.set_spread_axis(C.FINGER_SPREAD_AXIS)
    tool = D.tool_offset(category)
    limits = np.deg2rad([C.JOINT_LIMITS_DEG[f"J{i}"] for i in range(1, 7)])
    home = np.array([0.30, 0.0, 0.24])
    down = np.array([0.0, 0.0, -1.0])
    if previous is None:
        q = np.deg2rad([C.HOME_POSE_DEG[f"J{i}"] for i in range(1,7)])
        q, ok = ik.solve(q, home, down, limits, tool_offset=tool)
        if not ok: raise RuntimeError("Home IK failed")
    else:
        q = previous.copy()
        # Start exactly at the preceding arm endpoint, including when TCP changes.
        home = fk.tool_pose(q, tool)[1].copy()
    initial = q.copy()
    # Choose a symmetric spread direction nearest the home orientation.
    if not pick.get("force_spread",False) and np.dot(fk.fk(q)[0] @ np.asarray(C.FINGER_SPREAD_AXIS), pick["spread_dir"]) < 0:
        pick["spread_dir"] = -pick["spread_dir"]
    if not pick.get("force_place", False) and np.dot(pick["spread_dir"], pick["place_spread_dir"]) < 0:
        pick["place_spread_dir"] = -pick["place_spread_dir"]
    phases = []
    worst = 0.0
    for name, p0, p1, duration, j0, j1, direction in segments(pick, home):
        start = None
        if direction is not None and name in ("approach", "carry", "home"):
            start = fk.fk(q)[0] @ np.asarray(C.FINGER_SPREAD_AXIS)
            start[2] = 0
            start /= np.linalg.norm(start)
        steps = []
        # Slower than the starter so the motion is easy to watch.
        count = round(duration * 2 * 30)
        for i in range(count):
            t = ease((i + 1) / count)
            target = p0 + (p1 - p0) * t
            desired = turn(start, direction, t) if start is not None else direction
            q, ok = ik.solve(q, target, down, limits, tool_offset=tool,
                             spread_dir=desired)
            if not ok:
                q, ok = ik.solve(q, target, down, limits, tool_offset=tool,
                                 spread_dir=desired, iters=500, damping=0.02)
            rotation, actual = fk.tool_pose(q, tool)
            error = float(np.linalg.norm(actual - target))
            worst = max(worst, error)
            aligned = desired is None or abs(np.dot(
                rotation @ np.asarray(C.FINGER_SPREAD_AXIS), desired)) > 0.999
            approach_ok = np.dot(rotation @ np.asarray(C.TOOL_AXIS), down) > 0.999
            if not ok or error > 0.001 or not aligned or not approach_ok:
                raise RuntimeError(f"IK rejected {name} frame {i}: {error*1000:.3f} mm")
            steps.append((q.copy(), float(j0 + (j1-j0)*t), target.copy()))
        phases.append((name, steps))
    return initial, phases, pick, worst


def run_pick(app, world, arm, arm_idx, tick, measured, plan, output, phase_hook=None):
    from isaacsim.core.prims import SingleXFormPrim
    initial, phases, pick, worst = plan
    # reset_xform_properties=False: the default rewrites the prim's xform from the
    # stale USD spawn pose, which teleported a resting vial and set it rolling.
    cube = SingleXFormPrim(pick["prim"], reset_xform_properties=False)
    tool = D.tool_offset(pick["category"])
    report = dict(object=pick["category"], index=pick["index"], group=pick["group"],
                  color=pick["color"], destination_xy_m=pick["place_xy"],
                  perception_used=True, perception_mode="synthetic_segmentation_camera", pickup_verified=False, carry_verified=False,
                  release_verified=False, placement_verified=False, complete=False,
                  error=None, max_numerical_ik_error_mm=worst*1000)
    try:
        def object_centre():
            position, quat = cube.get_world_pose()
            return D.centre(position, quat, pick["category"], pick.get("size"))

        def in_gripper():
            """Object centre and long axis in the flange frame (diagnostics only)."""
            position, quat = cube.get_world_pose()
            w, x, y, z = quat
            axis = np.array([2*(x*z+y*w), 2*(y*z-x*w), 1-2*(x*x+y*y)])
            centre = D.centre(position, quat, pick["category"], pick.get("size")) - [0, 0, C.TABLE_TOP_Z]
            rotation, flange = fk.fk(measured()[arm_idx])
            return rotation.T @ (centre-flange), rotation.T @ axis

        grip_reference = None
        before = object_centre().copy()
        report["initial_object_centre_world_m"] = before.tolist()
        # Evaluation gate only: never use measured pose to adjust grasp commands.
        if np.linalg.norm(before[:2] - pick["xy"]) > 0.002:
            raise RuntimeError("Object moved away from the configured test position; aborting")
        for name, steps in phases:
            if phase_hook is not None: phase_hook(name)
            print(f"[demo] {name.upper()}", flush=True)
            for frame, (q, jaw, target) in enumerate(steps):
                tick(q, jaw)
                # Frame-level grip trace (diagnostics only): log every 15 frames, and
                # the first frame the object turns >2 deg in the fingers.
                if grip_reference is not None and name in ("lift", "carry", "lower") and (
                        frame % 15 == 0 or frame == len(steps)-1):
                    offset, axis = in_gripper()
                    turn_deg = float(np.degrees(np.arccos(np.clip(abs(axis@grip_reference[1]), -1, 1))))
                    lag = float(np.degrees(np.max(np.abs(measured()[arm_idx]-q))))
                    report.setdefault("grip_trace", []).append(dict(
                        phase=name, frame=frame, of=len(steps), rotation_deg=turn_deg,
                        slip_mm=float(np.linalg.norm(offset-grip_reference[0]))*1000,
                        max_joint_lag_deg=lag))
                    if turn_deg > 2 and not report.get("grip_first_turn"):
                        report["grip_first_turn"] = report["grip_trace"][-1]
                        print(f"[grip] Object turned {turn_deg:.1f} deg in the fingers at {name} "
                              f"frame {frame}/{len(steps)}; max joint lag {lag:.2f} deg", flush=True)
            report.setdefault("phase_object_centres_world_m", {})[name] = object_centre().tolist()
            # How far the object shifted/rotated inside the closed fingers.
            if name == "dwell":
                grip_reference = in_gripper()
            elif grip_reference is not None and name in ("lift", "carry", "lower"):
                offset, axis = in_gripper()
                slip = float(np.linalg.norm(offset-grip_reference[0]))*1000
                turn_deg = float(np.degrees(np.arccos(np.clip(abs(axis@grip_reference[1]), -1, 1))))
                report.setdefault("grip_slip_mm", {})[name] = slip
                report.setdefault("grip_rotation_deg", {})[name] = turn_deg
                print(f"[demo] In-grip slip {slip:.1f} mm, rotation {turn_deg:.1f} deg", flush=True)
            actual_tip = fk.tool_pose(measured()[arm_idx], tool)[1]
            error = float(np.linalg.norm(actual_tip-target))
            report.setdefault("tip_errors_mm", {})[name] = error*1000
            if name in ("descend", "lower") and error > 0.003:
                raise RuntimeError(f"Arm did not reach {name} target: {error*1000:.2f} mm error")
            if name == "lift":
                height = float(object_centre()[2]-before[2])
                report["lift_height_mm"] = height*1000
                report["pickup_verified"] = height > 0.025
                print(f"[demo] Object rose {height*1000:.1f} mm; pickup={report['pickup_verified']}", flush=True)
                if not report["pickup_verified"]:
                    raise RuntimeError("Object did not lift; stopping before carry")
            if name == "carry":
                pos = object_centre()
                report["carry_verified"] = bool(pos[2]-before[2] > 0.025
                    and np.linalg.norm(pos[:2]-pick["place_xy"]) < 0.005)
                if not report["carry_verified"]:
                    for row in report.get("grip_trace", []):
                        print(f"[grip] {row['phase']:5} {row['frame']:3}/{row['of']}: rotation "
                              f"{row['rotation_deg']:5.1f} deg, slip {row['slip_mm']:5.1f} mm, "
                              f"joint lag {row['max_joint_lag_deg']:.2f} deg", flush=True)
                    raise RuntimeError("Object lost during carry")
            if name == "settle":
                settled = object_centre().copy()
                report["settled_centre_world_m"] = settled.tolist()
            if name == "retreat":
                after = object_centre()
                movement = float(np.linalg.norm(after-settled))
                report["retreat_object_motion_mm"] = movement*1000
                report["release_verified"] = bool(movement < 0.002
                    and abs(after[2]-before[2]) < 0.0015)
                if not report["release_verified"]:
                    print("[demo] Release check failed; recording final pose.", flush=True)
        centre = object_centre()
        xy_error = float(np.linalg.norm(centre[:2]-pick["place_xy"]))
        z_error = float(abs(centre[2]-before[2]))
        report.update(final_object_centre_world_m=centre.tolist(),
                      placement_xy_error_mm=xy_error*1000,
                      placement_z_error_mm=z_error*1000)
        position, quat = cube.get_world_pose()
        w,x,y,z = quat
        axis = np.array([2*(x*z+y*w), 2*(y*z-x*w), 1-2*(x*x+y*y)])
        tilt = float(np.degrees(np.arccos(np.clip(axis[2], -1, 1))))
        orientation_ok = lying_ok(pick["category"], tilt)
        report["tilt_deg"] = tilt
        if pick["category"] == "cube":
            yaw = float(np.degrees(np.arctan2(2*(x*y+z*w), 1-2*(y*y+z*z))))
            report["yaw_error_deg_mod90"] = abs((yaw+45)%90-45)
            orientation_ok = orientation_ok and report["yaw_error_deg_mod90"] < 8
        report["placement_verified"] = bool(report["pickup_verified"]
            and report["carry_verified"] and report["release_verified"]
            and xy_error < C.CUBE_STACK_SITE_TOLERANCE and z_error < 0.0015
            and orientation_ok)
        report["complete"] = report["placement_verified"]
        if not report["complete"]:
            report["error"] = (
                f"Verification failed: release={report['release_verified']}, "
                f"XY error={xy_error*1000:.3f} mm, Z error={z_error*1000:.3f} mm, "
                f"orientation_ok={orientation_ok}")
        print(f"[demo] {'PASS' if report['complete'] else 'FAIL'}: "
              f"pickup={report['pickup_verified']}, placement={report['placement_verified']}", flush=True)
    except Exception as exc:
        report["error"] = str(exc)
        traceback.print_exc()
    finally:
        output.write_text(json.dumps(report, indent=2)+"\n")
        print(f"[sort] Object report: {output}", flush=True)
    return report


def lying_ok(category, tilt):
    """Object axis tilt from vertical (deg) is plausible for a settled object.
    Vials lie flat; screws lie on the head rim, axis ~6 deg off horizontal."""
    if category == 'vial': return abs(tilt-90) < 5
    if category == 'screw': return abs(tilt-90) < 12
    return tilt < 5


def object_box(pick, placed=False):
    """Conservative object envelopes for offline clearance checking."""
    kind = pick['category']
    xy = pick['place_xy'] if placed else pick['xy']
    if kind == 'cube':
        yaw = 0. if placed else np.deg2rad(pick.get('scan_yaw_deg', C.SORT_CUBE_YAWS_DEG[pick['index']]))
        size = [pick.get('size') or C.CUBE_EDGE]*3
    elif kind == 'screw':
        yaw = 0. if placed else np.deg2rad(pick.get('scan_yaw_deg', C.SORT_SCREWS[pick['index']][2]))
        size = [C.SCREW_LENGTH, C.SCREW_HEAD_D, C.SCREW_HEAD_D]
    elif kind == 'vial':
        yaw = 0. if placed else np.deg2rad(pick.get('scan_yaw_deg', C.SORT_VIALS[pick['index']][2]))
        size = [C.VIAL_LENGTH, 2*C.VIAL_RADIUS, 2*C.VIAL_RADIUS]
    else:
        yaw = 0.
        size = [2*C.CAP_RADIUS, 2*C.CAP_RADIUS, C.CAP_HEIGHT]
    c,s = np.cos(yaw),np.sin(yaw)
    rot = np.array([[c,-s,0],[s,c,0],[0,0,1.]])
    return np.array([*xy, C.SURFACE_DZ+size[2]/2]),rot,np.array(size)/2


def boxes_overlap(a,b,margin=0.002):
    ca,ra,ha=a; cb,rb,hb=b
    delta=cb-ca
    if np.any(np.abs(delta) > np.abs(ra)@ha + np.abs(rb)@hb + margin): return False
    axes=list(ra.T)+list(rb.T)+[np.cross(u,v) for u in ra.T for v in rb.T]
    for axis in axes:
        length=np.linalg.norm(axis)
        if length<1e-8: continue
        axis=axis/length
        reach=np.abs(ra.T@axis)@ha+np.abs(rb.T@axis)@hb+margin
        if abs(delta@axis)>reach: return False
    return True


def finger_boxes(q,jaw):
    rotation,flange=fk.fk(q)
    root=np.asarray(C.TOOL_AXIS)*(C.TOOL_LENGTH-C.FINGER_LENGTH)
    for sign in (-1,1):
        # Conservative full-length rectangular primary rail (includes lead-in).
        primary=('Primary',(C.FINGER_LENGTH,C.FINGER_WIDTH-C.PAD_FACE,C.FINGER_THICK),
                 (C.FINGER_LENGTH/2,sign*(C.FINGER_WIDTH+C.PAD_FACE)/2,0),True)
        for name,size,centre,_ in [primary]+D.extra_boxes(sign):
            local=root+np.asarray(centre)+np.asarray(C.FINGER_SPREAD_AXIS)*sign*jaw
            yield name,(flange+rotation@local,rotation,np.asarray(size)/2)


def validate_clearance(plans):
    """Finger/object envelope screen, not a whole-arm collision planner."""
    picks=[plan[2] for plan in plans]
    for i,pick in enumerate(picks):
        a=object_box(pick,True)
        center,rot,half=a
        extent=np.abs(rot)@half
        if not (C.TABLE_X[0] < center[0]-extent[0] and center[0]+extent[0] < C.TABLE_X[1]
                and C.TABLE_Y[0] < center[1]-extent[1] and center[1]+extent[1] < C.TABLE_Y[1]):
            raise RuntimeError('Destination extends beyond work surface')
        for other in picks[i+1:]:
            if boxes_overlap(a,object_box(other,True),.010):
                raise RuntimeError('Destination envelopes overlap')
        obstacles=[(other,object_box(other,j<i)) for j,other in enumerate(picks) if j!=i]
        close_q=next(steps[-1][0] for phase,steps in plans[i][1] if phase=='close')
        grip_rot,grip_pos=fk.fk(close_q)
        obj_pos,obj_rot,obj_half=object_box(pick)
        relative_pos=grip_rot.T@(obj_pos-grip_pos)
        relative_rot=grip_rot.T@obj_rot
        for phase,steps in plans[i][1]:
            for q,jaw,_ in steps:
                if phase in ('lift','carry','lower'):
                    rot,pos=fk.fk(q)
                    carried=(pos+rot@relative_pos,rot@relative_rot,obj_half)
                    for other,obstacle in obstacles:
                        if boxes_overlap(carried,obstacle):
                            raise RuntimeError(f"Carried object clearance rejected {pick['prim']} near {other['prim']}")
                for name,box in finger_boxes(q,jaw):
                    for other,obstacle in obstacles:
                        if boxes_overlap(box,obstacle):
                            raise RuntimeError(f"Clearance rejected {pick['category']} {pick['index']} {phase}: "
                                f"{name} near {other['category']} {other['index']}")
    print('[sort] Destination, finger and carried-object envelope checks passed (2 mm margin).',flush=True)


def add_slot_markers(stage):
    """Flat visual outlines only; no added collision geometry or raised floors."""
    from pxr import UsdGeom,Gf
    for group,slots in SLOTS.items():
        for i,(x,y) in enumerate(slots):
            hx,hy=(.078,.017) if group=='vial' else ((.023,.023) if group=='cap' else (.015,.015))
            curve=UsdGeom.BasisCurves.Define(stage,f'/World/SortTargets/{group}/Slot{i}')
            z=C.TABLE_TOP_Z+C.SURFACE_DZ+.00015
            points=[(x-hx,y-hy,z),(x+hx,y-hy,z),(x+hx,y+hy,z),(x-hx,y+hy,z),(x-hx,y-hy,z)]
            curve.CreateTypeAttr('linear')
            curve.CreateCurveVertexCountsAttr([5])
            curve.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
            curve.CreateWidthsAttr([.001])
            curve.CreateDisplayColorAttr([Gf.Vec3f(*COLORS[group])])


def pose_metrics(handle,pick,reference_z):
    position,quat=handle.get_world_pose()
    centre=D.centre(position,quat,pick['category'],pick.get('size'))
    w,x,y,z=quat
    tilt=float(np.degrees(np.arccos(np.clip(1-2*(x*x+y*y),-1,1))))
    xy=float(np.linalg.norm(centre[:2]-pick['place_xy']))
    dz=float(abs(centre[2]-reference_z))
    yaw=float(np.degrees(np.arctan2(2*(x*y+z*w),1-2*(y*y+z*z))))
    yaw_error=abs((yaw+45)%90-45)
    orient=lying_ok(pick['category'],tilt)
    if pick['category']=='cube': orient=orient and yaw_error<8
    return dict(centre_world_m=centre.tolist(),xy_error_mm=xy*1000,z_error_mm=dz*1000,
                tilt_deg=tilt,yaw_error_deg_mod90=yaw_error,
                verified=bool(xy<C.CUBE_STACK_SITE_TOLERANCE and dz<.0015 and orient))
