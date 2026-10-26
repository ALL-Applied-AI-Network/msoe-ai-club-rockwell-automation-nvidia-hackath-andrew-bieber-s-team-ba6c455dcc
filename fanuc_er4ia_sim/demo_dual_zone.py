"""Known-position diagnostic for the dual-zone prototype; NOT vision picking.
Run FINGER_SET=dual <Isaac Python> demo_dual_zone.py --object cap --index 0.
Each invocation builds a fresh scene. --check-only validates numerical IK only.
Measured object poses evaluate success and never change commanded targets.
"""
import argparse
import json
import time
import traceback
from datetime import datetime
from pathlib import Path

import numpy as np
import scene_config as C
import fk
import ik
import dual_zone as D

# Motion phases adapted from starter sort_scene.py; kept independent of its
# perception/report imports so this diagnostic needs no OpenCV or Cosmos.
def ease(t):
    return t*t*(3-2*t)


def turn(a, b, t):
    a, b = np.asarray(a), np.asarray(b)
    angle = np.arctan2(a[0]*b[1]-a[1]*b[0], np.dot(a[:2], b[:2]))*t
    c, s = np.cos(angle), np.sin(angle)
    return np.array([a[0]*c-a[1]*s, a[0]*s+a[1]*c, 0.0])


def segments(pick, home):
    p = np.array([*pick["xy"], pick["pick_z"]])
    d = np.array([*pick["place_xy"], pick["place_z"]])
    up, above = p+[0, 0, 0.13], d+[0, 0, 0.13]
    spread, dest = pick["spread_dir"], pick["place_spread_dir"]
    opened, shut = C.JAW_OPEN, pick["jaw_close"]
    return [
        ("approach", home, up, 1.2, opened, opened, spread),
        ("descend", up, p, .8, opened, opened, spread),
        ("close", p, p, .6, opened, shut, spread),
        ("dwell", p, p, .5, shut, shut, spread),
        ("lift", p, up, .8, shut, shut, spread),
        ("carry", up, above, 1.4, shut, shut, dest),
        ("lower", above, d, .8, shut, shut, dest),
        ("release", d, d, .6, shut, opened, dest),
        ("settle", d, d, .6, opened, opened, dest),
        ("retreat", d, above, .8, opened, opened, dest),
        ("home", above, home, 1.1, opened, opened, None),
    ]


def trajectory(category="cube", index=0):
    ik.set_tool_axis(C.TOOL_AXIS)
    ik.set_spread_axis(C.FINGER_SPREAD_AXIS)
    tool = D.tool_offset(category)
    limits = np.deg2rad([C.JOINT_LIMITS_DEG[f"J{i}"] for i in range(1, 7)])
    home = np.array([0.30, 0.0, 0.24])
    down = np.array([0.0, 0.0, -1.0])
    q = np.deg2rad([C.HOME_POSE_DEG[f"J{i}"] for i in range(1, 7)])
    q, ok = ik.solve(q, home, down, limits, tool_offset=tool)
    if not ok:
        raise RuntimeError("Home IK failed")
    initial = q.copy()
    pick = D.pick_spec(category, index)
    # Choose a symmetric spread direction nearest the home orientation.
    if np.dot(fk.fk(q)[0] @ np.asarray(C.FINGER_SPREAD_AXIS), pick["spread_dir"]) < 0:
        pick["spread_dir"] = -pick["spread_dir"]
    if np.dot(pick["spread_dir"], pick["place_spread_dir"]) < 0:
        pick["place_spread_dir"] = -pick["place_spread_dir"]
    phases = []
    worst = 0.0
    for name, p0, p1, duration, j0, j1, direction in segments(pick, home):
        start = None
        if direction is not None and name in ("approach", "carry"):
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--start-delay", type=float, default=10)
    parser.add_argument("--object", choices=("cube", "vial", "cap"), default="cap")
    parser.add_argument("--index", type=int, default=0)
    parser.add_argument("--exit-on-finish", action="store_true")
    args = parser.parse_args()
    if args.index < 0:
        parser.error("--index must be nonnegative")
    initial, phases, pick, worst = trajectory(args.object, args.index)
    print(f"[demo] IK CHECK OK: {sum(len(s) for _, s in phases)} targets; "
          f"maximum numerical tip error {worst*1000:.3f} mm", flush=True)
    print("[demo] KNOWN-POSITION diagnostic, not camera-based picking.", flush=True)
    if args.check_only:
        return
    base = Path(__file__).resolve().parent
    output = base / "runs" / ("dual_" + args.object + "_" + str(args.index) + "_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    output.mkdir(parents=True)
    report = dict(test="known_position_dual_zone", object=args.object, index=args.index, perception_used=False,
                  release_verified=False, carry_verified=False,
                  pickup_verified=False, placement_verified=False,
                  complete=False, error=None, max_numerical_ik_error_mm=worst*1000)
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": args.headless, "limit_cpu_threads": 2,
                         "renderer": "RaytracedLighting", "width": 1280, "height": 720})
    world = None
    try:
        import omni.usd
        from pxr import UsdPhysics
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
        from isaacsim.core.utils.types import ArticulationAction
        import cell
        if not np.isclose(cell.tip_offset(), C.TOOL_LENGTH):
            raise RuntimeError("Gripper TCP differs from the numerical plan")
        world = World(stage_units_in_meters=1.0, physics_dt=1/60, rendering_dt=1/30)
        stage = omni.usd.get_context().get_stage()
        cell.build(stage)
        if not args.headless:
            # Isaac 6.0 core.utils.viewports API (deprecated but available).
            # https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/deprecated/isaacsim.core.utils/docs/index.html
            from isaacsim.core.utils.viewports import set_camera_view
            set_camera_view(eye=np.array([1.05, 0.80, 1.65]),
                            target=np.array([0.25, 0.04, 1.04]))
        bodies = [p for p in stage.Traverse()
                  if str(p.GetPath()).startswith(("/World/SortCube", "/World/SortCap", "/World/SortVial"))
                  and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        for prim in bodies:
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True)
        cell.enable_ccd(world)
        world.reset()
        arm = SingleArticulation(prim_path=C.ROBOT_PRIM, name="demo_er4ia")
        arm.initialize()
        names = list(arm.dof_names)
        arm_idx = [names.index(f"J{i}") for i in range(1, 7)]
        joints = {name: i for i, name in enumerate(names)}
        cube = SingleXFormPrim(pick["prim"])
        tool = D.tool_offset(args.object)

        def measured():
            values = arm.get_joint_positions()
            if values is None:
                raise RuntimeError("Articulation physics view unavailable; do not press Stop during the test")
            return values.copy()

        def command(q, jaw):
            full = measured()
            full[arm_idx] = q
            for name, value in cell.jaw_targets(jaw).items():
                full[joints[name]] = value
            arm.apply_action(ArticulationAction(joint_positions=full))
            return full

        def tick(q, jaw):
            if not app.is_running() or not world.is_playing():
                raise RuntimeError("Demo interrupted")
            start = time.monotonic()
            command(q, jaw)
            world.step(render=False)
            world.step(render=not args.headless)
            if not args.headless:
                time.sleep(max(0, 1/30 - (time.monotonic()-start)))

        arm.set_joint_positions(command(initial, C.JAW_OPEN))
        for _ in range(60):
            tick(initial, C.JAW_OPEN)
        for prim in bodies:
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(False)
        print(f"[demo] Scene ready. Starting in {max(0,args.start_delay):g} seconds of simulation.", flush=True)
        for _ in range(max(30, round(max(0,args.start_delay)*30))):
            tick(initial, C.JAW_OPEN)
        def object_centre():
            position, quat = cube.get_world_pose()
            return D.centre(position, quat, args.object)

        before = object_centre().copy()
        report["initial_object_centre_world_m"] = before.tolist()
        # Evaluation gate only: never use measured pose to adjust grasp commands.
        if np.linalg.norm(before[:2] - pick["xy"]) > 0.002:
            raise RuntimeError("Object moved away from the configured test position; aborting")
        for name, steps in phases:
            print(f"[demo] {name.upper()}", flush=True)
            for q, jaw, target in steps:
                tick(q, jaw)
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
        orientation_ok = abs(tilt-90) < 5 if args.object == "vial" else tilt < 5
        report["tilt_deg"] = tilt
        if args.object == "cube":
            yaw = float(np.degrees(np.arctan2(2*(x*y+z*w), 1-2*(y*y+z*z))))
            report["yaw_error_deg_mod90"] = abs((yaw+45)%90-45)
            orientation_ok = orientation_ok and report["yaw_error_deg_mod90"] < 8
        report["placement_verified"] = bool(report["pickup_verified"]
            and report["carry_verified"] and report["release_verified"]
            and xy_error < C.CUBE_STACK_SITE_TOLERANCE and z_error < 0.0015
            and orientation_ok)
        report["complete"] = report["placement_verified"]
        print(f"[demo] {'PASS' if report['complete'] else 'FAIL'}: "
              f"pickup={report['pickup_verified']}, placement={report['placement_verified']}", flush=True)
    except Exception as exc:
        report["error"] = str(exc)
        traceback.print_exc()
    finally:
        path = output / "demo_report.json"
        path.write_text(json.dumps(report, indent=2)+"\n")
        print(f"[demo] Report: {path}", flush=True)
        if world is not None:
            world.pause()
        if not args.headless and not args.exit_on_finish and app.is_running():
            print("[demo] Finished; window remains open for inspection. Close it when done.", flush=True)
            while app.is_running():
                app.update()
                time.sleep(0.02)
        app.close()
    if not report["complete"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
