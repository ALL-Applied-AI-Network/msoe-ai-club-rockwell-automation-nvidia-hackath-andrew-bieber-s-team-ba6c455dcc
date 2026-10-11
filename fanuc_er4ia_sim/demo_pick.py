"""One known-position cube pick in simulation; no model or camera inference.

Place beside the supplied scene_config.py, cell.py and sort_scene.py.
Run with Isaac Python. --check-only runs numerical IK checks with host Python.
This diagnostic deliberately uses the configured layout. It is NOT a vision
benchmark and does not change or bypass the starter's perception-plan gates.
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
    opened, shut = C.JAW_OPEN, C.CUBE_JAW_HALF_TARGET
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


def trajectory():
    ik.set_tool_axis(C.TOOL_AXIS)
    ik.set_spread_axis(C.FINGER_SPREAD_AXIS)
    tool = np.asarray(C.TOOL_AXIS) * C.TOOL_LENGTH
    limits = np.deg2rad([C.JOINT_LIMITS_DEG[f"J{i}"] for i in range(1, 7)])
    home = np.array([0.30, 0.0, 0.24])
    down = np.array([0.0, 0.0, -1.0])
    q = np.deg2rad([C.HOME_POSE_DEG[f"J{i}"] for i in range(1, 7)])
    q, ok = ik.solve(q, home, down, limits, tool_offset=tool)
    if not ok:
        raise RuntimeError("Home IK failed")
    initial = q.copy()
    x, y, colour = C.SORT_CUBES[0]
    yaw = np.deg2rad(C.SORT_CUBE_YAWS_DEG[0])
    spread = np.array([-np.sin(yaw), np.cos(yaw), 0.0])
    if np.dot(fk.fk(q)[0] @ np.asarray(C.FINGER_SPREAD_AXIS), spread) < 0:
        spread = -spread
    destination = np.array([0.0, 1.0, 0.0])
    if np.dot(spread, destination) < 0:
        destination = -destination
    pick = dict(category="cube", xy=[x, y], colour=colour,
                pick_z=C.SURFACE_DZ + C.CUBE_TIP_CLEARANCE,
                place_xy=list(C.CUBE_STACK_XY),
                place_z=C.SURFACE_DZ + C.CUBE_TIP_CLEARANCE,
                spread_dir=spread, place_spread_dir=destination)
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
    args = parser.parse_args()
    initial, phases, pick, worst = trajectory()
    print(f"[demo] IK CHECK OK: {sum(len(s) for _, s in phases)} targets; "
          f"maximum numerical tip error {worst*1000:.3f} mm", flush=True)
    print("[demo] KNOWN-POSITION diagnostic, not camera-based picking.", flush=True)
    if args.check_only:
        return
    base = Path(__file__).resolve().parent
    output = base / "runs" / ("demo_pick_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    output.mkdir(parents=True)
    report = dict(test="known_position_single_cube", perception_used=False,
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
        cube = SingleXFormPrim("/World/SortCube0")
        tool = np.asarray(C.TOOL_AXIS) * C.TOOL_LENGTH

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
        before = cube.get_world_pose()[0].copy()
        report["initial_cube_origin_world_m"] = before.tolist()
        # Evaluation gate only: never use measured pose to adjust grasp commands.
        if np.linalg.norm(before[:2] - pick["xy"]) > 0.002:
            raise RuntimeError("Cube moved away from the configured test position; aborting")
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
                height = float(cube.get_world_pose()[0][2]-before[2])
                report["lift_height_mm"] = height*1000
                report["pickup_verified"] = height > 0.025
                print(f"[demo] Cube rose {height*1000:.1f} mm; pickup={report['pickup_verified']}", flush=True)
                if not report["pickup_verified"]:
                    raise RuntimeError("Cube did not lift; stopping before carry")
        position, quat = cube.get_world_pose()
        w, x, y, z = quat
        up = np.array([2*(x*z+y*w), 2*(y*z-x*w), 1-2*(x*x+y*y)])
        centre = position + up*C.CUBE_EDGE/2
        xy_error = float(np.linalg.norm(centre[:2]-pick["place_xy"]))
        z_error = float(abs(centre[2]-(C.TABLE_TOP_Z+C.SURFACE_DZ+C.CUBE_EDGE/2)))
        tilt = float(np.degrees(np.arccos(np.clip(up[2], -1, 1))))
        yaw = float(np.degrees(np.arctan2(2*(x*y+z*w), 1-2*(y*y+z*z))))
        yaw_error = abs((yaw+45)%90-45)
        report.update(final_cube_centre_world_m=centre.tolist(),
                      placement_xy_error_mm=xy_error*1000,
                      placement_z_error_mm=z_error*1000, tilt_deg=tilt,
                      yaw_error_deg_mod90=yaw_error)
        report["placement_verified"] = bool(report["pickup_verified"]
            and xy_error < C.CUBE_STACK_SITE_TOLERANCE and z_error < 0.0015
            and tilt < 5 and yaw_error < 8)
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
        if not args.headless and app.is_running():
            print("[demo] Finished; window remains open for inspection. Close it when done.", flush=True)
            while app.is_running():
                app.update()
                time.sleep(0.02)
        app.close()


if __name__ == "__main__":
    main()
