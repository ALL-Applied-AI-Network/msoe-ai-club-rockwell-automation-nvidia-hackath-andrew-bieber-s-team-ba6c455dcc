#!/usr/bin/env python3
"""Image-guided mixed-object sorting using the uploaded Isaac motion planner.

Run without --execute first. Add six random screws by default, classify SAM
masks by metric shape/pixel colour, then attempt screws/cubes before caps/vials.
Install beside the existing cell.py, fk.py, ik.py and scene_config.py.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import time
import traceback
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

import numpy as np

import fk
import ik
import scene_config as C
import sort_config as S
from sort_vision import SamClient, Attempts, build_candidates, write_overlay, roi_from_pixels
from add_random_screws import add_random_screws


def ease(t: float) -> float:
    return t * t * (3.0 - 2.0 * t)


def turn(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    a, b = np.asarray(a), np.asarray(b)
    angle = np.arctan2(a[0] * b[1] - a[1] * b[0], np.dot(a[:2], b[:2])) * t
    c, s = np.cos(angle), np.sin(angle)
    return np.array([a[0] * c - a[1] * s, a[0] * s + a[1] * c, 0.0])


def parse_csv(value: str, count: int, name: str) -> tuple[float, ...]:
    values = tuple(float(part.strip()) for part in value.split(","))
    if len(values) != count:
        raise argparse.ArgumentTypeError(f"{name} must contain {count} comma-separated numbers")
    return values


def load_homography(path: Path | None) -> np.ndarray | None:
    if path is None:
        return None
    payload = json.loads(path.read_text(encoding="utf-8"))
    raw = payload.get("pixel_to_world_homography", payload)
    matrix = np.asarray(raw, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError("Homography JSON must contain a 3x3 pixel_to_world_homography")
    return matrix


def pixel_to_world(u: float, v: float, homography: np.ndarray) -> np.ndarray:
    projected = homography @ np.array([u, v, 1.0], dtype=np.float64)
    if abs(projected[2]) < 1e-12:
        raise ValueError("Homography projected a point to infinity")
    return projected[:2] / projected[2]


@dataclass(frozen=True)
class PickTarget:
    detection_id: int
    pixel_uv: tuple[float, float]
    world_xy: np.ndarray
    spread_dir: np.ndarray
    area_px: int
    confidence: float


def segments(
    target: PickTarget,
    home: np.ndarray,
    pick_z: float,
    drop_xy: np.ndarray,
    drop_z: float,
    jaw_close: float,
    release_slide: float = 0.0,
) -> list[tuple[str, np.ndarray, np.ndarray, float, float, float, np.ndarray | None]]:
    p = np.array([target.world_xy[0], target.world_xy[1], pick_z])
    d = np.array([drop_xy[0], drop_xy[1], drop_z])
    up, above = p + [0.0, 0.0, 0.13], d + [0.0, 0.0, 0.13]
    destination = np.array([0.0, 1.0, 0.0])
    if np.dot(target.spread_dir, destination) < 0:
        destination = -destination
    opened = C.JAW_OPEN
    phases = [
        ("approach", home, up, 1.2, opened, opened, target.spread_dir),
        ("descend", up, p, 0.8, opened, opened, target.spread_dir),
        ("close", p, p, 0.6, opened, jaw_close, target.spread_dir),
        ("dwell", p, p, 0.5, jaw_close, jaw_close, target.spread_dir),
        ("lift", p, up, 0.8, jaw_close, jaw_close, target.spread_dir),
        ("carry", up, above, 1.4, jaw_close, jaw_close, destination),
        ("lower", above, d, 0.8, jaw_close, jaw_close, destination),
        ("release", d, d, 0.6, jaw_close, opened, destination),
        ("settle", d, d, 0.6, opened, opened, destination),
    ]
    if release_slide > 0.0:
        # Move perpendicular to the open jaws while staying at setdown height.
        # A vertical retreat from inside the cap can re-hook it in the finger
        # scallops; this sideways move clears the cap before lifting.
        escape_direction = np.array([destination[1], -destination[0], 0.0])
        escaped = d + release_slide * escape_direction
        escaped_above = escaped + [0.0, 0.0, 0.13]
        phases.extend(
            [
                ("slide_release", d, escaped, 0.6, opened, opened, destination),
                ("retreat", escaped, escaped_above, 0.8, opened, opened, destination),
                ("home", escaped_above, home, 1.1, opened, opened, None),
            ]
        )
    else:
        phases.extend(
            [
                ("retreat", d, above, 0.8, opened, opened, destination),
                ("home", above, home, 1.1, opened, opened, None),
            ]
        )
    return phases


def plan_trajectory(
    target: PickTarget,
    initial: np.ndarray,
    home: np.ndarray,
    limits: np.ndarray,
    tool: np.ndarray,
    pick_z: float,
    drop_xy: np.ndarray,
    drop_z: float,
    jaw_close: float,
    release_slide: float = 0.0,
) -> tuple[list[tuple[str, list[tuple[np.ndarray, float, np.ndarray]]]], float]:
    down = np.array([0.0, 0.0, -1.0])
    q = initial.copy()
    spread = target.spread_dir.copy()
    if np.dot(fk.fk(q)[0] @ np.asarray(C.FINGER_SPREAD_AXIS), spread) < 0:
        spread = -spread
    target = PickTarget(
        target.detection_id,
        target.pixel_uv,
        target.world_xy,
        spread,
        target.area_px,
        target.confidence,
    )
    phases = []
    worst = 0.0
    for name, p0, p1, duration, j0, j1, direction in segments(
        target, home, pick_z, drop_xy, drop_z, jaw_close, release_slide
    ):
        start = None
        if direction is not None and name in ("approach", "carry"):
            start = fk.fk(q)[0] @ np.asarray(C.FINGER_SPREAD_AXIS)
            start[2] = 0.0
            start /= np.linalg.norm(start)
        steps = []
        count = round(duration * 2 * 30)
        for index in range(count):
            t = ease((index + 1) / count)
            position = p0 + (p1 - p0) * t
            desired = turn(start, direction, t) if start is not None else direction
            q, ok = ik.solve(q, position, down, limits, tool_offset=tool, spread_dir=desired)
            if not ok:
                q, ok = ik.solve(
                    q,
                    position,
                    down,
                    limits,
                    tool_offset=tool,
                    spread_dir=desired,
                    iters=500,
                    damping=0.02,
                )
            rotation, actual = fk.tool_pose(q, tool)
            error = float(np.linalg.norm(actual - position))
            worst = max(worst, error)
            aligned = desired is None or abs(
                np.dot(rotation @ np.asarray(C.FINGER_SPREAD_AXIS), desired)
            ) > 0.999
            approach_ok = np.dot(rotation @ np.asarray(C.TOOL_AXIS), down) > 0.999
            if not ok or error > 0.001 or not aligned or not approach_ok:
                raise RuntimeError(
                    f"IK rejected {name} frame {index}: {error * 1000:.3f} mm"
                )
            steps.append((q.copy(), float(j0 + (j1 - j0) * t), position.copy()))
        phases.append((name, steps))
    return phases, worst


def parse_args() -> argparse.Namespace:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="Enable robot motion.")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--start-delay", type=float, default=10.0)
    parser.add_argument("--max-picks", type=int, default=10)
    parser.add_argument("--homography", type=Path)
    parser.add_argument("--sam-python", type=Path, default=Path.home() / "venvs/sam2-check/bin/python")
    parser.add_argument("--sam-worker", type=Path, default=base / "sort_vision.py")
    parser.add_argument("--model-id", default="facebook/sam2.1-hiera-small")
    parser.add_argument("--points-per-side", type=int, default=64)
    parser.add_argument("--points-per-batch", type=int, default=32)
    parser.add_argument("--roi", help="Camera ROI as x1,y1,x2,y2 pixels; default covers the pickup workspace.")
    parser.add_argument("--min-area-px", type=int, default=20)
    parser.add_argument("--max-area-fraction", type=float, default=0.08)
    parser.add_argument("--max-objects", type=int, default=30)
    parser.add_argument("--border-margin-px", type=int, default=3)
    parser.add_argument("--resolution", default="1920,1080", help="width,height")
    parser.add_argument("--camera-eye", default="0.37,0.0,2.20", help="world x,y,z")
    parser.add_argument("--camera-target", default="0.37,0.0,1.00", help="world x,y,z")
    parser.add_argument("--camera-focal-length", type=float, default=24.0)
    parser.add_argument(
        "--workspace",
        default=",".join(map(str, S.WORKSPACE)),
        help="reachable xmin,xmax,ymin,ymax in world meters",
    )
    parser.add_argument(
        "--pick-z",
        type=float,
        default=float(C.SURFACE_DZ + C.CUBE_TIP_CLEARANCE),
        help="tool-tip Z for first cube test",
    )
    parser.add_argument("--jaw-close", type=float, default=float(C.CUBE_JAW_HALF_TARGET))
    parser.add_argument(
        "--cap-pick-height",
        type=float,
        default=0.005,
        help="cap fingertip height above the work surface in meters (default: 0.005)",
    )
    parser.add_argument("--object-match-radius", type=float, default=0.020)
    parser.add_argument(
        "--target-body",
        help="diagnostic-only exact simulated body path, e.g. /World/SortCap0",
    )
    parser.add_argument(
        "--target-type",
        choices=("screw", "cube", "cap", "vial", "any"),
        default="any",
        help="Image-class filter; default attempts every supported type.",
    )
    parser.add_argument("--screws", type=int, default=6)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    args.resolution = tuple(int(v) for v in parse_csv(args.resolution, 2, "--resolution"))
    args.camera_eye = parse_csv(args.camera_eye, 3, "--camera-eye")
    args.camera_target = parse_csv(args.camera_target, 3, "--camera-target")
    args.workspace = parse_csv(args.workspace, 4, "--workspace")
    return args


def main() -> None:
    args = parse_args()
    print("[vision] mixed-object sorting; image labels, smallest categories first", flush=True)
    homography = load_homography(args.homography)
    if not args.execute:
        print("[vision] DRY RUN: world targets will be printed, but the robot will not move.", flush=True)

    ik.set_tool_axis(C.TOOL_AXIS)
    ik.set_spread_axis(C.FINGER_SPREAD_AXIS)
    tool = np.asarray(C.TOOL_AXIS) * C.TOOL_LENGTH
    limits = np.deg2rad([C.JOINT_LIMITS_DEG[f"J{i}"] for i in range(1, 7)])
    # Camera-clear observation/return pose, numerically checked against this arm.
    home = np.array([0.12, -0.32, 0.38])
    down = np.array([0.0, 0.0, -1.0])
    initial = np.deg2rad([C.HOME_POSE_DEG[f"J{i}"] for i in range(1, 7)])
    initial, ok = ik.solve(initial, home, down, limits, tool_offset=tool)
    if not ok:
        raise RuntimeError("Home IK failed")

    base = Path(__file__).resolve().parent
    output = base / "runs" / ("vision_pick_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    output.mkdir(parents=True)
    report: dict[str, Any] = {
        "test": "sam2_closed_loop_pick_and_dump",
        "execute": bool(args.execute),
        "complete": False,
        "picks": [],
        "error": None,
    }

    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": args.headless,
            "limit_cpu_threads": 2,
            "renderer": "RaytracedLighting",
            "width": args.resolution[0],
            "height": args.resolution[1],
        }
    )
    world = None
    sam = None
    rgb_annotator = None
    depth_annotator = None
    render_product = None
    try:
        import omni.replicator.core as rep
        import omni.usd
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
        from isaacsim.core.utils.types import ArticulationAction
        import cell

        if not np.isclose(cell.tip_offset(), C.TOOL_LENGTH):
            raise RuntimeError("Gripper TCP differs from the numerical plan")
        world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 30)
        stage = omni.usd.get_context().get_stage()
        cell.build(stage)
        if args.screws:
            add_random_screws(stage, C, args.screws, args.seed)

        bodies = [
            prim
            for prim in stage.Traverse()
            if str(prim.GetPath()).startswith(
                ("/World/SortCube", "/World/SortCap", "/World/SortVial", "/World/SortScrew")
            )
            and prim.HasAPI(UsdPhysics.RigidBodyAPI)
        ]
        for prim in bodies:
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True)
        cell.enable_ccd(world)
        world.reset()

        arm = SingleArticulation(prim_path=C.ROBOT_PRIM, name="vision_er4ia")
        arm.initialize()
        names = list(arm.dof_names)
        arm_idx = [names.index(f"J{i}") for i in range(1, 7)]
        joints = {name: index for index, name in enumerate(names)}
        tracked_bodies = [(str(prim.GetPath()), SingleXFormPrim(str(prim.GetPath()))) for prim in bodies]

        camera = rep.functional.create.camera(
            position=args.camera_eye,
            look_at=args.camera_target,
            focal_length=args.camera_focal_length,
            clipping_range=(0.01, 100.0),
            parent="/World",
            name="VisionTopCamera",
        )
        render_product = rep.create.render_product(
            camera.GetPath(), args.resolution, name="VisionTopRenderProduct"
        )
        rgb_annotator = rep.annotators.get("rgb")
        rgb_annotator.attach(render_product)
        depth_annotator = rep.annotators.get("distance_to_image_plane")
        depth_annotator.attach(render_product)

        camera_prim = UsdGeom.Camera(stage.GetPrimAtPath(str(camera.GetPath())))
        if not camera_prim:
            raise RuntimeError("Could not wrap the vision camera USD prim")
        focal_length = float(camera_prim.GetFocalLengthAttr().Get())
        horizontal_aperture = float(camera_prim.GetHorizontalApertureAttr().Get())
        camera_to_world = UsdGeom.Xformable(camera_prim.GetPrim()).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        image_width, image_height = args.resolution
        fx = focal_length * image_width / horizontal_aperture
        fy = fx
        cx = image_width / 2.0
        cy = image_height / 2.0
        if args.roi is None:
            world_to_camera = camera_to_world.GetInverse()
            xmin,xmax,ymin,ymax = args.workspace
            pixels = []
            for x in (xmin,xmax):
                for y in (ymin,ymax):
                    p = world_to_camera.Transform(Gf.Vec3d(x,y,C.TABLE_TOP_Z+C.SURFACE_DZ))
                    pixels.append([cx + fx*p[0]/(-p[2]), cy - fy*p[1]/(-p[2])])
            args.roi = roi_from_pixels(pixels,image_width,image_height)

        def measured() -> np.ndarray:
            values = arm.get_joint_positions()
            if values is None:
                raise RuntimeError("Articulation unavailable; do not press Stop during the test")
            return values.copy()

        def command(q: np.ndarray, jaw: float) -> np.ndarray:
            full = measured()
            full[arm_idx] = q
            for name, value in cell.jaw_targets(jaw).items():
                full[joints[name]] = value
            arm.apply_action(ArticulationAction(joint_positions=full))
            return full

        def tick(q: np.ndarray, jaw: float, render: bool = False) -> None:
            if not app.is_running() or not world.is_playing():
                raise RuntimeError("Demo interrupted")
            started = time.monotonic()
            command(q, jaw)
            world.step(render=False)
            # Camera render products still need rendered frames in headless mode.
            world.step(render=render)
            if render and not args.headless:
                time.sleep(max(0.0, 1 / 30 - (time.monotonic() - started)))

        def capture(frame_dir: Path) -> tuple[Path, np.ndarray]:
            frame_dir.mkdir(parents=True, exist_ok=True)
            # Render a few settled frames. Avoid rep.orchestrator.step() here because
            # its default timeline behavior can pause a running physics simulation.
            for _ in range(4):
                tick(initial, C.JAW_OPEN, render=True)
            data = rgb_annotator.get_data()
            if data is None or np.asarray(data).size == 0:
                raise RuntimeError("RGB camera returned no data")
            depth = depth_annotator.get_data()
            depth = np.asarray(depth, dtype=np.float64)
            if depth.size == 0 or depth.shape[:2] != (image_height, image_width):
                raise RuntimeError(f"Depth camera returned invalid shape {depth.shape}")
            image_path = frame_dir / "camera_rgb.png"
            rep.functional.write_image(path=str(image_path), data=data)
            return image_path, depth.copy()

        def plane_projector(u, v, height=0.0):
            if homography is not None:
                return pixel_to_world(u, v, homography)
            origin = camera_to_world.Transform(Gf.Vec3d(0, 0, 0))
            endpoint = camera_to_world.Transform(Gf.Vec3d((u-cx)/fx, -(v-cy)/fy, -1.0))
            ray = np.array(endpoint) - np.array(origin)
            z = C.TABLE_TOP_Z + C.SURFACE_DZ + height
            if abs(ray[2]) < 1e-9:
                raise ValueError("Camera ray is parallel to the work surface")
            t = (z-origin[2])/ray[2]
            if t <= 0:
                raise ValueError("Camera ray points away from the work surface")
            return (np.array(origin) + ray*t)[:2]

        def recover_home(q, jaw):
            start = fk.tool_pose(q, tool)[1]
            for step in range(120):
                p = start + (home-start)*ease((step+1)/120)
                q, ok = ik.solve(q, p, down, limits, tool_offset=tool)
                if not ok:
                    raise RuntimeError("Failed-pick retreat IK failed")
                tick(q, jaw, render=True)
            for _ in range(15):
                tick(initial, C.JAW_OPEN, render=True)

        def nearest_body(xy: np.ndarray) -> tuple[str, Any, np.ndarray, float] | None:
            choices = []
            for path, body in tracked_bodies:
                position = body.get_world_pose()[0].copy()
                choices.append((float(np.linalg.norm(position[:2] - xy)), path, body, position))
            if not choices:
                return None
            distance, path, body, position = min(choices, key=lambda item: item[0])
            if distance > args.object_match_radius:
                return None
            return path, body, position, distance

        arm.set_joint_positions(command(initial, C.JAW_OPEN))
        for _ in range(60):
            tick(initial, C.JAW_OPEN, render=True)
        for prim in bodies:
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(False)
        print(
            f"[vision] Scene ready. Starting in {max(0.0, args.start_delay):g} seconds of simulation.",
            flush=True,
        )
        for _ in range(max(30, round(max(0.0, args.start_delay) * 30))):
            tick(initial, C.JAW_OPEN, render=True)

        sam = SamClient(
            args.sam_python,
            args.sam_worker.resolve(),
            args.model_id,
            args.points_per_side,
            args.points_per_batch,
        )

        attempts = Attempts()
        for cycle in range(args.max_picks):
            cycle_dir = output / f"cycle_{cycle:02d}"
            image_path, depth = capture(cycle_dir)
            response = sam.detect(
                {
                    "image": str(image_path),
                    "output_dir": str(cycle_dir),
                    "roi": args.roi,
                    "min_area_px": args.min_area_px,
                    "max_area_fraction": args.max_area_fraction,
                    "max_objects": args.max_objects,
                    "border_margin_px": args.border_margin_px,
                }
            )
            detections = response["detections"]
            print(
                f"[vision] cycle={cycle} masks={response['raw_mask_count']} "
                f"objects={len(detections)} overlay={response['overlay']}",
                flush=True,
            )
            cycle_report: dict[str, Any] = {
                "cycle": cycle,
                "image": str(image_path),
                "overlay": response["overlay"],
                "detections": detections,
            }
            report["picks"].append(cycle_report)

            candidates, rejected = build_candidates(detections, plane_projector, args.workspace)
            candidates = [c for c in candidates if args.target_type in ("any", c["kind"])]
            candidates = candidates[:args.max_objects]
            if args.target_body is not None:
                # Explicit diagnostic override only; normal selection uses image labels.
                candidates = [c for c in candidates if
                              (nearest_body(np.array(c["world_xy"])) or [None])[0] == args.target_body]
            cycle_report["rejected"] = rejected
            cycle_report["world_targets"] = candidates
            write_overlay(image_path, candidates, cycle_dir / "targets.png")
            print("[vision] order: " + ", ".join(
                f"{c['label']} ({c['world_xy'][0]:.3f},{c['world_xy'][1]:.3f}) -> {c['drop']}"
                for c in candidates), flush=True)
            candidates = attempts.available(candidates)
            if not args.execute:
                print("[vision] Dry-run target conversion complete; robot motion disabled.", flush=True)
                report["complete"] = True
                report["stop_reason"] = "world_coordinate_dry_run"
                break
            if not candidates:
                print("[vision] No eligible detections remain (or attempt limits reached).", flush=True)
                report["complete"] = True
                report["stop_reason"] = "no_eligible_detections"
                break
            chosen = candidates[0]
            attempts.add(chosen)
            target = PickTarget(chosen["id"], tuple(chosen["center_full_px"]),
                np.array(chosen["world_xy"]), np.array(chosen["spread_dir"]),
                chosen["area_px"], chosen["predicted_iou"]*chosen["stability_score"])
            # Ground truth is used for validation ONLY, not to label or select objects.
            matched = nearest_body(target.world_xy)
            body_path, body, before, match_distance = matched if matched is not None else (None,None,None,None)
            object_type = chosen["kind"]
            pick_z = float(C.SURFACE_DZ + S.PICK_HEIGHT[object_type])
            if object_type == "cube":
                pick_z = args.pick_z
            elif object_type == "cap":
                pick_z = C.SURFACE_DZ + args.cap_pick_height
            drop_z = C.SURFACE_DZ + S.DROP_HEIGHT
            drop_xy = np.array(S.DROP_XY[chosen["drop"]])
            jaw_close = args.jaw_close if object_type == "cube" else C.JAW_CLOSED
            release_slide = 0.0  # release ABOVE the cup; no lateral slide through its rim
            cycle_report.update(selected=chosen, matched_body=body_path,
                match_error_mm=None if matched is None else match_distance*1000,
                planned_pick_z=pick_z, planned_drop_z=drop_z,
                pickup_verified=None, released=None)
            phases, worst = plan_trajectory(
                target,
                initial,
                home,
                limits,
                tool,
                pick_z,
                drop_xy,
                drop_z,
                jaw_close,
                release_slide,
            )
            cycle_report["max_numerical_ik_error_mm"] = worst * 1000
            print(
                f"[vision] PICK #{target.detection_id} world=({target.world_xy[0]:.3f},"
                f"{target.world_xy[1]:.3f}) body={body_path}",
                flush=True,
            )
            print(
                f"[vision] type={object_type} fingertip_height_mm="
                f"{(pick_z - C.SURFACE_DZ) * 1000:.1f} "
                f"release_slide_mm={release_slide * 1000:.1f}",
                flush=True,
            )
            release_reference = None
            failed_pick = False
            for name, steps in phases:
                print(f"[vision] {name.upper()}", flush=True)
                for q, jaw, position in steps:
                    tick(q, jaw, render=True)
                actual_tip = fk.tool_pose(measured()[arm_idx], tool)[1]
                error = float(np.linalg.norm(actual_tip - position))
                cycle_report.setdefault("tip_errors_mm", {})[name] = error * 1000
                if name in ("descend", "lower") and error > 0.003:
                    raise RuntimeError(
                        f"Arm did not reach {name} target: {error * 1000:.2f} mm error"
                    )
                if name == "lift" and body is not None:
                    lift = float(body.get_world_pose()[0][2] - before[2])
                    cycle_report["lift_height_mm"] = lift * 1000
                    cycle_report["pickup_verified"] = lift > 0.025
                    print(
                        f"[vision] {body_path} rose {lift * 1000:.1f} mm; "
                        f"pickup={cycle_report['pickup_verified']}",
                        flush=True,
                    )
                    if not cycle_report["pickup_verified"]:
                        failed_pick = True
                        recover_home(q, jaw)
                        break
                if body is not None and name == "settle":
                    release_reference = np.asarray(body.get_world_pose()[0], dtype=float)
                if body is not None and name == "retreat":
                    released_pose = np.asarray(body.get_world_pose()[0], dtype=float)
                    if release_reference is None:
                        raise RuntimeError("Release reference was not recorded")
                    followed_xy = float(
                        np.linalg.norm(released_pose[:2] - release_reference[:2])
                    )
                    followed_z = float(released_pose[2] - release_reference[2])
                    cycle_report["release_follow_xy_mm"] = followed_xy * 1000
                    cycle_report["release_follow_z_mm"] = followed_z * 1000
                    if followed_z > 0.015:
                        raise RuntimeError(
                            "Object followed the retreat; it may still be caught in the gripper"
                        )
            if failed_pick:
                cycle_report["released"] = False
                continue
            if body is not None:
                final_position = np.array(body.get_world_pose()[0])
                cycle_report["released"] = bool(
                    np.linalg.norm(final_position[:2]-drop_xy) < S.DROP_RADIUS and
                    final_position[2] < C.TABLE_TOP_Z + C.SURFACE_DZ + 0.050)
            # An unmatched target is unverified, never reported as a successful pickup.
            # Let the dumped object settle before the next clean observation.
            for _ in range(30):
                tick(initial, C.JAW_OPEN, render=True)
        else:
            report["complete"] = True
            report["stop_reason"] = "max_picks"

    except Exception as exc:
        report["error"] = str(exc)
        traceback.print_exc()
    finally:
        report_path = output / "vision_pick_report.json"
        report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"[vision] Report: {report_path}", flush=True)
        if sam is not None:
            sam.close()
        if rgb_annotator is not None and render_product is not None:
            try:
                rgb_annotator.detach(render_product)
            except Exception:
                pass
        if depth_annotator is not None and render_product is not None:
            try:
                depth_annotator.detach(render_product)
            except Exception:
                pass
        if world is not None:
            world.pause()
        if not args.headless and app.is_running():
            print("[vision] Finished; window remains open. Close it when done.", flush=True)
            while app.is_running():
                app.update()
                time.sleep(0.02)
        app.close()
    if report["error"] is not None:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
