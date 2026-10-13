#!/usr/bin/env python3
"""Vision-guided top-down pick-and-dump loop for the FANUC Isaac Sim cell.

The script creates a clean fixed RGB camera, sends frames to a persistent SAM 2
worker, filters detections into reachable world XY targets, picks one target,
dumps it at a fixed location, returns home, and re-detects before the next pick.

Default mode is perception-only. Pixel targets are projected with the depth
camera (or an optional --homography), and checked against simulation bodies.
Robot motion requires --execute; validate the dry-run coordinates first.
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


def image_axis_to_world_spread(
    u: float,
    v: float,
    yaw_image_deg: float,
    projector: Callable[[float, float], np.ndarray],
) -> np.ndarray:
    angle = math.radians(yaw_image_deg)
    p0 = projector(u, v)
    p1 = projector(u + 20.0 * math.cos(angle), v + 20.0 * math.sin(angle))
    object_axis = p1 - p0
    norm = np.linalg.norm(object_axis)
    if norm < 1e-9:
        raise ValueError("Could not transform image grasp direction to world coordinates")
    object_axis /= norm
    # Match demo_pick.py: finger-spread direction is perpendicular to object yaw.
    return np.array([-object_axis[1], object_axis[0], 0.0])


@dataclass(frozen=True)
class PickTarget:
    detection_id: int
    pixel_uv: tuple[float, float]
    world_xy: np.ndarray
    spread_dir: np.ndarray
    area_px: int
    confidence: float


def build_targets(
    detections: list[dict[str, Any]],
    projector: Callable[[float, float], np.ndarray],
    workspace: tuple[float, float, float, float],
    drop_xy: np.ndarray,
    drop_exclusion_radius: float,
) -> tuple[list[PickTarget], list[dict[str, Any]]]:
    xmin, xmax, ymin, ymax = workspace
    targets: list[PickTarget] = []
    rejected: list[dict[str, Any]] = []
    for detection in detections:
        u, v = (float(value) for value in detection["center_full_px"])
        xy = projector(u, v)
        reason = None
        if not (xmin <= xy[0] <= xmax and ymin <= xy[1] <= ymax):
            reason = "outside_workspace"
        elif np.linalg.norm(xy - drop_xy) < drop_exclusion_radius:
            reason = "inside_drop_exclusion"
        if reason is not None:
            rejected.append({"id": detection["id"], "world_xy": xy.tolist(), "reason": reason})
            continue
        targets.append(
            PickTarget(
                detection_id=int(detection["id"]),
                pixel_uv=(u, v),
                world_xy=xy,
                spread_dir=image_axis_to_world_spread(
                    u, v, float(detection["grasp_yaw_image_deg"]), projector
                ),
                area_px=int(detection["area_px"]),
                confidence=float(detection["predicted_iou"])
                * float(detection["stability_score"]),
            )
        )
    # Start near the home pose and prefer high-quality masks when distances tie.
    home_xy = np.array([0.30, 0.0])
    targets.sort(key=lambda item: (np.linalg.norm(item.world_xy - home_xy), -item.confidence))
    return targets, rejected


def segments(
    target: PickTarget,
    home: np.ndarray,
    pick_z: float,
    drop_xy: np.ndarray,
    drop_z: float,
    jaw_close: float,
) -> list[tuple[str, np.ndarray, np.ndarray, float, float, float, np.ndarray | None]]:
    p = np.array([target.world_xy[0], target.world_xy[1], pick_z])
    d = np.array([drop_xy[0], drop_xy[1], drop_z])
    up, above = p + [0.0, 0.0, 0.13], d + [0.0, 0.0, 0.13]
    destination = np.array([0.0, 1.0, 0.0])
    if np.dot(target.spread_dir, destination) < 0:
        destination = -destination
    opened = C.JAW_OPEN
    return [
        ("approach", home, up, 1.2, opened, opened, target.spread_dir),
        ("descend", up, p, 0.8, opened, opened, target.spread_dir),
        ("close", p, p, 0.6, opened, jaw_close, target.spread_dir),
        ("dwell", p, p, 0.5, jaw_close, jaw_close, target.spread_dir),
        ("lift", p, up, 0.8, jaw_close, jaw_close, target.spread_dir),
        ("carry", up, above, 1.4, jaw_close, jaw_close, destination),
        ("lower", above, d, 0.8, jaw_close, jaw_close, destination),
        ("release", d, d, 0.6, jaw_close, opened, destination),
        ("settle", d, d, 0.6, opened, opened, destination),
        ("retreat", d, above, 0.8, opened, opened, destination),
        ("home", above, home, 1.1, opened, opened, None),
    ]


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
        target, home, pick_z, drop_xy, drop_z, jaw_close
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


class SamClient:
    def __init__(
        self,
        python: Path,
        worker: Path,
        model_id: str,
        points_per_side: int,
        points_per_batch: int,
    ) -> None:
        command = [
            str(python.expanduser()),
            "-u",
            str(worker),
            "--model-id",
            model_id,
            "--points-per-side",
            str(points_per_side),
            "--points-per-batch",
            str(points_per_batch),
        ]
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        ready = self._read_response()
        if not ready.get("ok") or ready.get("status") != "ready":
            raise RuntimeError(f"SAM worker failed to start: {ready}")
        print(
            f"[vision] SAM ready: {ready['model_id']} on {ready['device']}", flush=True
        )

    def _read_response(self) -> dict[str, Any]:
        if self.process.stdout is None:
            raise RuntimeError("SAM worker stdout is unavailable")
        while True:
            line = self.process.stdout.readline()
            if not line:
                raise RuntimeError(f"SAM worker exited with code {self.process.poll()}")
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                print(f"[sam2 stdout] {line.rstrip()}", flush=True)

    def detect(self, request: dict[str, Any]) -> dict[str, Any]:
        if self.process.stdin is None:
            raise RuntimeError("SAM worker stdin is unavailable")
        self.process.stdin.write(json.dumps(request, separators=(",", ":")) + "\n")
        self.process.stdin.flush()
        response = self._read_response()
        if not response.get("ok"):
            raise RuntimeError(f"SAM inference failed: {response.get('error', response)}")
        return response

    def close(self) -> None:
        if self.process.poll() is not None:
            return
        try:
            if self.process.stdin is not None:
                self.process.stdin.write('{"command":"shutdown"}\n')
                self.process.stdin.flush()
            self.process.wait(timeout=5)
        except Exception:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()


def parse_args() -> argparse.Namespace:
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="Enable robot motion.")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--start-delay", type=float, default=10.0)
    parser.add_argument("--max-picks", type=int, default=10)
    parser.add_argument("--homography", type=Path)
    parser.add_argument("--sam-python", type=Path, default=Path.home() / "venvs/sam2-check/bin/python")
    parser.add_argument("--sam-worker", type=Path, default=base / "vision/sam2_worker.py")
    parser.add_argument("--model-id", default="facebook/sam2.1-hiera-small")
    parser.add_argument("--points-per-side", type=int, default=48)
    parser.add_argument("--points-per-batch", type=int, default=32)
    parser.add_argument("--roi", help="Camera ROI as x1,y1,x2,y2 pixels; default is full frame.")
    parser.add_argument("--min-area-px", type=int, default=100)
    parser.add_argument("--max-area-fraction", type=float, default=0.08)
    parser.add_argument("--max-objects", type=int, default=30)
    parser.add_argument("--border-margin-px", type=int, default=3)
    parser.add_argument("--resolution", default="1280,720", help="width,height")
    parser.add_argument("--camera-eye", default="0.379,0.032,2.20", help="world x,y,z")
    parser.add_argument("--camera-target", default="0.379,0.032,1.00", help="world x,y,z")
    parser.add_argument("--camera-focal-length", type=float, default=50.0)
    parser.add_argument(
        "--workspace",
        default="0.05,0.58,-0.30,0.30",
        help="reachable xmin,xmax,ymin,ymax in world meters",
    )
    parser.add_argument("--drop-xy", default=None, help="dump x,y; defaults to C.CUBE_STACK_XY")
    parser.add_argument("--drop-exclusion-radius", type=float, default=0.06)
    parser.add_argument(
        "--pick-z",
        type=float,
        default=float(C.SURFACE_DZ + C.CUBE_TIP_CLEARANCE),
        help="tool-tip Z for first cube test",
    )
    parser.add_argument(
        "--drop-z",
        type=float,
        default=float(C.SURFACE_DZ + C.CUBE_TIP_CLEARANCE + 0.08),
        help="tool-tip Z while releasing into the dump area",
    )
    parser.add_argument("--jaw-close", type=float, default=float(C.CUBE_JAW_HALF_TARGET))
    parser.add_argument("--object-match-radius", type=float, default=0.020)
    parser.add_argument(
        "--target-type",
        choices=("cube", "cap", "vial", "any"),
        default="cube",
        help="Simulation validation gate for the first object type to pick.",
    )
    args = parser.parse_args()
    args.resolution = tuple(int(v) for v in parse_csv(args.resolution, 2, "--resolution"))
    args.camera_eye = parse_csv(args.camera_eye, 3, "--camera-eye")
    args.camera_target = parse_csv(args.camera_target, 3, "--camera-target")
    args.workspace = parse_csv(args.workspace, 4, "--workspace")
    args.drop_xy = (
        np.asarray(C.CUBE_STACK_XY, dtype=float)
        if args.drop_xy is None
        else np.asarray(parse_csv(args.drop_xy, 2, "--drop-xy"), dtype=float)
    )
    return args


def main() -> None:
    args = parse_args()
    print("[vision] depth-coordinate validation revision (2026-10-13)", flush=True)
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

        bodies = [
            prim
            for prim in stage.Traverse()
            if str(prim.GetPath()).startswith(
                ("/World/SortCube", "/World/SortCap", "/World/SortVial")
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

        def depth_projector(depth: np.ndarray) -> Callable[[float, float], np.ndarray]:
            def project(u: float, v: float) -> np.ndarray:
                ui = int(np.clip(round(u), 0, image_width - 1))
                vi = int(np.clip(round(v), 0, image_height - 1))
                z_forward = float(depth[vi, ui])
                if not np.isfinite(z_forward) or z_forward <= 0.0:
                    raise ValueError(f"No valid depth at pixel ({u:.1f}, {v:.1f})")
                # USD cameras use -Z forward, +Y up. The depth annotator reports
                # distance to the image plane, so standard pinhole unprojection applies.
                camera_point = Gf.Vec3d(
                    (u - cx) * z_forward / fx,
                    -(v - cy) * z_forward / fy,
                    -z_forward,
                )
                world_point = camera_to_world.Transform(camera_point)
                return np.array([world_point[0], world_point[1]], dtype=np.float64)

            return project

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

            projector = (
                (lambda u, v: pixel_to_world(u, v, homography))
                if homography is not None
                else depth_projector(depth)
            )
            targets, rejected = build_targets(
                detections,
                projector,
                args.workspace,
                args.drop_xy,
                args.drop_exclusion_radius,
            )
            cycle_report["rejected"] = rejected
            cycle_report["world_targets"] = [
                {
                    "id": item.detection_id,
                    "pixel_uv": list(item.pixel_uv),
                    "world_xy": item.world_xy.tolist(),
                    "area_px": item.area_px,
                    "confidence": item.confidence,
                }
                for item in targets
            ]
            print(
                "[vision] world candidates: "
                + ", ".join(
                    f"#{item.detection_id}=({item.world_xy[0]:.3f},{item.world_xy[1]:.3f})"
                    for item in targets
                ),
                flush=True,
            )
            validation = []
            for item in targets:
                match = nearest_body(item.world_xy)
                validation.append(
                    {
                        "detection_id": item.detection_id,
                        "matched_body": None if match is None else match[0],
                        "xy_error_mm": None if match is None else match[3] * 1000,
                    }
                )
            cycle_report["simulation_validation"] = validation
            valid_matches = [item for item in validation if item["matched_body"] is not None]
            max_match_error = max(
                (item["xy_error_mm"] for item in valid_matches), default=float("nan")
            )
            print(
                f"[vision] simulation matches={len(valid_matches)}/{len(targets)} "
                f"max_xy_error_mm={max_match_error:.2f}",
                flush=True,
            )
            if not args.execute:
                print("[vision] Dry-run target conversion complete; robot motion disabled.", flush=True)
                report["complete"] = True
                report["stop_reason"] = "world_coordinate_dry_run"
                break
            if not targets:
                print("[vision] No reachable objects remain. Done.", flush=True)
                report["complete"] = True
                break

            selected = None
            for candidate in targets:
                candidate_match = nearest_body(candidate.world_xy)
                if candidate_match is None:
                    continue
                candidate_path = candidate_match[0].lower()
                if args.target_type == "any" or args.target_type in candidate_path:
                    selected = (candidate, candidate_match)
                    break
            if selected is None:
                raise RuntimeError(
                    f"No {args.target_type} detection matched a simulated object within "
                    f"{args.object_match_radius * 1000:.0f} mm; refusing to move"
                )
            target, matched = selected
            body_path, body, before, match_distance = matched
            cycle_report.update(
                selected_detection_id=target.detection_id,
                selected_world_xy=target.world_xy.tolist(),
                matched_body=body_path,
                match_error_mm=match_distance * 1000,
            )
            phases, worst = plan_trajectory(
                target,
                initial,
                home,
                limits,
                tool,
                args.pick_z,
                args.drop_xy,
                args.drop_z,
                args.jaw_close,
            )
            cycle_report["max_numerical_ik_error_mm"] = worst * 1000
            print(
                f"[vision] PICK #{target.detection_id} world=({target.world_xy[0]:.3f},"
                f"{target.world_xy[1]:.3f}) body={body_path}",
                flush=True,
            )
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
                if name == "lift":
                    lift = float(body.get_world_pose()[0][2] - before[2])
                    cycle_report["lift_height_mm"] = lift * 1000
                    cycle_report["pickup_verified"] = lift > 0.025
                    print(
                        f"[vision] {body_path} rose {lift * 1000:.1f} mm; "
                        f"pickup={cycle_report['pickup_verified']}",
                        flush=True,
                    )
                    if not cycle_report["pickup_verified"]:
                        raise RuntimeError("Object did not lift; stopping before carry")
            cycle_report["released"] = True
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


if __name__ == "__main__":
    main()