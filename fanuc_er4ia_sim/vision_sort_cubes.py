#!/usr/bin/env python3
"""Two-view, cube-only color sorting MVP for the existing Isaac Sim cell.

Place this script beside cell.py, fk.py, ik.py and scene_config.py. Reuses
vision/sam2_worker.py unchanged; color comes from a small RGB center patch.
Cube eligibility and outcome checks still use simulated bodies, NOT a learned
cube classifier. This is an Isaac Sim script, not a physical-robot driver.

Required settings: --colors COLOR_A,COLOR_B --drop-b X,Y
--drop-a X,Y defaults to the existing C.CUBE_STACK_XY. Coordinates are meters.
Supported colors: red, orange, yellow, green, cyan, blue, purple, magenta,
black, white, gray. Inspect printed median RGB and both camera images first.

Run with the same Isaac Python/environment as the original script:
  python -u vision_sort_cubes.py --colors red,blue --drop-b X,Y
    Static dry run: one image, no sorting or observation moves.
  python -u vision_sort_cubes.py --colors red,blue --drop-b X,Y --execute --scan-only
    Move through BOTH scan poses, merge detections, plan queued picks; no grasp.
  python -u vision_sort_cubes.py --colors red,blue --drop-b X,Y --execute --once
    Scan twice and sort one batch. Use this for the first twin test.
  python -u vision_sort_cubes.py --colors red,blue --drop-b X,Y --execute
    Repeat batches, waiting 60 wall-clock seconds after a batch. Ctrl+C stops.
Add --two-color-demo to temporarily recolor unselected cube colors as COLOR_A
in this simulated scene (e.g. yellow becomes red for the six-cube test).
Replace example colors and X,Y with the actual colors and checked destination.

The second scan defaults to TCP (0.30, 0.00, 0.38), pointing down; override
with --second-scan-xyz X,Y,Z if another pose reveals more table.
New simulated cubes must have /World/SortCube... rigid-body paths, as in the
starter scene. Both destination circles are excluded from subsequent picks.
Cube travel uses pick/drop clearance heights, separate from the scan height.
The RGB/depth camera updates only during scans; the viewport remains live.
The report is checkpointed BEFORE motion phases to survive native crashes.
IK checks reachability, not collision freedom; verify both scan
poses and the new travel paths in the digital twin before hardware integration.
"""

from __future__ import annotations

import argparse
import colorsys
import json
import math
import os
import subprocess
import time
import traceback
from dataclasses import dataclass, replace
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
    raw = payload.get("pixel_to_world_homography", payload) if isinstance(payload, dict) else payload
    matrix = np.asarray(raw, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError("Homography JSON must contain a 3x3 pixel_to_world_homography")
    return matrix


def write_report(path: Path, report: dict[str, Any]) -> None:
    """Keep the last complete checkpoint readable even if a native call fails."""
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def capture_frame(render_product, rgb_annotator, depth_annotator, step, shape):
    """Enable the perception render product only long enough for a fresh frame.

    NVIDIA's Replicator API supports gating its Hydra texture independently of
    the viewport. Copy annotator buffers BEFORE disabling texture updates.
    """
    render_product.hydra_texture.set_updates_enabled(True)
    try:
        for _ in range(8):
            step()
        rgb = np.array(rgb_annotator.get_data(), copy=True)
        depth = np.array(depth_annotator.get_data(), dtype=np.float64, copy=True)
        if depth.ndim == 3 and depth.shape[2] == 1:
            depth = depth[..., 0]
        if rgb.ndim != 3 or rgb.shape[:2] != shape or rgb.shape[2] < 3:
            raise RuntimeError(f"RGB camera returned invalid shape {rgb.shape}")
        if depth.shape != shape:
            raise RuntimeError(f"Depth camera returned invalid shape {depth.shape}")
        return rgb, depth
    finally:
        render_product.hydra_texture.set_updates_enabled(False)


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


# Hue references in degrees; classify all colors before checking the selected pair.
HUES = {"red": 0, "orange": 30, "yellow": 60, "green": 120,
        "cyan": 180, "blue": 240, "purple": 270, "magenta": 300}


def pixel_color(rgb: np.ndarray, uv: tuple[float, float]) -> tuple[str, tuple[int, ...]]:
    """Median of a 7x7 center patch: no SAM worker schema changes required."""
    u, v = (int(round(x)) for x in uv)
    h, w = rgb.shape[:2]
    if not (0 <= u < w and 0 <= v < h):
        return "unknown", (0, 0, 0)
    patch = rgb[max(0, v-3):min(h, v+4), max(0, u-3):min(w, u+4), :3]
    values = np.median(patch.reshape(-1, 3), axis=0)
    sample = tuple(int(x) for x in values)
    hue, saturation, value = colorsys.rgb_to_hsv(*(values / 255.0))
    if value < 0.16:
        return "black", sample
    if saturation < 0.20:
        return ("white" if value > 0.72 else "gray"), sample
    degrees = hue * 360.0
    color = min(HUES, key=lambda name: abs((degrees - HUES[name] + 180) % 360 - 180))
    return color, sample


@dataclass(frozen=True)
class PickTarget:
    detection_id: int
    pixel_uv: tuple[float, float]
    world_xy: np.ndarray
    spread_dir: np.ndarray
    area_px: int
    confidence: float
    color: str
    rgb: tuple[int, ...]
    view: str
    body_path: str = ""
    observed_xyz: np.ndarray | None = None


def in_drop_area(xy: np.ndarray, drops: dict[str, np.ndarray], radius: float) -> bool:
    return any(np.linalg.norm(xy - center) <= radius for center in drops.values())


def build_targets(detections, projector, workspace, drops, radius, rgb, view):
    xmin, xmax, ymin, ymax = workspace
    targets, rejected = [], []
    for detection in detections:
        u, v = (float(value) for value in detection["center_full_px"])
        color, sample = pixel_color(rgb, (u, v))
        try:
            xy = projector(u, v)
            reason = None
            if not np.all(np.isfinite(xy)):
                reason = "invalid_depth"
            elif not (xmin <= xy[0] <= xmax and ymin <= xy[1] <= ymax):
                reason = "outside_workspace"
            elif in_drop_area(xy, drops, radius):
                reason = "inside_drop_area"
            elif color not in drops:
                reason = "unselected_color"
            if reason is not None:
                rejected.append({"id": detection["id"], "reason": reason,
                                 "color": color, "rgb": sample})
                continue
            spread = image_axis_to_world_spread(
                u, v, float(detection["grasp_yaw_image_deg"]), projector)
            targets.append(PickTarget(
                int(detection["id"]), (u, v), xy, spread,
                int(detection["area_px"]),
                float(detection["predicted_iou"]) * float(detection["stability_score"]),
                color, sample, view))
        except ValueError as exc:
            rejected.append({"id": detection["id"], "reason": str(exc)})
    return targets, rejected


def merge_targets(targets: list[PickTarget]) -> list[PickTarget]:
    """Use existing simulated-body matches to merge repeat masks/across views.

    Distinct touching cubes stay distinct. Conflicting color observations are
    left for a later scan instead of arbitrarily selecting a destination.
    """
    merged, conflicts = {}, set()
    for target in targets:
        old = merged.get(target.body_path)
        if old is not None and old.color != target.color:
            conflicts.add(target.body_path)
        if old is None or target.confidence > old.confidence:
            merged[target.body_path] = target
    return [target for path, target in merged.items() if path not in conflicts]



def segments(target, start, pick_z, drop_xy, drop_z, jaw_close, travel_z):
    """Move vertically to clearance, travel, pick; finish above drop, not home."""
    p = np.array([*target.world_xy, pick_z])
    d = np.array([*drop_xy, drop_z])
    up, above = p.copy(), d.copy()
    up[2] = above[2] = travel_z
    raised = start.copy()
    raised[2] = travel_z
    destination = np.array([0.0, 1.0, 0.0])
    if np.dot(target.spread_dir, destination) < 0:
        destination = -destination
    opened = C.JAW_OPEN
    phases = [
        ("clear", start, raised, 0.8, opened, opened, None),
        ("approach", raised, up, 1.2, opened, opened, target.spread_dir),
        ("descend", up, p, 0.8, opened, opened, target.spread_dir),
        ("close", p, p, 0.6, opened, jaw_close, target.spread_dir),
        ("dwell", p, p, 0.5, jaw_close, jaw_close, target.spread_dir),
        ("lift", p, up, 0.8, jaw_close, jaw_close, target.spread_dir),
        ("carry", up, above, 1.4, jaw_close, jaw_close, destination),
        ("lower", above, d, 0.8, jaw_close, jaw_close, destination),
        ("release", d, d, 0.6, jaw_close, opened, destination),
        ("settle", d, d, 0.6, opened, opened, destination),
        ("retreat", d, above, 0.8, opened, opened, destination),
    ]
    if np.linalg.norm(start - raised) < 0.001:
        phases.pop(0)
    return phases


def plan_phases(initial, limits, tool, specifications):
    """Original numerical IK checks, shared by scans and cube trajectories."""
    down = np.array([0.0, 0.0, -1.0])
    q = initial.copy()
    phases, worst = [], 0.0
    for name, p0, p1, duration, j0, j1, direction in specifications:
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
                q, ok = ik.solve(q, position, down, limits, tool_offset=tool,
                                spread_dir=desired, iters=500, damping=0.02)
            rotation, actual = fk.tool_pose(q, tool)
            error = float(np.linalg.norm(actual - position))
            worst = max(worst, error)
            aligned = desired is None or abs(
                np.dot(rotation @ np.asarray(C.FINGER_SPREAD_AXIS), desired)) > 0.999
            approach_ok = np.dot(rotation @ np.asarray(C.TOOL_AXIS), down) > 0.999
            if not ok or error > 0.001 or not aligned or not approach_ok:
                raise RuntimeError(f"IK rejected {name} frame {index}: {error * 1000:.3f} mm")
            steps.append((q.copy(), float(j0 + (j1 - j0) * t), position.copy()))
        phases.append((name, steps))
    return phases, worst


def plan_trajectory(target, initial, limits, tool, pick_z, drop_xy, drop_z,
                    jaw_close, travel_z):
    spread = target.spread_dir.copy()
    if np.dot(fk.fk(initial)[0] @ np.asarray(C.FINGER_SPREAD_AXIS), spread) < 0:
        spread = -spread
    target = replace(target, spread_dir=spread)
    start = fk.tool_pose(initial, tool)[1].copy()
    return plan_phases(initial, limits, tool,
                       segments(target, start, pick_z, drop_xy, drop_z, jaw_close, travel_z))


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
    parser.add_argument("--max-picks", type=int, default=0, help="0 = continuous")
    parser.add_argument("--once", action="store_true", help="Sort one two-scan batch, then stop")
    parser.add_argument("--scan-only", action="store_true", help="With --execute: two scans and IK planning, no picks")
    parser.add_argument("--poll-seconds", type=float, default=60.0)
    parser.add_argument("--colors", required=True, help="Two colors, e.g. red,blue; in drop-a,drop-b order")
    parser.add_argument("--two-color-demo", action="store_true",
                        help="Simulation only: recolor unselected cubes as the first color")
    parser.add_argument("--second-scan-xyz", default="0.30,0.00,0.38",
                        help="Second observation TCP, meters; default 0.30,0.00,0.38")
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
    parser.add_argument("--drop-a", default=None, help="First color x,y; default C.CUBE_STACK_XY")
    parser.add_argument("--drop-b", required=True, help="Second color x,y; choose a clear, reachable area")
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
    args = parser.parse_args()
    args.resolution = tuple(int(v) for v in parse_csv(args.resolution, 2, "--resolution"))
    args.camera_eye = parse_csv(args.camera_eye, 3, "--camera-eye")
    args.camera_target = parse_csv(args.camera_target, 3, "--camera-target")
    args.workspace = parse_csv(args.workspace, 4, "--workspace")
    args.colors = tuple(x.strip().lower() for x in args.colors.split(","))
    known = set(HUES) | {"black", "white", "gray"}
    if len(args.colors) != 2 or len(set(args.colors)) != 2 or not set(args.colors) <= known:
        parser.error("--colors needs two different supported color names; see --help")
    drop_a = (np.asarray(C.CUBE_STACK_XY, dtype=float) if args.drop_a is None else
              np.asarray(parse_csv(args.drop_a, 2, "--drop-a")))
    drop_b = np.asarray(parse_csv(args.drop_b, 2, "--drop-b"))
    args.drops = dict(zip(args.colors, (drop_a, drop_b)))
    if not np.all(np.isfinite([drop_a, drop_b])):
        parser.error("Drop coordinates must be finite")
    if (not np.all(np.isfinite([args.drop_exclusion_radius, args.object_match_radius]))
            or args.drop_exclusion_radius <= 0 or args.object_match_radius <= 0):
        parser.error("Exclusion and matching radii must be positive")
    if np.linalg.norm(drop_a - drop_b) <= 2 * args.drop_exclusion_radius:
        parser.error("Drop areas must not overlap; increase their separation")
    if args.max_picks < 0 or not np.isfinite(args.poll_seconds) or args.poll_seconds <= 0:
        parser.error("--max-picks must be >= 0 and --poll-seconds positive")
    args.second_scan_xyz = np.asarray(parse_csv(args.second_scan_xyz, 3, "--second-scan-xyz"))
    return args


def main() -> None:
    args = parse_args()
    print("[sort] Cube sorter revision 2; camera paused between scans", flush=True)
    if args.two_color_demo:
        C.SORT_CUBES = [(x, y, color if color in args.colors else args.colors[0])
                        for x, y, color in C.SORT_CUBES]
        print(f"[sort] Two-color demo scene: {args.colors}; config file unchanged", flush=True)
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
    neutral = args.second_scan_xyz
    if not np.all(np.isfinite(neutral)) or np.linalg.norm(neutral - home) < 0.05:
        raise ValueError("Choose a distinct --second-scan-xyz observation position")
    # Scan poses are high and near the base. Carrying far cubes at that same
    # height unnecessarily approaches the arm's reach limit.
    travel_z = max(args.pick_z, args.drop_z) + 0.13
    scan_travel_z = max(home[2], neutral[2], travel_z)
    initial, ok = ik.solve(initial, home, down, limits, tool_offset=tool)
    if not ok:
        raise RuntimeError("Home IK failed")

    base = Path(__file__).resolve().parent
    output = base / "runs" / ("vision_sort_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    output.mkdir(parents=True)
    report: dict[str, Any] = {
        "test": "sam2_two_view_cube_color_sort",
        "revision": 2,
        "colors_and_drops": {color: xy.tolist() for color, xy in args.drops.items()},
        "observation_tcp": {"right": home.tolist(), "neutral": neutral.tolist()},
        "travel_z": travel_z,
        "scan_travel_z": scan_travel_z,
        "two_color_demo": args.two_color_demo,
        "active_phase": "startup",
        "scans": [],
        "sorted_counts": {color: 0 for color in args.colors},
        "execute": bool(args.execute),
        "complete": False,
        "picks": [],
        "error": None,
    }
    report_path = output / "vision_sort_report.json"

    def save_report():
        write_report(report_path, report)

    def mark_phase(name):
        report["active_phase"] = name
        save_report()
        print(f"[sort] {name}", flush=True)

    save_report()
    print(f"[sort] Report/checkpoint: {report_path}", flush=True)

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
        # Keep the GUI viewport active, but stop unnecessary RGB/depth renders
        # during arm motion. Re-enabled inside capture_frame only.
        render_product.hydra_texture.set_updates_enabled(False)

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
            # Preserve the working baseline's physics cadence and GUI updates.
            # Perception rendering is gated separately by capture_frame().
            world.step(render=render)
            if render and not args.headless:
                time.sleep(max(0.0, 1 / 30 - (time.monotonic() - started)))

        def capture(frame_dir: Path) -> tuple[Path, np.ndarray, np.ndarray]:
            frame_dir.mkdir(parents=True, exist_ok=True)
            hold_q = measured()[arm_idx]
            data, depth = capture_frame(render_product, rgb_annotator, depth_annotator,
                lambda: tick(hold_q, C.JAW_OPEN, render=True), (image_height, image_width))
            image_path = frame_dir / "camera_rgb.png"
            rep.functional.write_image(path=str(image_path), data=data)
            return image_path, depth, data[..., :3]

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
            # Refresh so cubes added while idle are recognized on the next scan.
            for prim in stage.Traverse():
                path = str(prim.GetPath())
                if not path.startswith(("/World/SortCube", "/World/SortCap", "/World/SortVial")):
                    continue
                if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    continue
                body = SingleXFormPrim(path)
                position = body.get_world_pose()[0].copy()
                choices.append((float(np.linalg.norm(position[:2] - xy)), path, body, position))
            if not choices:
                return None
            distance, path, body, position = min(choices, key=lambda item: item[0])
            if distance > args.object_match_radius:
                return None
            return path, body, position, distance

        mark_phase("scene_warmup")
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

        def move_to(position):
            q = measured()[arm_idx]
            start = fk.tool_pose(q, tool)[1].copy()
            if np.linalg.norm(start - position) < 0.001:
                return
            raised, above = start.copy(), np.asarray(position).copy()
            raised[2] = above[2] = scan_travel_z
            opened = C.JAW_OPEN
            specs = [("scan_raise", start, raised, 0.8, opened, opened, None),
                     ("scan_travel", raised, above, 1.2, opened, opened, None),
                     ("scan_lower", above, position, 0.8, opened, opened, None)]
            phases, _ = plan_phases(q, limits, tool, specs)
            for name, steps in phases:
                mark_phase(name)
                for q, jaw, _ in steps:
                    tick(q, jaw, render=True)
                error = np.linalg.norm(fk.tool_pose(measured()[arm_idx], tool)[1] - steps[-1][2])
                if error > 0.003:
                    raise RuntimeError(f"{name} tracking error: {error*1000:.2f} mm")
            for _ in range(15):
                tick(q, C.JAW_OPEN, render=True)

        def scan(batch, view):
            frame_dir = output / f"batch_{batch:03d}_{view}"
            mark_phase(f"batch_{batch}_{view}_capture")
            image_path, depth, rgb = capture(frame_dir)
            mark_phase(f"batch_{batch}_{view}_sam")
            response = sam.detect({
                "image": str(image_path), "output_dir": str(frame_dir), "roi": args.roi,
                "min_area_px": args.min_area_px, "max_area_fraction": args.max_area_fraction,
                "max_objects": args.max_objects, "border_margin_px": args.border_margin_px,
            })
            projector = ((lambda u, v: pixel_to_world(u, v, homography))
                         if homography is not None else depth_projector(depth))
            targets, rejected = build_targets(response["detections"], projector,
                args.workspace, args.drops, args.drop_exclusion_radius, rgb, view)
            eligible = []
            for target in targets:
                match = nearest_body(target.world_xy)
                if match is None or not match[0].startswith("/World/SortCube"):
                    rejected.append({"id": target.detection_id, "reason": "not_matched_cube"})
                    continue
                path, _, position, distance = match
                eligible.append(replace(target, body_path=path, observed_xyz=position.copy()))
                print(f"[sort] {view} {path} color={target.color} rgb={target.rgb} "
                      f"xy={target.world_xy.round(4).tolist()} error_mm={distance*1000:.1f}", flush=True)
            report["scans"].append({
                "batch": batch, "view": view, "image": str(image_path),
                "overlay": response.get("overlay"), "rejected": rejected,
                "targets": [{"body": t.body_path, "color": t.color, "rgb": t.rgb,
                             "xy": t.world_xy.tolist()} for t in eligible],
            })
            save_report()
            return eligible

        batch, picks = 0, 0
        while app.is_running():
            if args.execute:
                move_to(home)
            candidates = scan(batch, "right")
            if args.execute:
                move_to(neutral)
                candidates += scan(batch, "neutral")
            queue = merge_targets(candidates)
            print(f"[sort] batch={batch} unique_cubes={len(queue)}", flush=True)
            if not args.execute:
                report["complete"] = True
                report["stop_reason"] = "static_dry_run_one_view"
                break
            if args.scan_only:
                # Plan every queued pick in sequence without actuating the gripper.
                q = measured()[arm_idx]
                while queue:
                    current_xy = fk.tool_pose(q, tool)[1][:2]
                    queue.sort(key=lambda t: np.linalg.norm(t.world_xy - current_xy))
                    target = queue.pop(0)
                    phases, _ = plan_trajectory(target, q, limits, tool, args.pick_z,
                        args.drops[target.color], args.drop_z, args.jaw_close, travel_z)
                    q = phases[-1][1][-1][0]
                report["complete"] = True
                report["stop_reason"] = "two_scans_and_queue_ik_checked"
                break
            stale_queue = False
            while queue and (args.max_picks == 0 or picks < args.max_picks):
                # Choose from current TCP; no default/home return between picks.
                current_xy = fk.tool_pose(measured()[arm_idx], tool)[1][:2]
                queue.sort(key=lambda t: np.linalg.norm(t.world_xy - current_xy))
                target = queue.pop(0)
                matched = nearest_body(target.world_xy)
                if (matched is None or matched[0] != target.body_path or
                        np.linalg.norm(matched[2] - target.observed_xyz) > 0.003):
                    print("[sort] Queued cube moved; rescan instead of using stale coordinates.", flush=True)
                    stale_queue = True
                    break
                body_path, body, before, match_distance = matched
                drop_xy = args.drops[target.color]
                phases, worst = plan_trajectory(target, measured()[arm_idx], limits, tool,
                    args.pick_z, drop_xy, args.drop_z, args.jaw_close, travel_z)
                pick_report = {"batch": batch, "body": body_path, "color": target.color,
                    "rgb": target.rgb, "view": target.view, "world_xy": target.world_xy.tolist(),
                    "drop_xy": drop_xy.tolist(), "match_error_mm": match_distance * 1000,
                    "max_numerical_ik_error_mm": worst * 1000, "pickup_verified": False,
                    "sorted_verified": False}
                report["picks"].append(pick_report)
                print(f"[sort] PICK {body_path} -> {target.color} {drop_xy.tolist()}", flush=True)
                for name, steps in phases:
                    pick_report["phase"] = name
                    pick_report["phase_state"] = "started"
                    pick_report["frame"] = 0
                    mark_phase(f"{body_path}:{name}")
                    for index, (q, jaw, position) in enumerate(steps):
                        if index % 30 == 0:
                            pick_report["frame"] = index
                            save_report()
                        tick(q, jaw, render=True)
                    error = float(np.linalg.norm(fk.tool_pose(measured()[arm_idx], tool)[1] - position))
                    if name in ("descend", "lower", "retreat") and error > 0.003:
                        raise RuntimeError(f"{name} tracking error: {error*1000:.2f} mm")
                    if name == "lift":
                        lift = float(body.get_world_pose()[0][2] - before[2])
                        pick_report["lift_height_mm"] = lift * 1000
                        pick_report["pickup_verified"] = lift > 0.025
                        if not pick_report["pickup_verified"]:
                            raise RuntimeError("Cube did not lift; stopping before carry")
                    pick_report["phase_state"] = "completed"
                    pick_report["frame"] = len(steps)
                    save_report()
                # Hold ABOVE the destination instead of commanding the home pose.
                mark_phase(f"{body_path}:verify_release")
                for _ in range(30):
                    tick(q, C.JAW_OPEN, render=True)
                after = np.asarray(body.get_world_pose()[0], dtype=float)
                pick_report["final_xyz"] = after.tolist()
                pick_report["sorted_verified"] = bool(
                    np.linalg.norm(after[:2] - drop_xy) <= args.drop_exclusion_radius
                    and abs(after[2] - before[2]) < 0.06)
                if not pick_report["sorted_verified"]:
                    raise RuntimeError("Cube did not settle in its color area; inspect the twin")
                report["sorted_counts"][target.color] += 1
                picks += 1
                save_report()
            batch += 1
            if args.max_picks and picks >= args.max_picks:
                report["complete"], report["stop_reason"] = True, "max_picks"
                break
            if args.once:
                report["complete"] = not stale_queue
                report["stop_reason"] = "queue_moved_rerun_batch" if stale_queue else "one_batch"
                break
            if stale_queue:
                continue
            move_to(home)
            mark_phase("idle")
            print(f"[sort] Waiting {args.poll_seconds:g}s; counts={report['sorted_counts']}", flush=True)
            deadline = time.monotonic() + args.poll_seconds
            hold_q = measured()[arm_idx]
            while time.monotonic() < deadline:
                started = time.monotonic()
                tick(hold_q, C.JAW_OPEN, render=True)
                time.sleep(max(0.0, 1/30 - (time.monotonic() - started)))

    except KeyboardInterrupt:
        report["stop_reason"] = "user_interrupt"
        print("[sort] Interrupted; saving report.", flush=True)

    except Exception as exc:
        report["error"] = str(exc)
        traceback.print_exc()
    finally:
        if report["complete"]:
            report["active_phase"] = "finished"
        save_report()
        print(f"[sort] Verified placements: {report['sorted_counts']}", flush=True)
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
        if not args.headless and app.is_running() and report.get("stop_reason") != "user_interrupt":
            print("[vision] Finished; window remains open. Close it when done.", flush=True)
            while app.is_running():
                app.update()
                time.sleep(0.02)
        app.close()


if __name__ == "__main__":
    main()
