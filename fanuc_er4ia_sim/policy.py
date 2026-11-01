"""Physical robot policy: copy beside the organizer's run.py.

Default: save detections and hold the current pose. Configure sort_config.py
before enabling movement. No simulator ground truth is imported here.
"""
from __future__ import annotations

import atexit
from collections import deque
from datetime import datetime
import json
import math
from pathlib import Path
import sys

import numpy as np
from calibration.geometry import load, project as camera_project, pixel_to_base
import sort_config as S
from sort_vision import SamClient, Attempts, build_candidates, write_overlay, roi_from_pixels


def base_to_tcp(x, y, z, r=0.0, grip=0.0):
    return [x * 1000, y * 1000, z * 1000 + S.UF0_Z_MM, -180.0, 0.0, r, grip]


def angle_error(a, b):
    return (np.asarray(a) - b + 180.0) % 360.0 - 180.0


def interpolate(start, end):
    """Small absolute TCP targets, <=100 per network chunk, never premature closure."""
    a, b = np.asarray(start, float), np.asarray(end, float)
    delta = b - a
    delta[3:6] = angle_error(b[3:6], a[3:6])
    n = max(1, math.ceil(np.linalg.norm(delta[:3]) / S.LINEAR_STEP_MM),
            math.ceil(np.max(np.abs(delta[3:6])) / S.ANGULAR_STEP_DEG))
    rows = []
    for i in range(1, n + 1):
        row = a + delta * (i / n)
        row[6] = a[6] if i < n else b[6]
        rows.append(row.tolist())
    return [rows[i:i+100] for i in range(0, len(rows), 100)]


class Policy:
    CAMERAS = ["table"]
    IMAGE_SIZE = (1280, 720)  # more pixels on the narrow screws than the 640x360 default
    IMAGE_FORMAT = "png"
    ACTION = "tcp"
    PREFETCH = False  # a fresh observation must settle before the next phase
    DEPTH = []

    def __init__(self, checkpoint=None):
        self.cameras = load()
        self.sam = None
        self.model_id = checkpoint or S.MODEL_ID
        self.mock = "--mock" in sys.argv  # organizer run.py starts a LOCAL mock for this flag
        atexit.register(self.close)
        self.reset()

    def close(self):
        if self.sam is not None:
            self.sam.close()
            self.sam = None

    def reset(self):
        self.attempts = Attempts()
        self.queue = deque()
        self.pending = None
        self.wait_calls = 0
        self.settled_calls = 0
        self.state = "start"
        self.selected = None
        self.picks = 0
        self.cycle = 0
        self.halted = False
        self.preview_done = False
        self.output = S.ROOT / "runs" / ("physical_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
        self.output.mkdir(parents=True, exist_ok=True)

    def log(self, event, **data):
        entry = {"event": event, **data}
        print("[sort] " + json.dumps(entry), flush=True)
        with (self.output / "events.jsonl").open("a") as f:
            f.write(json.dumps(entry) + "\n")

    @staticmethod
    def hold(obs):
        return [*map(float, obs["tcp_pose"]), float(obs["joints"][6])]

    def halt(self, obs, reason):
        self.queue.clear()
        self.pending = None
        self.halted = True
        self.log("hold_for_operator", reason=reason)
        return self.hold(obs)

    def detect(self, obs):
        from PIL import Image
        rgb = np.asarray(obs["images"]["table"], dtype=np.uint8)
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("Expected an RGB table image")
        folder = self.output / f"cycle_{self.cycle:03d}"
        folder.mkdir(exist_ok=True)
        self.cycle += 1
        image_path = folder / "table.png"
        Image.fromarray(rgb).save(image_path)
        h, w = rgb.shape[:2]
        cam = self.cameras["table"]
        cw, ch = cam["size"]
        xmin,xmax,ymin,ymax = S.WORKSPACE
        corners = [[x,y,S.SURFACE_Z] for x in (xmin,xmax) for y in (ymin,ymax)]
        roi = roi_from_pixels(camera_project(cam,corners)*[w/cw,h/ch],w,h)
        if self.sam is None:
            self.sam = SamClient(model_id=self.model_id)
        masks = self.sam.detect({"image": str(image_path), "output_dir": str(folder),
                                 "min_area_px": 20, "roi": roi})["detections"]
        def project(u, v, height):
            # Images are resized whole frames, NOT crops. Restore calibration pixels.
            return pixel_to_base(cam, (u * cw / w, v * ch / h), z=S.SURFACE_Z + height)[:2]
        targets, rejected = build_candidates(masks, project)
        write_overlay(image_path, targets, folder / "targets.png")
        (folder / "targets.json").write_text(json.dumps({"targets": targets, "rejected": rejected}, indent=2))
        self.log("detections", count=len(targets), overlay=str(folder / "targets.png"),
                 order=[t["label"] for t in targets])
        return targets

    def enqueue(self, start, poses, next_state):
        here = list(start)
        for pose in poses:
            self.validate(pose)
            for chunk in interpolate(here, pose):
                self.queue.append(chunk)
            here = list(pose)
        self.state = next_state

    @staticmethod
    def validate(pose):
        if len(pose) != 7 or not np.all(np.isfinite(pose)):
            raise ValueError("Invalid TCP target")
        if math.hypot(pose[0], pose[1]) > S.MAX_RADIUS * 1000:
            raise ValueError("Target outside configured radial reach")
        base_z = (pose[2] - S.UF0_Z_MM) / 1000
        if base_z < S.SURFACE_Z + 0.001 or base_z > S.SURFACE_Z + 0.45:
            raise ValueError("Target outside configured height range")

    def motion(self, obs):
        if self.pending is not None:
            now = self.hold(obs)
            position_error = np.linalg.norm(np.array(now[:3]) - self.pending[:3])
            orientation_error = np.max(np.abs(angle_error(now[3:6], self.pending[3:6])))
            # Gripper observation is a commanded state, not proof of contact/release.
            close = (position_error <= S.POSITION_TOLERANCE_MM and
                     orientation_error <= S.ANGLE_TOLERANCE_DEG and
                     abs(now[6] - self.pending[6]) < 0.25)
            self.wait_calls += 1
            self.settled_calls = self.settled_calls + 1 if close else 0
            if self.settled_calls < 3:
                if self.wait_calls > S.MAX_SETTLE_CALLS:
                    return self.halt(obs, "TCP did not settle; check controller/gripper before resuming")
                return self.pending
            self.pending = None
        if self.queue:
            chunk = self.queue.popleft()
            self.pending = chunk[-1]
            self.wait_calls = self.settled_calls = 0
            return chunk
        return None

    def scan_poses(self, current):
        # Lift vertically before changing orientation or moving across the table.
        high = max(current[2], (S.SURFACE_Z + S.TRAVEL_HEIGHT) * 1000 + S.UF0_Z_MM)
        lifted = [*current[:2], high, *current[3:6], current[6]]
        down = [*lifted[:3], -180.0, 0.0, S.TCP_R_FOR_AXIS_X, current[6]]
        scan = base_to_tcp(*S.SCAN_XY, S.SURFACE_Z + S.SCAN_HEIGHT, S.TCP_R_FOR_AXIS_X, current[6])
        return [lifted, down, scan]

    def act(self, obs):
        if self.mock:
            # Their mock has tcp_pose=None until its first TCP command, and its RGB
            # frames are synthetic. Check the wire format without claiming vision validation.
            if obs.get("tcp_pose") is None:
                return [300.0, 0.0, -100.0, -180.0, 0.0, 0.0, 0.0]
            return self.hold(obs)
        if self.halted:
            return self.hold(obs)
        try:
            if not S.PHYSICAL_EXECUTE:
                if not self.preview_done:
                    self.detect(obs)
                    self.preview_done = True
                    self.log("preview_only", message="PHYSICAL_EXECUTE=False; holding current pose")
                return self.hold(obs)
            if not S.PHYSICAL_POSES_CONFIRMED:
                return self.halt(obs, "Set measured scan/drop poses and PHYSICAL_POSES_CONFIRMED first")
            action = self.motion(obs)
            if action is not None:
                return action
            current = self.hold(obs)
            if self.state == "start":
                if current[6] >= 0.5:
                    return self.halt(obs, "Start with an empty, open gripper")
                self.enqueue(current, self.scan_poses(current), "detect")
            elif self.state == "detect":
                if self.picks >= S.MAX_PICKS:
                    return self.halt(obs, "Attempt limit reached")
                choices = self.attempts.available(self.detect(obs))
                if not choices:
                    return self.halt(obs, "No eligible detections; inspect overlay for missed/exhausted objects")
                t = self.selected = choices[0]
                self.attempts.add(t)
                self.picks += 1
                x, y = t["world_xy"]
                r = t["yaw_base_deg"] + S.TCP_R_FOR_AXIS_X
                r = (r + 90) % 180 - 90  # parallel-jaw symmetry
                z = S.SURFACE_Z + S.PICK_HEIGHT[t["kind"]]
                hover = base_to_tcp(x, y, S.SURFACE_Z + S.TRAVEL_HEIGHT, r, 0)
                pick = base_to_tcp(x, y, z, r, 0)
                close = [*pick[:6], 1.0]
                lift = [*hover[:6], 1.0]
                # Reach the hover Z vertically at the scan point before horizontal travel.
                level = [*current[:2], hover[2], *current[3:6], current[6]]
                self.enqueue(current, [level, hover, pick, close, lift], "carry")
                self.log("attempt", label=t["label"], xy=t["world_xy"], drop=t["drop"],
                         pickup_verified=False)
            elif self.state == "carry":
                # Source visibility can show a failed pick; disappearance alone is not proof.
                targets = self.detect(obs)
                t = self.selected
                still_there = any(a["kind"] == t["kind"] and
                    np.linalg.norm(np.array(a["world_xy"]) - t["world_xy"]) < 0.012 for a in targets)
                if still_there:
                    self.log("source_still_detected", label=t["label"])
                    # Return to the original pickup point before opening an uncertain grasp.
                    pick = base_to_tcp(*t["world_xy"], S.SURFACE_Z + S.PICK_HEIGHT[t["kind"]], current[5], 1)
                    opened = [*pick[:6], 0.0]
                    up = base_to_tcp(*t["world_xy"], S.SURFACE_Z + S.TRAVEL_HEIGHT, current[5], 0)
                    self.enqueue(current, [pick, opened, up], "rescan")
                else:
                    xy = S.DROP_XY[t["drop"]]
                    above = base_to_tcp(*xy, S.SURFACE_Z + S.TRAVEL_HEIGHT, current[5], 1)
                    drop = base_to_tcp(*xy, S.SURFACE_Z + S.DROP_HEIGHT, current[5], 1)
                    opened = [*drop[:6], 0.0]
                    retreat = [*above[:6], 0.0]
                    self.enqueue(current, [above, drop, opened, retreat], "released")
                    self.log("carry", pickup_verified=False, reason="source no longer detected")
            elif self.state == "released":
                self.log("release_commanded", label=self.selected["label"], release_verified=False)
                if S.PAUSE_AFTER_LARGE_OBJECT and self.selected["kind"] in ("cap", "vial"):
                    return self.halt(obs, "Cap/vial attempt finished; inspect for hooked object before next rollout")
                self.enqueue(current, self.scan_poses(current), "detect")
            elif self.state == "rescan":
                self.enqueue(current, self.scan_poses(current), "detect")
            action = self.motion(obs)
            return self.hold(obs) if action is None else action
        except Exception as e:
            if not S.PHYSICAL_EXECUTE:
                raise  # report missing dependencies/model errors in preview runs
            return self.halt(obs, f"{type(e).__name__}: {e}")


if __name__ == "__main__":
    import argparse
    from PIL import Image
    parser = argparse.ArgumentParser(description="Detect a saved real TABLE-camera RGB frame; no robot connection.")
    parser.add_argument("--image", required=True, type=Path)
    args = parser.parse_args()
    policy = Policy()
    try:
        policy.detect({"images": {"table": np.asarray(Image.open(args.image).convert("RGB"))}})
    finally:
        policy.close()
