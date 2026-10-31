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
import time

import numpy as np
from calibration.geometry import load, project as camera_project, pixel_to_base
import sort_config as S
import gripper_profile as G
from dual_clearance import ClearanceError, check_path
from sort_vision import (SamClient, Attempts, ScanQueue, target_unchanged,
                         build_candidates, write_overlay, roi_from_pixels)
from camera_check import draw_mapping, check_frame_size


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
        self.drop_counts = {}
        self.drop_reservations = {}
        self.placed_envelopes = []
        self.observed_objects = []
        self.batch_queue = ScanQueue(whole_batch=True)
        self.batch_reference = None
        self.current_rgb = None
        self.batch_number = 0
        self.batch_attempt_start = 0
        self.batch_idle_passes = 0
        self.batch_active = False
        self.cycle = 0
        self.halted = False
        self.preview_done = False
        self.idle_until = None
        self.started = time.monotonic()
        self.output = S.ROOT / "runs" / ("physical_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
        self.output.mkdir(parents=True, exist_ok=True)

    def log(self, event, **data):
        entry = {"event": event, "elapsed_s": round(time.monotonic()-self.started, 3), **data}
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
        check_frame_size((w, h), cam)
        cw, ch = cam["size"]
        xmin,xmax,ymin,ymax = S.WORKSPACE
        corners = [[x,y,S.SURFACE_Z] for x in (xmin,xmax) for y in (ymin,ymax)]
        if S.PHYSICAL_INSPECT_DROP_AREAS:
            for x, y in S.DROP_XY.values():
                corners.extend([[x+dx, y+dy, S.SURFACE_Z]
                    for dx in (-S.DROP_RADIUS, S.DROP_RADIUS)
                    for dy in (-S.DROP_RADIUS, S.DROP_RADIUS)])
        roi = roi_from_pixels(camera_project(cam,corners)*[w/cw,h/ch],w,h)
        # Keep the organizer's raw image orientation: the extrinsics already
        # map the physical view into robot-base coordinates. No image flip.
        draw_mapping(Image.fromarray(rgb), cam, folder / "camera_mapping.png")
        if self.sam is None:
            self.sam = SamClient(model_id=self.model_id)
        masks = self.sam.detect({"image": str(image_path), "output_dir": str(folder),
                                 "min_area_px": 20, "roi": roi})["detections"]
        def project(u, v, height):
            # Images are resized whole frames, NOT crops. Restore calibration pixels.
            return pixel_to_base(cam, (u * cw / w, v * ch / h), z=S.SURFACE_Z + height)[:2]
        heights = None
        if G.profile_name() != "legacy":
            vial_diameter = (S.DUAL_ZONE_V2_VIAL_DIAMETER_MM
                if G.profile_name() == "dual_zone_v2" else S.DUAL_ZONE_VIAL_DIAMETER_MM)
            heights = {"vial": vial_diameter/1000, "cap": G.CAP_HEIGHT_M}
        targets, rejected = build_candidates(masks, project,
            inspect_drop_areas=S.PHYSICAL_INSPECT_DROP_AREAS, height_by_kind=heights)
        # Correctly sorted objects are excluded from pickup targets, but still
        # occupy landing spots. Reuse the same masks: this is NOT another SAM call.
        self.observed_objects, _ = build_candidates(masks, project, exclude_drops=False,
            inspect_drop_areas=S.PHYSICAL_INSPECT_DROP_AREAS, height_by_kind=heights)
        eligible, disabled = self.motion_targets(targets)
        write_overlay(image_path, targets, folder / "targets.png")
        (folder / "targets.json").write_text(json.dumps({"targets": targets,
            "motion_eligible": eligible, "disabled_for_motion": disabled,
            "rejected": rejected}, indent=2))
        self.log("detections", count=len(targets), overlay=str(folder / "targets.png"),
                 order=[t["label"] for t in targets])
        return targets

    @staticmethod
    def motion_targets(targets):
        eligible, disabled = [], []
        for t in targets:
            reason = None
            if t["kind"] not in S.PHYSICAL_ALLOWED_KINDS:
                reason = "kind not enabled for this physical test"
            elif not np.isfinite(t.get("short_mm", float("nan"))):
                reason = "missing finite grasp width"
            elif G.profile_name() == "legacy" and (not 0 < t["short_mm"] or (
                t["short_mm"] > S.PHYSICAL_MAX_GRASP_WIDTH_MM
                and t["kind"] not in getattr(S, "PHYSICAL_VERIFIED_FIT_KINDS", ())
            )):
                reason = "grasp wider than the configured jaw clearance"
            if reason is None:
                try:
                    x,y = t["world_xy"]
                    spec = G.grasp_spec(t)
                    valid = False
                    failure = None
                    for r in Policy.approach_angles(t):
                        try:
                            Policy.validate(Policy.object_pose(t, (x,y),
                                S.SURFACE_Z + spec["pick_height_m"], r, 0, spec))
                            valid = True
                            break
                        except ValueError as exc:
                            failure = exc
                    if not valid:
                        raise ValueError(str(failure))
                except (ValueError, KeyError, TypeError) as e:
                    reason = f"invalid physical pickup target: {e}"
            if reason:
                disabled.append({"label": t["label"], "world_xy": t["world_xy"], "reason": reason})
            else:
                eligible.append(t)
        return eligible, disabled

    @staticmethod
    def approach_angles(target):
        first = (target["yaw_base_deg"] + S.TCP_R_FOR_AXIS_X + 90) % 180 - 90
        if G.profile_name() == "legacy":
            return [first]
        angles = [first, first+180]
        if target["kind"] == "cap":
            angles += [first+90, first-90]
        return [(angle+180)%360-180 for angle in angles]

    @staticmethod
    def object_pose(target, xy, z, r, grip, spec=None):
        """Object centre -> active contact -> existing controller TCP, once."""
        if G.profile_name() == "legacy":
            return base_to_tcp(*xy, z, r, grip)
        spec = G.grasp_spec(target) if spec is None else spec
        contact = G.object_to_contact_base(xy, r, spec)
        position = G.contact_to_tcp_base(contact, z, r, spec)
        return base_to_tcp(*position, r, grip)

    def obstacles(self):
        # Keep uncertain source locations until a fresh scan and all commanded
        # landings until reset. Neither a close nor an open command proves success.
        return [*self.observed_objects, *self.placed_envelopes]

    @staticmethod
    def planned_chunks(start, poses):
        planned, here = [], list(start)
        Policy.validate(here)
        for pose in poses:
            Policy.validate(pose)
            for chunk in interpolate(here, pose):
                for row in chunk:
                    Policy.validate(row)
                planned.append(chunk)
            here = list(pose)
        return planned

    def dual_check(self, start, poses, target=None, carrying=False):
        chunks = self.planned_chunks(start, poses)
        if G.profile_name() != "legacy":
            check_path([start, *(row for chunk in chunks for row in chunk)],
                       target, self.obstacles(), carrying=carrying)
        return chunks

    def drop_candidates(self, target):
        origin = np.asarray(S.DROP_XY[target["drop"]], float)
        for dx, dy in ((0,0),(-.04,0),(0,-.04),(.04,0),(0,.04),
                       (-.04,-.04),(-.04,.04),(.04,-.04),(.04,.04)):
            if math.hypot(dx, dy) <= S.DROP_RADIUS:
                yield (origin + [dx,dy]).tolist()

    def next_drop_xy(self, target):
        name = target["drop"]
        origin = np.asarray(S.DROP_XY[name], dtype=float)
        candidates = []
        offsets = ((0,0),(-.04,0),(0,-.04),(.04,0),(0,.04),
                   (-.04,-.04),(-.04,.04),(.04,-.04),(.04,.04))
        for dx, dy in offsets:
            if math.hypot(dx, dy) > S.DROP_RADIUS:
                continue
            xy = origin + [dx, dy]
            try:
                self.validate(base_to_tcp(
                    *xy, S.SURFACE_Z + S.DROP_HEIGHT, 0, 1))
            except ValueError:
                continue
            candidates.append(xy.tolist())
        occupied = [t["world_xy"] for t in self.observed_objects]
        occupied.extend(xy for spots in self.drop_reservations.values() for xy in spots)
        for xy in candidates:
            if all(np.linalg.norm(np.asarray(xy) - p) >= .035 for p in occupied):
                return xy
        raise ValueError(f"Destination {name} has no clear unused landing spots")

    @staticmethod
    def batch_mode():
        mode = S.PHYSICAL_SCAN_MODE
        if mode not in ("batch", "per_pick"):
            raise ValueError("PHYSICAL_SCAN_MODE must be 'batch' or 'per_pick'")
        return mode == "batch"

    def begin_pick(self, current, target):
        if G.profile_name() != "legacy":
            return self.begin_dual_pick(current, target)
        t = self.selected = dict(target)
        t["drop_xy"] = self.next_drop_xy(t)
        x, y = t["world_xy"]
        r = (t["yaw_base_deg"] + S.TCP_R_FOR_AXIS_X + 90) % 180 - 90
        z = S.SURFACE_Z + S.PICK_HEIGHT[t["kind"]]
        hover = base_to_tcp(x, y, S.SURFACE_Z + S.TRAVEL_HEIGHT, r, 0)
        pick = base_to_tcp(x, y, z, r, 0)
        close = [*pick[:6], 1.0]
        lift = [*hover[:6], 1.0]
        # Lift vertically before horizontal travel, including between cached picks.
        level = [*current[:2], hover[2], *current[3:6], current[6]]
        self.enqueue(current, [level, hover, pick, close, lift], "carry")
        self.attempts.add(t)
        self.picks += 1
        self.log("attempt", label=t["label"], xy=t["world_xy"], drop=t["drop"],
                 pickup_verified=False)

    def begin_dual_pick(self, current, target):
        spec = G.grasp_spec(target)
        # An offset rail makes 180-degree approaches physically different.
        failures = []
        for r in self.approach_angles(target):
            for drop_xy in self.drop_candidates(target):
                t = {**target, "grasp_spec": spec, "drop_xy": drop_xy, "grasp_r": r}
                pose = lambda xy,z,grip: self.object_pose(t, xy, S.SURFACE_Z+z, r, grip, spec)
                hover = pose(t["world_xy"], S.TRAVEL_HEIGHT, 0)
                pick = pose(t["world_xy"], spec["pick_height_m"], 0)
                close = [*pick[:6], 1.0]
                lift = [*hover[:6], 1.0]
                # Lift at the current orientation before approaching either rail.
                high = max(current[2], hover[2])
                level = [*current[:2], high, *current[3:6], current[6]]
                high_hover = [*hover[:2], high, *hover[3:]]
                pickup = [level, high_hover, hover, pick, close, lift]
                above = pose(drop_xy, S.TRAVEL_HEIGHT, 1)
                drop = pose(drop_xy, spec["release_height_m"], 1)
                opened, retreat = [*drop[:6], 0.0], [*above[:6], 0.0]
                delivery = [above, drop, opened, retreat]
                try:
                    # Screen the entire pickup AND delivery before closing once.
                    self.dual_check(current, [*pickup, *delivery], t, carrying=True)
                    if self.batch_active and not self.dual_regions_unchanged(t, (pick, drop)):
                        raise ClearanceError("RGB changed in a planned finger or object region; rescan")
                except (ValueError, KeyError, TypeError) as exc:
                    failures.append(str(exc))
                    continue
                self.selected = t
                self.enqueue(current, pickup, "carry")
                self.attempts.add(t)
                self.picks += 1
                self.log("attempt", label=t["label"], xy=t["world_xy"],
                         contact_xy=G.object_to_contact_base(t["world_xy"],r,spec).tolist(),
                         profile=G.profile_name(), zone=spec["zone"], r=r,
                         drop=t["drop"], pickup_verified=False)
                return
        raise ClearanceError("No valid pickup/delivery approach: " +
                             "; ".join(dict.fromkeys(failures))[:1000])

    def dual_regions_unchanged(self, target, poses):
        """Cheap fresh RGB check includes the inactive rail and landing footprint."""
        if self.batch_reference is None or self.current_rgb is None:
            return False
        if self.batch_reference.shape != self.current_rgb.shape:
            return False
        cam = self.cameras["table"]
        h,w = self.current_rgb.shape[:2]
        cw,ch = cam["size"]
        spec = target["grasp_spec"]
        for pose in poses:
            shapes = G.rail_rectangles(pose, margin_m=.004)
            centre = G.tcp_to_object_xy(pose, spec)
            shapes.append(G.object_rectangle(target, xy=centre,
                yaw_deg=pose[5]-S.TCP_R_FOR_AXIS_X, margin_m=.004))
            for shape in shapes:
                world = [[x,y,S.SURFACE_Z+z] for x,y in shape for z in (0,.045)]
                uv = camera_project(cam,world)*[w/cw,h/ch]
                if not np.all(np.isfinite(uv)):
                    return False
                lo = np.floor(uv.min(axis=0)).astype(int)-2
                hi = np.ceil(uv.max(axis=0)).astype(int)+3
                if np.any(lo < 0) or hi[0] > w or hi[1] > h:
                    return False
                x0,y0 = lo; x1,y1 = hi
                a = self.batch_reference[y0:y1,x0:x1].astype(np.int16)
                b = self.current_rgb[y0:y1,x0:x1].astype(np.int16)
                changes = np.count_nonzero(np.max(np.abs(a-b),axis=2)>30)
                if changes > max(3, .02*(x1-x0)*(y1-y0)):
                    return False
        return True

    def carry_to_drop(self, current, reason):
        t = self.selected
        xy = t.get("drop_xy", S.DROP_XY[t["drop"]])
        spec = (G.grasp_spec(t) if G.profile_name() != "legacy" else
                {"release_height_m": S.DROP_HEIGHT})
        above = self.object_pose(t, xy, S.SURFACE_Z + S.TRAVEL_HEIGHT, current[5], 1, spec)
        drop = self.object_pose(t, xy, S.SURFACE_Z + spec["release_height_m"], current[5], 1, spec)
        opened = [*drop[:6], 0.0]
        retreat = [*above[:6], 0.0]
        level = [*current[:2], above[2], *current[3:6], 1.0]
        self.enqueue(current, [level, above, drop, opened, retreat], "released")
        self.log("carry", pickup_verified=False, reason=reason)

    def enqueue(self, start, poses, next_state):
        # Build the whole plan before publishing any rows. A later invalid pose
        # must not leave an executable prefix queued.
        planned = self.dual_check(start, poses, self.selected,
                                 carrying=self.selected is not None)
        self.queue.extend(planned)
        self.state = next_state

    @staticmethod
    def validate(pose):
        if len(pose) != 7 or not np.all(np.isfinite(pose)):
            raise ValueError("Invalid TCP target")
        for value,(lo,hi) in zip(pose[:3],S.PHYSICAL_TCP_BOX_MM):
            if not lo <= value <= hi:
                raise ValueError("Target outside organizer TCP action box (UF0 mm)")
        if max(map(abs,pose[3:6])) > 360 or not 0 <= pose[6] <= 1:
            raise ValueError("Invalid TCP orientation or gripper command")
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

    def scan_poses(self, current, preserve_rotation=False):
        # Lift vertically before changing orientation or moving across the table.
        high = max(current[2], (S.SURFACE_Z + S.TRAVEL_HEIGHT) * 1000 + S.UF0_Z_MM)
        lifted = [*current[:2], high, *current[3:6], current[6]]
        r = current[5] if preserve_rotation else S.TCP_R_FOR_AXIS_X
        down = [*lifted[:3], -180.0, 0.0, r, current[6]]
        scan = base_to_tcp(*S.SCAN_XY, S.SURFACE_Z + S.SCAN_HEIGHT, r, current[6])
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
            G.profile_name()  # reject a misspelled hardware profile before planning
            if G.profile_name() != "legacy" and "table" in obs.get("images", {}):
                self.current_rgb = np.asarray(obs["images"]["table"], dtype=np.uint8)
            if not S.PHYSICAL_EXECUTE:
                if not self.preview_done:
                    self.detect(obs)
                    self.preview_done = True
                    self.log("preview_only", message="PHYSICAL_EXECUTE=False; holding current pose",
                             tcp_pose_uf0_mm=[round(float(v), 1) for v in obs["tcp_pose"]])
                return self.hold(obs)
            if not S.PHYSICAL_POSES_CONFIRMED:
                return self.halt(obs, "Set measured scan/drop poses and PHYSICAL_POSES_CONFIRMED first")
            if G.profile_name() != "legacy" and not all((S.DUAL_ZONE_MOUNT_CONFIRMED,
                    S.DUAL_ZONE_OBJECTS_CONFIRMED,S.DUAL_ZONE_LAYOUT_CONFIRMED)):
                return self.halt(obs, "Custom fingers need measured mounting/TCP, inventory and layout confirmations")
            action = self.motion(obs)
            if action is not None:
                return action
            current = self.hold(obs)
            if self.state == "start":
                if current[6] >= 0.5:
                    return self.halt(obs, "Start with an empty, open gripper")
                self.enqueue(current, self.scan_poses(current), "detect")
            elif self.state == "detect":
                batch = self.batch_mode()
                if not batch and S.MAX_PICKS > 0 and self.picks >= S.MAX_PICKS:
                    return self.halt(obs, "Attempt limit reached")
                detected = self.detect(obs)
                self.attempts.observe(detected)
                eligible, disabled = self.motion_targets(detected)
                if disabled:
                    self.log("disabled_targets", targets=disabled)
                choices = self.attempts.available(eligible)
                if batch:
                    # Always scan after the last batch, even at the attempt cap.
                    # Remaining detections are evidence for recovery, not a score.
                    self.log("batch_scan", batch=self.batch_number + 1,
                             attempts=self.picks, remaining_eligible=len(eligible),
                             retryable=len(choices), pickup_verified=False,
                             release_verified=False)
                    if S.MAX_PICKS > 0 and self.picks >= S.MAX_PICKS:
                        return self.halt(obs, "Attempt limit reached; final batch scan saved")
                    if self.batch_active and self.picks == self.batch_attempt_start:
                        self.batch_idle_passes += 1
                    else:
                        self.batch_idle_passes = 0
                    self.batch_active = False
                    if choices and self.batch_idle_passes >= S.PHYSICAL_BATCH_MAX_IDLE_PASSES:
                        return self.halt(obs, "Cached targets repeatedly changed; inspect latest overlay")
                if not choices:
                    self.batch_idle_passes = 0
                    if not S.PHYSICAL_WATCH:
                        return self.halt(obs, "No eligible detections; inspect overlay for missed/exhausted objects")
                    self.idle_until = time.monotonic() + S.PHYSICAL_IDLE_SECONDS
                    self.state = "idle"
                    self.log("idle", seconds=S.PHYSICAL_IDLE_SECONDS,
                             exhausted_visible=len(eligible))
                    return self.hold(obs)
                if batch:
                    if G.profile_name() == "legacy" and any(t["kind"] not in ("screw", "cube") for t in choices):
                        return self.halt(obs, "Batch mode supports cubes and screws; use per_pick for other kinds")
                    self.batch_number += 1
                    self.batch_active = True
                    self.batch_attempt_start = self.picks
                    self.batch_queue.refresh(choices)
                    self.batch_reference = np.asarray(obs["images"]["table"], dtype=np.uint8).copy()
                    self.state = "batch_next"
                else:
                    self.begin_pick(current, choices[0])
            elif self.state == "batch_next":
                # Only use cached geometry from the same clear, settled camera pose.
                # A changed target waits for the NEXT full scan, while unchanged
                # neighbours can still be attempted during this pass.
                choices = self.batch_queue.available(self.attempts)
                if S.MAX_PICKS > 0 and self.picks >= S.MAX_PICKS:
                    choices = []
                for t in choices:
                    if not target_unchanged(self.batch_reference,
                            np.asarray(obs["images"]["table"], dtype=np.uint8), t):
                        self.batch_queue.discard(t)
                        self.log("cached_target_deferred", label=t["label"], xy=t["world_xy"])
                        continue
                    try:
                        self.begin_pick(current, t)
                    except ClearanceError as exc:
                        # Try a different detected object before spending another
                        # SAM pass. No grasp attempt was consumed by this rejection.
                        self.batch_queue.discard(t)
                        self.log("cached_target_deferred", label=t["label"],
                                 xy=t["world_xy"], reason=str(exc))
                        continue
                    self.batch_queue.finish(t, False)  # result intentionally unknown until rescan
                    break
                else:
                    self.batch_queue.refresh([])
                    self.enqueue(current, self.scan_poses(current), "detect")
            elif self.state == "carry":
                # Clear the pickup view before checking the source. The gripper
                # directly above a missed object can hide it from the table camera.
                # Keep the grasp commanded closed throughout this scan move.
                if self.batch_mode():
                    self.carry_to_drop(current, "batch mode; pickup check deferred until batch rescan")
                else:
                    self.enqueue(current, self.scan_poses(current, preserve_rotation=True), "verify_source")
            elif self.state == "verify_source":
                # Source visibility can show a failed pick; disappearance alone is not proof.
                targets = self.detect(obs)
                t = self.selected
                still_there = any(a["kind"] == t["kind"] and
                    np.linalg.norm(np.array(a["world_xy"]) - t["world_xy"]) < 0.012 for a in targets)
                if still_there:
                    self.log("source_still_detected", label=t["label"])
                    # Return to the original pickup point before opening an uncertain grasp.
                    spec = (G.grasp_spec(t) if G.profile_name() != "legacy" else
                            {"pick_height_m": S.PICK_HEIGHT[t["kind"]]})
                    above = self.object_pose(t,t["world_xy"], S.SURFACE_Z + S.TRAVEL_HEIGHT, current[5], 1,spec)
                    pick = self.object_pose(t,t["world_xy"], S.SURFACE_Z + spec["pick_height_m"], current[5], 1,spec)
                    opened = [*pick[:6], 0.0]
                    up = [*above[:6],0.0]
                    # Stay at clearance while crossing from the scan point,
                    # then descend vertically over the original source.
                    level = [*current[:2], above[2], *current[3:6], 1.0]
                    self.enqueue(current, [level, above, pick, opened, up], "rescan")
                else:
                    self.carry_to_drop(current, "source no longer detected")
            elif self.state == "released":
                name = self.selected["drop"]
                self.drop_counts[name] = self.drop_counts.get(name, 0) + 1
                self.drop_reservations.setdefault(name, []).append(self.selected["drop_xy"])
                self.placed_envelopes.append({**self.selected,
                    "world_xy": list(self.selected["drop_xy"]),
                    "yaw_base_deg": self.selected.get("grasp_r",current[5])-S.TCP_R_FOR_AXIS_X})
                self.log("release_commanded", label=self.selected["label"], release_verified=False)
                if S.PAUSE_AFTER_LARGE_OBJECT and self.selected["kind"] in ("cap", "vial"):
                    return self.halt(obs, "Cap/vial attempt finished; inspect for hooked object before next rollout")
                self.enqueue(current, self.scan_poses(current),
                             "batch_next" if self.batch_mode() else "detect")
            elif self.state == "rescan":
                self.enqueue(current, self.scan_poses(current), "detect")
            elif self.state == "idle":
                if time.monotonic() < self.idle_until:
                    return self.hold(obs)
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
