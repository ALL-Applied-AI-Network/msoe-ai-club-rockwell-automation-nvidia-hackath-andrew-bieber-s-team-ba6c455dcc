"""SAM masks -> pixel colours / metric shapes -> sorted pickup candidates.

SAM segments; it does NOT supply semantic labels. The simple size/shape rules
below are a first baseline for this cell, to tune on saved real-camera overlays.
No USD names or simulator object coordinates are used for classification.
"""
from __future__ import annotations

import argparse
import contextlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from typing import Callable

import numpy as np
import sort_config as S


def roi_from_pixels(uv, width, height, margin=96):
    points = np.asarray(uv, float)
    lo = np.maximum(np.floor(points.min(axis=0))-margin, [0,0]).astype(int)
    hi = np.minimum(np.ceil(points.max(axis=0))+margin, [width,height]).astype(int)
    if np.any(hi <= lo):
        raise ValueError("Pickup workspace is outside the camera image")
    return ",".join(map(str, [*lo,*hi]))


def object_kind(label):
    return "cube" if label.startswith("cube_") else label


def classify(long_mm, short_mm, color):
    """Broad physical-size ranges; unknown masks are rejected, never guessed."""
    aspect = long_mm / max(short_mm, 0.001)
    if 7 <= long_mm <= 25 and 6 <= short_mm <= 22 and aspect <= 1.8:
        if color in ("red", "blue", "yellow"):
            return "cube_" + color
    if 20 <= long_mm <= 45 and 3 <= short_mm <= 15 and aspect >= 1.8:
        return "screw"
    if 24 <= long_mm <= 43 and 20 <= short_mm <= 39 and aspect <= 1.65:
        return "cap"
    if 75 <= long_mm <= 245 and 10 <= short_mm <= 37 and aspect >= 3.0:
        return "vial"
    return None


def metric_shape(points):
    p = np.asarray(points, float)[:, :2]
    # Raster contours contain tiny stair steps. Use convex-hull edges so a square's
    # fitted angle follows its long straight sides rather than those individual pixels.
    ordered = sorted(set(map(tuple, p)))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    def half(seq):
        result = []
        for q in seq:
            while len(result) >= 2 and cross(result[-2],result[-1],q) <= 0:
                result.pop()
            result.append(q)
        return result
    hull = np.array(half(ordered)[:-1] + half(ordered[::-1])[:-1])
    if len(hull) < 3:
        raise ValueError("degenerate contour")
    edges = np.roll(hull, -1, axis=0) - hull
    edges = edges[np.linalg.norm(edges, axis=1) > 1e-9]
    if len(edges) < 3:
        raise ValueError("degenerate contour")
    # Minimum-area rectangle, rather than PCA: PCA gives unstable diagonals on squares.
    choices = []
    for angle in np.arctan2(edges[:, 1], edges[:, 0]):
        axis = np.array([np.cos(angle), np.sin(angle)])
        cross = np.array([-axis[1], axis[0]])
        a, b = np.ptp(p @ axis), np.ptp(p @ cross)
        choices.append((a * b, a, b, axis, cross))
    _, long, short, axis, cross = min(choices, key=lambda x: x[0])
    if short > long:
        long, short, axis = short, long, cross
    return long * 1000, short * 1000, axis


def build_candidates(detections, projector: Callable, workspace=S.WORKSPACE,
                     drops=None, exclude_drops=True, allow_misc=False, height_at=None,
                     inspect_drop_areas=False):
    """projector(u,v,height_m) -> base XY in metres; height is above the surface."""
    drops = S.DROP_XY if drops is None else drops
    found, rejected = [], []
    xmin, xmax, ymin, ymax = workspace
    for d in detections:
        try:
            contour = d["contour_full_px"]
            points = np.array([projector(u, v, 0.0) for u, v in contour])
            long, short, axis = metric_shape(points)
            label = classify(long, short, d.get("color", "unknown"))
            misc_height = None
            if label is None:
                if not allow_misc or height_at is None:
                    raise ValueError(f"unclassified shape {long:.1f}x{short:.1f} mm")
                # Unknown does not mean graspable: require a small, compact mask
                # and positive RGB-D height evidence, so mat graphics stay excluded.
                if not (5 <= long <= S.MISC_MAX_LENGTH_MM and
                        4 <= short <= S.MISC_MAX_WIDTH_MM and long / short <= 5 and
                        d.get("predicted_iou", 0) >= .85 and
                        d.get("stability_score", 0) >= .94 and d.get("solidity", 1) >= .65):
                    raise ValueError("unknown object outside misc grasp limits")
                misc_height = float(height_at(*d["center_full_px"]))
                if not np.isfinite(misc_height) or not .004 <= misc_height <= .025:
                    raise ValueError("unknown object lacks a 4-25 mm measured height")
                label = "misc"
            kind = object_kind(label)
            # Correct perspective using an approximate visible top-surface height.
            height = misc_height if kind == "misc" else {
                "screw": 0.006, "cube": 0.010 if long < 14 else 0.016,
                "cap": 0.0157, "vial": min(short / 1000, 0.02718)}[kind]
            points = np.array([projector(u, v, height) for u, v in contour])
            long, short, axis = metric_shape(points)
            u, v = map(float, d["center_full_px"])
            # An image centroid is biased by perspective on long vials. Compute
            # the grasp centre after projection into the table's metric plane.
            cross_axis = np.array([-axis[1], axis[0]])
            along, across = points @ axis, points @ cross_axis
            xy = axis*(along.min()+along.max())/2 + cross_axis*(across.min()+across.max())/2
            if not np.all(np.isfinite(xy)):
                raise ValueError("invalid projection")
            # For elongated screws the mask centroid is pulled toward the head.
            # Use the middle of the fitted long-axis span to land on the shaft.
            if kind == "screw":
                t = points @ axis
                xy += axis * ((t.min() + t.max()) / 2 - xy @ axis)
            nearby_drops = [name for name, p in drops.items()
                            if np.linalg.norm(xy-p) < S.DROP_RADIUS]
            inside_workspace = xmin <= xy[0] <= xmax and ymin <= xy[1] <= ymax
            if not inside_workspace and not (inspect_drop_areas and nearby_drops):
                raise ValueError("outside pickup area and inspected destinations")
            if np.linalg.norm(xy) > S.MAX_RADIUS:
                raise ValueError("outside radial reach limit")
            destination = S.BIN_FOR[label]
            if inspect_drop_areas and destination not in drops:
                raise ValueError(f"unconfigured destination: {destination}")
            if exclude_drops and nearby_drops:
                if not inspect_drop_areas:
                    raise ValueError("inside drop area")
                if destination in nearby_drops:
                    raise ValueError(f"already in correct destination: {destination}")
            yaw = math.degrees(math.atan2(axis[1], axis[0])) % 180.0
            found.append({**d, "label": label, "kind": kind, "world_xy": xy.tolist(),
                          "yaw_base_deg": yaw, "long_mm": float(long), "short_mm": float(short),
                          "drop": destination,
                          "source_zone": nearby_drops[0] if nearby_drops else "pickup",
                          "height_m": height,
                          "bounds_xy": [points.min(axis=0).tolist(), points.max(axis=0).tolist()],
                          "spread_dir": [-float(axis[1]), float(axis[0]), 0.0]})
        except (ValueError, KeyError, np.linalg.LinAlgError) as e:
            rejected.append({"id": d.get("id"), "reason": str(e)})
    # Remove overlapping masks for the same object; keep the better SAM score.
    unique = []
    known = [a for a in found if a["kind"] != "misc"]
    for d in sorted(found, key=lambda a: (a["kind"] == "misc",
                                        -a["predicted_iou"] * a["stability_score"])):
        if d["kind"] == "misc" and any(
            np.all(np.array(d["world_xy"]) >= np.array(a["bounds_xy"][0])-.006) and
            np.all(np.array(d["world_xy"]) <= np.array(a["bounds_xy"][1])+.006)
            for a in known):
            rejected.append({"id": d["id"], "reason": "misc mask overlaps a known object"})
            continue
        if any(np.linalg.norm(np.array(d["world_xy"]) - a["world_xy"]) < 0.008 for a in unique):
            continue
        unique.append(d)
    unique.sort(key=lambda d: (S.PICK_ORDER[d["kind"]], d["long_mm"] * d["short_mm"]))
    return unique, rejected


class Attempts:
    """Remember locations, not SAM mask IDs (IDs change on every image)."""
    def __init__(self):
        self.items = []
        self.missing_scans = {}

    def count(self, target):
        return sum(1 for kind, xy in self.items if kind == target["kind"] and
                   np.linalg.norm(np.asarray(target["world_xy"]) - xy) < S.ATTEMPT_RADIUS)

    def available(self, targets):
        return [t for t in targets if self.count(t) < S.MAX_ATTEMPTS]

    def add(self, target):
        self.items.append((target["kind"], np.array(target["world_xy"], float)))

    def observe(self, targets):
        """Retire old locations after two full scans without that object.

        Call only on fresh SAM results, before any target-count/type filtering.
        A single missed detection must not reset an exhausted retry budget.
        """
        missing = {}
        for kind, xy in self.items:
            key = (kind, *xy)
            visible = any(t["kind"] == kind and
                np.linalg.norm(np.asarray(t["world_xy"]) - xy) < S.ATTEMPT_RADIUS
                for t in targets)
            missing[key] = 0 if visible else self.missing_scans.get(key, 0) + 1
        self.items = [(kind, xy) for kind, xy in self.items if missing[(kind, *xy)] < 2]
        self.missing_scans = {key: n for key, n in missing.items() if n < 2}


class ScanQueue:
    """Reuse a few targets, or finish one entire scan before the next SAM pass."""
    def __init__(self, limit=1, whole_batch=False):
        self.limit = limit
        self.whole_batch = whole_batch
        self.targets = []
        self.used = 0

    def refresh(self, targets):
        self.targets = list(targets)
        self.used = 0

    def available(self, attempts):
        return attempts.available(self.targets) if self.whole_batch or self.used < self.limit else []

    def discard(self, chosen):
        """Consume one queued target; allow it again only after a fresh scan."""
        self.targets = [t for t in self.targets if t is not chosen]

    def finish(self, chosen, success):
        self.used += 1
        if self.whole_batch:
            # A normal failed pick is retried on the next pass, not immediately.
            # Fresh RGB checks defer any later targets displaced by this attempt.
            self.discard(chosen)
            return
        if not success or chosen["kind"] in ("cap", "vial"):
            self.targets = []
            return
        # Nearby objects may have been nudged by the fingers. Detect them again.
        self.targets = [t for t in self.targets if
            np.linalg.norm(np.array(t["world_xy"]) - chosen["world_xy"]) > .060]


def target_unchanged(reference, current, target):
    """Cheap camera check before using a cached target; changed pixels force SAM."""
    if reference.shape != current.shape:
        return False
    p = np.asarray(target["contour_full_px"])
    h, w = current.shape[:2]
    x0, y0 = np.maximum(np.floor(p.min(axis=0)).astype(int)-5, [0, 0])
    x1, y1 = np.minimum(np.ceil(p.max(axis=0)).astype(int)+6, [w, h])
    if x1 <= x0 or y1 <= y0:
        return False
    a = reference[y0:y1, x0:x1].astype(np.int16)
    b = current[y0:y1, x0:x1].astype(np.int16)
    changed = np.count_nonzero(np.max(np.abs(a-b), axis=2) > 30)
    return changed <= max(3, .20 * target["area_px"])


class SamClient:
    """Same persistent-process pattern as the uploaded runner; keeps torch out of Isaac."""
    def __init__(self, python=S.SAM_PYTHON, worker=None, model_id=S.MODEL_ID,
                 points_per_side=64, points_per_batch=4, precision="fp32"):
        worker = Path(__file__) if worker is None else Path(worker)
        self.process = subprocess.Popen([str(Path(python).expanduser()), "-u", str(worker),
            "--worker", "--model-id", model_id, "--points-per-side", str(points_per_side),
            "--points-per-batch", str(points_per_batch), "--precision", precision], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, text=True, bufsize=1)
        ready = self._read()
        if ready.get("status") != "ready":
            raise RuntimeError(str(ready))
        print(f"[sam] device={ready.get('device')} precision={ready.get('precision')} "
              f"batch={points_per_batch}", file=sys.stderr, flush=True)

    def _read(self):
        for line in self.process.stdout:
            try:
                result = json.loads(line)
            except json.JSONDecodeError:
                print(line.rstrip(), file=sys.stderr)
                continue
            if not result.get("ok"):
                raise RuntimeError(result.get("error", "SAM worker failed"))
            return result
        raise RuntimeError(f"SAM worker exited ({self.process.poll()})")

    def detect(self, request):
        self.process.stdin.write(json.dumps(request) + "\n")
        self.process.stdin.flush()
        return self._read()

    def close(self):
        if self.process.poll() is None:
            try:
                self.process.stdin.write('{"command":"shutdown"}\n')
                self.process.stdin.flush()
                self.process.wait(timeout=5)
            except (OSError, subprocess.TimeoutExpired):
                self.process.terminate()
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()


def describe_masks(rgb, masks, min_area=20, max_fraction=0.08, border=3):
    """Summarize masks in the worker; host only needs numpy and these small records."""
    import cv2
    h, w = rgb.shape[:2]
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    output = []
    for mask in masks:
        binary = np.ascontiguousarray(mask["segmentation"], dtype=np.uint8)
        area = int(binary.sum())
        if not min_area <= area <= h * w * max_fraction:
            continue
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea).reshape(-1, 2)
        if len(contour) < 5:
            continue
        if (contour[:, 0].min() < border or contour[:, 1].min() < border or
            contour[:, 0].max() >= w-border or contour[:, 1].max() >= h-border):
            continue
        # Ignore disconnected fragments. Erode for color only, preserving small masks.
        filled = np.zeros(binary.shape, dtype=np.uint8)
        cv2.drawContours(filled, [contour], -1, 1, -1)
        inside = cv2.erode(filled, np.ones((3, 3), np.uint8))
        if inside.sum() < 5:
            inside = filled
        pixels = hsv[inside.astype(bool)]
        hue, sat, value = pixels.T
        chroma = (sat >= 65) & (value >= 40)
        votes = {"red": np.mean(chroma & ((hue < 12) | (hue > 168))),
                 "yellow": np.mean(chroma & (hue >= 17) & (hue <= 40)),
                 "blue": np.mean(chroma & (hue >= 90) & (hue <= 135))}
        color = max(votes, key=votes.get)
        if votes[color] < 0.25:
            color = "unknown"
        ys, xs = np.where(filled)
        centre = [float(xs.mean()), float(ys.mean())]
        sampled = contour[np.linspace(0, len(contour)-1, min(80, len(contour))).astype(int)]
        rect = cv2.minAreaRect(contour.astype(np.float32))
        yaw = rect[2] + (90 if rect[1][1] > rect[1][0] else 0)
        output.append({"id": len(output), "center_full_px": centre,
                       "contour_full_px": sampled.tolist(), "area_px": area, "color": color,
                       "grasp_yaw_image_deg": float(yaw),
                       "solidity": float(cv2.contourArea(contour) /
                           max(cv2.contourArea(cv2.convexHull(contour)), 1.0)),
                       "predicted_iou": float(mask.get("predicted_iou", 1.0)),
                       "stability_score": float(mask.get("stability_score", 1.0))})
    return output


def write_overlay(image_path, targets, output_path):
    from PIL import Image, ImageDraw
    im = Image.open(image_path).convert("RGB")
    draw = ImageDraw.Draw(im)
    for n, d in enumerate(targets):
        u, v = d["center_full_px"]
        draw.line([tuple(p) for p in d["contour_full_px"]] + [tuple(d["contour_full_px"][0])],
                  fill=(0, 255, 80), width=2)
        draw.ellipse((u-4, v-4, u+4, v+4), fill=(255, 50, 50))
        text = f'{n+1}: {d.get("label", d.get("color"))}'
        if "drop" in d:
            text += f' -> {d["drop"]}'
        draw.text((u+6, v+5), text, fill=(255, 0, 255), stroke_width=1, stroke_fill="black")
    im.save(output_path)


def worker_main(args):
    import torch
    import cv2
    from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
    device = "cuda" if torch.cuda.is_available() else "cpu"
    use_amp = device == "cuda" and args.precision == "auto"
    amp_dtype = (torch.bfloat16 if device == "cuda" and torch.cuda.is_bf16_supported()
                 else torch.float16)
    with contextlib.redirect_stdout(sys.stderr):
        generator = SAM2AutomaticMaskGenerator.from_pretrained(args.model_id, device=device,
            points_per_side=args.points_per_side, points_per_batch=args.points_per_batch,
            pred_iou_thresh=0.75, stability_score_thresh=0.90, crop_n_layers=1,
            crop_n_points_downscale_factor=2, min_mask_region_area=0)
    print(json.dumps({"ok": True, "status": "ready", "model_id": args.model_id, "device": device,
                      "precision": str(amp_dtype) if use_amp else "float32"}), flush=True)
    for line in sys.stdin:
        try:
            request = json.loads(line)
            if request.get("command") == "shutdown":
                break
            bgr = cv2.imread(request["image"])
            if bgr is None:
                raise ValueError("Could not read input image")
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            x1, y1, x2, y2 = 0, 0, rgb.shape[1], rgb.shape[0]
            if request.get("roi"):
                x1, y1, x2, y2 = map(int, request["roi"].split(","))
                if not (0 <= x1 < x2 <= rgb.shape[1] and 0 <= y1 < y2 <= rgb.shape[0]):
                    raise ValueError("ROI is outside the image")
            crop = rgb[y1:y2, x1:x2]
            started = time.monotonic()
            amp = torch.autocast("cuda", dtype=amp_dtype) if use_amp else contextlib.nullcontext()
            with torch.inference_mode(), amp, contextlib.redirect_stdout(sys.stderr):
                masks = generator.generate(crop)
            detections = describe_masks(crop, masks, request.get("min_area_px", 20),
                request.get("max_area_fraction", 0.08), request.get("border_margin_px", 3))
            for d in detections:
                d["center_full_px"] = (np.array(d["center_full_px"]) + [x1, y1]).tolist()
                d["contour_full_px"] = (np.array(d["contour_full_px"]) + [x1, y1]).tolist()
            output = Path(request["output_dir"])
            output.mkdir(parents=True, exist_ok=True)
            overlay = output / "masks.png"
            write_overlay(request["image"], detections, overlay)
            elapsed = time.monotonic() - started
            mask_count = len(masks)
            del masks
            # Release cached allocations while Isaac runs the motion phases.
            if device == "cuda":
                generator.predictor.reset_predictor()
                torch.cuda.empty_cache()
            print(json.dumps({"ok": True, "detections": detections, "raw_mask_count": mask_count,
                              "seconds": round(elapsed, 3), "overlay": str(overlay)}), flush=True)
        except Exception as e:
            print(json.dumps({"ok": False, "error": f"{type(e).__name__}: {e}"}), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", action="store_true")
    parser.add_argument("--model-id", default=S.MODEL_ID)
    parser.add_argument("--points-per-side", type=int, default=64)
    parser.add_argument("--points-per-batch", type=int, default=4)
    parser.add_argument("--precision", choices=("auto", "fp32"), default="fp32")
    worker_main(parser.parse_args())
