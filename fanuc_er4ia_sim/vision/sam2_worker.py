#!/usr/bin/env python3
"""Persistent SAM 2 object-localization worker.

The worker loads SAM once, then accepts one JSON request per stdin line.  Each
request points at an RGB image written by Isaac Sim.  One JSON response is
written to stdout; human-readable logging goes to stderr so the protocol stays
machine-readable.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import traceback
from contextlib import nullcontext
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
from sam2.build_sam import build_sam2_hf


PALETTE = (
    (255, 99, 71),
    (60, 179, 113),
    (30, 144, 255),
    (255, 215, 0),
    (186, 85, 211),
    (0, 206, 209),
)


def log(message: str) -> None:
    print(f"[sam2] {message}", file=sys.stderr, flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-id", default="facebook/sam2.1-hiera-small")
    parser.add_argument("--points-per-side", type=int, default=32)
    parser.add_argument("--points-per-batch", type=int, default=32)
    parser.add_argument("--pred-iou-thresh", type=float, default=0.80)
    parser.add_argument("--stability-score-thresh", type=float, default=0.92)
    return parser.parse_args()


def parse_roi(value: Any, width: int, height: int) -> tuple[int, int, int, int]:
    if value is None:
        return 0, 0, width, height
    if isinstance(value, str):
        parts = [int(part.strip()) for part in value.split(",")]
    else:
        parts = [int(part) for part in value]
    if len(parts) != 4:
        raise ValueError("roi must contain x1,y1,x2,y2")
    x1, y1, x2, y2 = parts
    if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
        raise ValueError(f"ROI {parts} is outside the {width}x{height} image")
    return x1, y1, x2, y2


def mask_overlap(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    intersection = int(np.count_nonzero(a & b))
    if intersection == 0:
        return 0.0, 0.0
    area_a = int(np.count_nonzero(a))
    area_b = int(np.count_nonzero(b))
    union = area_a + area_b - intersection
    return intersection / max(union, 1), intersection / max(min(area_a, area_b), 1)


def filter_masks(
    raw_masks: list[dict[str, Any]],
    roi_shape: tuple[int, int],
    min_area_px: int,
    max_area_fraction: float,
    max_objects: int,
    border_margin_px: int,
) -> list[dict[str, Any]]:
    height, width = roi_shape
    roi_area = height * width
    candidates: list[dict[str, Any]] = []
    for item in raw_masks:
        area = int(item["area"])
        if not min_area_px <= area <= int(roi_area * max_area_fraction):
            continue
        x, y, w, h = (float(value) for value in item["bbox"])
        touches_border = (
            x <= border_margin_px
            or y <= border_margin_px
            or x + w >= width - border_margin_px
            or y + h >= height - border_margin_px
        )
        if touches_border:
            continue
        candidates.append(item)

    # Prefer complete objects over smaller masks nested inside those objects.
    candidates.sort(
        key=lambda item: (
            int(item["area"]),
            float(item["predicted_iou"]) * float(item["stability_score"]),
        ),
        reverse=True,
    )
    kept: list[dict[str, Any]] = []
    for candidate in candidates:
        mask = np.asarray(candidate["segmentation"], dtype=bool)
        duplicate = False
        for accepted in kept:
            accepted_mask = np.asarray(accepted["segmentation"], dtype=bool)
            iou, containment = mask_overlap(mask, accepted_mask)
            if iou >= 0.70 or containment >= 0.88:
                duplicate = True
                break
        if not duplicate:
            kept.append(candidate)
        if len(kept) >= max_objects:
            break
    return kept


def mask_features(mask: np.ndarray, rgb_roi: np.ndarray) -> dict[str, Any]:
    ys, xs = np.nonzero(mask)
    if len(xs) == 0:
        raise ValueError("Cannot calculate features for an empty mask")

    center = np.array([xs.mean(), ys.mean()], dtype=np.float64)
    coords = np.column_stack((xs, ys)).astype(np.float64)
    centered = coords - center
    covariance = centered.T @ centered / max(len(coords) - 1, 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    major_axis = eigenvectors[:, int(np.argmax(eigenvalues))]
    angle_deg = float(math.degrees(math.atan2(major_axis[1], major_axis[0])) % 180.0)

    mask_u8 = mask.astype(np.uint8)
    eroded = cv2.erode(mask_u8, np.ones((5, 5), dtype=np.uint8), iterations=1).astype(bool)
    color_mask = eroded if np.any(eroded) else mask
    median_rgb = np.median(rgb_roi[color_mask], axis=0)

    return {
        "center_roi_px": center.tolist(),
        "grasp_yaw_image_deg": angle_deg,
        "median_rgb": [int(round(value)) for value in median_rgb],
    }


def annotate(
    bgr_image: np.ndarray,
    detections: list[dict[str, Any]],
    full_masks: list[np.ndarray],
    roi: tuple[int, int, int, int],
) -> np.ndarray:
    overlay = bgr_image.copy()
    x1, y1, x2, y2 = roi
    cv2.rectangle(overlay, (x1, y1), (x2 - 1, y2 - 1), (255, 255, 255), 2)
    for detection, mask in zip(detections, full_masks):
        rgb_color = PALETTE[(detection["id"] - 1) % len(PALETTE)]
        color = (rgb_color[2], rgb_color[1], rgb_color[0])
        tint = np.zeros_like(overlay)
        tint[mask] = color
        overlay = cv2.addWeighted(overlay, 1.0, tint, 0.35, 0.0)
        contours, _ = cv2.findContours(
            mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(overlay, contours, -1, color, 2)
        u, v = detection["center_full_px"]
        center = (int(round(u)), int(round(v)))
        angle = math.radians(detection["grasp_yaw_image_deg"])
        endpoint = (
            int(round(center[0] + 35 * math.cos(angle))),
            int(round(center[1] + 35 * math.sin(angle))),
        )
        cv2.circle(overlay, center, 5, (255, 255, 255), -1)
        cv2.arrowedLine(overlay, center, endpoint, color, 3, tipLength=0.25)
        cv2.putText(
            overlay,
            f"obj {detection['id']} ({center[0]},{center[1]})",
            (center[0] + 8, center[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
    return overlay


class SamWorker:
    def __init__(self, args: argparse.Namespace) -> None:
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        log(f"loading {args.model_id} on {self.device}")
        model = build_sam2_hf(args.model_id, device=self.device)
        self.generator = SAM2AutomaticMaskGenerator(
            model=model,
            points_per_side=args.points_per_side,
            points_per_batch=args.points_per_batch,
            pred_iou_thresh=args.pred_iou_thresh,
            stability_score_thresh=args.stability_score_thresh,
            box_nms_thresh=0.70,
            crop_n_layers=0,
            min_mask_region_area=0,
            output_mode="binary_mask",
        )
        self.model_id = args.model_id

    def process(self, request: dict[str, Any]) -> dict[str, Any]:
        image_path = Path(request["image"]).expanduser().resolve()
        output_dir = Path(request["output_dir"]).expanduser().resolve()
        if not image_path.is_file():
            raise FileNotFoundError(image_path)
        bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError(f"OpenCV could not decode {image_path}")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        height, width = rgb.shape[:2]
        roi = parse_roi(request.get("roi"), width, height)
        x1, y1, x2, y2 = roi
        rgb_roi = np.ascontiguousarray(rgb[y1:y2, x1:x2])

        autocast_context = (
            torch.autocast(device_type="cuda", dtype=torch.float16)
            if self.device == "cuda"
            else nullcontext()
        )
        with torch.inference_mode(), autocast_context:
            raw_masks = self.generator.generate(rgb_roi)

        masks = filter_masks(
            raw_masks,
            rgb_roi.shape[:2],
            int(request.get("min_area_px", 250)),
            float(request.get("max_area_fraction", 0.20)),
            int(request.get("max_objects", 30)),
            int(request.get("border_margin_px", 3)),
        )

        detections: list[dict[str, Any]] = []
        full_masks: list[np.ndarray] = []
        for index, item in enumerate(masks, start=1):
            mask_roi = np.asarray(item["segmentation"], dtype=bool)
            features = mask_features(mask_roi, rgb_roi)
            u_roi, v_roi = features.pop("center_roi_px")
            u_full, v_full = u_roi + x1, v_roi + y1
            full_mask = np.zeros((height, width), dtype=bool)
            full_mask[y1:y2, x1:x2] = mask_roi
            full_masks.append(full_mask)
            detections.append(
                {
                    "id": index,
                    "center_full_px": [u_full, v_full],
                    "bbox_roi_xywh": [float(value) for value in item["bbox"]],
                    "area_px": int(item["area"]),
                    "predicted_iou": float(item["predicted_iou"]),
                    "stability_score": float(item["stability_score"]),
                    **features,
                }
            )

        output_dir.mkdir(parents=True, exist_ok=True)
        overlay_path = output_dir / "annotated.png"
        json_path = output_dir / "detections.json"
        response = {
            "ok": True,
            "source_image": str(image_path),
            "image_size_wh": [width, height],
            "roi_xyxy": list(roi),
            "model_id": self.model_id,
            "raw_mask_count": len(raw_masks),
            "detections": detections,
            "overlay": str(overlay_path),
        }
        cv2.imwrite(str(overlay_path), annotate(bgr, detections, full_masks, roi))
        json_path.write_text(json.dumps(response, indent=2) + "\n", encoding="utf-8")
        return response


def emit(payload: dict[str, Any]) -> None:
    print(json.dumps(payload, separators=(",", ":")), flush=True)


def main() -> None:
    args = parse_args()
    worker = SamWorker(args)
    emit({"ok": True, "status": "ready", "model_id": args.model_id, "device": worker.device})
    for line in sys.stdin:
        if not line.strip():
            continue
        try:
            request = json.loads(line)
            if request.get("command") == "shutdown":
                emit({"ok": True, "status": "shutdown"})
                return
            response = worker.process(request)
            log(
                f"{Path(response['source_image']).name}: "
                f"raw={response['raw_mask_count']} kept={len(response['detections'])}"
            )
            emit(response)
        except Exception as exc:
            traceback.print_exc(file=sys.stderr)
            emit({"ok": False, "error": str(exc)})


if __name__ == "__main__":
    main()
