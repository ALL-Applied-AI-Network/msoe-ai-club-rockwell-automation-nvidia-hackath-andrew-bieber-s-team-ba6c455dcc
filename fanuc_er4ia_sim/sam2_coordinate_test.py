#!/usr/bin/env python3
"""Zero-training object localization with SAM 2.1.

This is deliberately an offline first test:
    RGB image -> object masks -> pixel centroids/orientations -> JSON + overlay

It does not command the robot. Once the pixel detections are correct, provide a
top-camera pixel-to-table homography with --homography to also emit world XY.
"""

from __future__ import annotations

import argparse
import json
import math
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate SAM 2 masks and candidate grasp pixels from one RGB image."
    )
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("runs/sam2_first_test"))
    parser.add_argument(
        "--model-id",
        default="facebook/sam2.1-hiera-small",
        help="Hugging Face SAM 2 model ID.",
    )
    parser.add_argument(
        "--roi",
        default=None,
        help="Optional workspace crop as x1,y1,x2,y2 in full-image pixels.",
    )
    parser.add_argument("--points-per-side", type=int, default=32)
    parser.add_argument("--points-per-batch", type=int, default=32)
    parser.add_argument("--min-area-px", type=int, default=250)
    parser.add_argument(
        "--max-area-fraction",
        type=float,
        default=0.20,
        help="Reject masks larger than this fraction of the ROI.",
    )
    parser.add_argument("--max-objects", type=int, default=30)
    parser.add_argument(
        "--homography",
        type=Path,
        default=None,
        help="Optional JSON file containing a 3x3 pixel-to-world-plane matrix.",
    )
    return parser.parse_args()


def parse_roi(value: str | None, width: int, height: int) -> tuple[int, int, int, int]:
    if value is None:
        return 0, 0, width, height

    parts = [int(part.strip()) for part in value.split(",")]
    if len(parts) != 4:
        raise ValueError("--roi must be x1,y1,x2,y2")

    x1, y1, x2, y2 = parts
    if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
        raise ValueError(f"ROI {parts} is outside the {width}x{height} image")
    return x1, y1, x2, y2


def load_homography(path: Path | None) -> np.ndarray | None:
    if path is None:
        return None
    payload = json.loads(path.read_text(encoding="utf-8"))
    raw = payload.get("pixel_to_world_homography", payload)
    matrix = np.asarray(raw, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError("Homography JSON must contain a 3x3 matrix")
    return matrix


def pixel_to_world_xy(u: float, v: float, homography: np.ndarray) -> list[float]:
    projected = homography @ np.array([u, v, 1.0], dtype=np.float64)
    if abs(projected[2]) < 1e-12:
        raise ValueError("Homography projected a point to infinity")
    return [float(projected[0] / projected[2]), float(projected[1] / projected[2])]


def mask_overlap(a: np.ndarray, b: np.ndarray) -> tuple[float, float]:
    intersection = int(np.count_nonzero(a & b))
    if intersection == 0:
        return 0.0, 0.0
    area_a = int(np.count_nonzero(a))
    area_b = int(np.count_nonzero(b))
    union = area_a + area_b - intersection
    iou = intersection / max(union, 1)
    containment = intersection / max(min(area_a, area_b), 1)
    return iou, containment


def filter_masks(
    raw_masks: list[dict[str, Any]],
    roi_shape: tuple[int, int],
    min_area_px: int,
    max_area_fraction: float,
    max_objects: int,
) -> list[dict[str, Any]]:
    roi_area = roi_shape[0] * roi_shape[1]
    candidates = [
        item
        for item in raw_masks
        if min_area_px <= int(item["area"]) <= int(roi_area * max_area_fraction)
    ]

    # Prefer complete object masks over nested masks of handles, labels, edges, etc.
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

    u = float(xs.mean())
    v = float(ys.mean())

    coords = np.column_stack((xs, ys)).astype(np.float64)
    centered = coords - coords.mean(axis=0, keepdims=True)
    covariance = centered.T @ centered / max(len(coords) - 1, 1)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    major_axis = eigenvectors[:, int(np.argmax(eigenvalues))]
    angle_deg = float(math.degrees(math.atan2(major_axis[1], major_axis[0])) % 180.0)

    mask_u8 = mask.astype(np.uint8)
    eroded = cv2.erode(mask_u8, np.ones((5, 5), dtype=np.uint8), iterations=1).astype(bool)
    color_mask = eroded if np.any(eroded) else mask
    median_rgb = np.median(rgb_roi[color_mask], axis=0)

    return {
        "center_roi_px": [u, v],
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
        color = PALETTE[(detection["id"] - 1) % len(PALETTE)]
        color_bgr = (color[2], color[1], color[0])
        tint = np.zeros_like(overlay)
        tint[mask] = color_bgr
        overlay = cv2.addWeighted(overlay, 1.0, tint, 0.35, 0.0)

        contours, _ = cv2.findContours(
            mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(overlay, contours, -1, color_bgr, 2)

        u, v = detection["center_full_px"]
        center = (int(round(u)), int(round(v)))
        angle = math.radians(detection["grasp_yaw_image_deg"])
        endpoint = (
            int(round(center[0] + 35 * math.cos(angle))),
            int(round(center[1] + 35 * math.sin(angle))),
        )
        cv2.circle(overlay, center, 5, (255, 255, 255), -1)
        cv2.arrowedLine(overlay, center, endpoint, color_bgr, 3, tipLength=0.25)
        cv2.putText(
            overlay,
            f"obj {detection['id']} ({center[0]},{center[1]})",
            (center[0] + 8, center[1] - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color_bgr,
            2,
            cv2.LINE_AA,
        )
    return overlay


def main() -> None:
    args = parse_args()
    if not args.image.is_file():
        raise FileNotFoundError(args.image)

    bgr = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if bgr is None:
        raise ValueError(f"OpenCV could not decode {args.image}")
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    height, width = rgb.shape[:2]
    roi = parse_roi(args.roi, width, height)
    x1, y1, x2, y2 = roi
    rgb_roi = np.ascontiguousarray(rgb[y1:y2, x1:x2])

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[vision] device={device} model={args.model_id}")
    print(f"[vision] image={args.image} size={width}x{height} roi={roi}")

    model = build_sam2_hf(args.model_id, device=device)
    generator = SAM2AutomaticMaskGenerator(
        model=model,
        points_per_side=args.points_per_side,
        points_per_batch=args.points_per_batch,
        pred_iou_thresh=0.80,
        stability_score_thresh=0.92,
        box_nms_thresh=0.70,
        crop_n_layers=0,
        min_mask_region_area=0,
        output_mode="binary_mask",
    )

    autocast_context = (
        torch.autocast(device_type="cuda", dtype=torch.float16)
        if device == "cuda"
        else nullcontext()
    )
    with torch.inference_mode(), autocast_context:
        raw_masks = generator.generate(rgb_roi)

    masks = filter_masks(
        raw_masks=raw_masks,
        roi_shape=rgb_roi.shape[:2],
        min_area_px=args.min_area_px,
        max_area_fraction=args.max_area_fraction,
        max_objects=args.max_objects,
    )
    homography = load_homography(args.homography)

    detections: list[dict[str, Any]] = []
    full_masks: list[np.ndarray] = []
    roi_center = np.array([(x1 + x2) / 2.0, (y1 + y2) / 2.0])

    for index, item in enumerate(masks, start=1):
        mask_roi = np.asarray(item["segmentation"], dtype=bool)
        features = mask_features(mask_roi, rgb_roi)
        u_roi, v_roi = features.pop("center_roi_px")
        u_full, v_full = u_roi + x1, v_roi + y1

        full_mask = np.zeros((height, width), dtype=bool)
        full_mask[y1:y2, x1:x2] = mask_roi
        full_masks.append(full_mask)

        detection: dict[str, Any] = {
            "id": index,
            "center_full_px": [u_full, v_full],
            "bbox_roi_xywh": [float(value) for value in item["bbox"]],
            "area_px": int(item["area"]),
            "predicted_iou": float(item["predicted_iou"]),
            "stability_score": float(item["stability_score"]),
            **features,
        }
        if homography is not None:
            detection["world_xy"] = pixel_to_world_xy(u_full, v_full, homography)
        detections.append(detection)

    # Pick the candidate nearest the center of the configured robot workspace.
    suggested_pick_id: int | None = None
    if detections:
        suggested = min(
            detections,
            key=lambda item: float(
                np.linalg.norm(np.asarray(item["center_full_px"]) - roi_center)
            ),
        )
        suggested_pick_id = int(suggested["id"])

    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = {
        "schema_version": 1,
        "source_image": str(args.image.resolve()),
        "image_size_wh": [width, height],
        "roi_xyxy": list(roi),
        "model_id": args.model_id,
        "coordinate_frame": "full_image_pixels",
        "suggested_pick_id": suggested_pick_id,
        "detections": detections,
        "notes": [
            "Pixel centers are not robot coordinates unless world_xy is present.",
            "grasp_yaw_image_deg must be converted to the robot/tool convention.",
            "Review the overlay before connecting this output to motion control.",
        ],
    }
    json_path = args.output_dir / "detections.json"
    overlay_path = args.output_dir / "annotated.png"
    json_path.write_text(json.dumps(output, indent=2), encoding="utf-8")
    cv2.imwrite(str(overlay_path), annotate(bgr, detections, full_masks, roi))

    print(f"[vision] raw_masks={len(raw_masks)} kept_objects={len(detections)}")
    print(f"[vision] suggested_pick_id={suggested_pick_id}")
    print(f"[vision] detections={json_path}")
    print(f"[vision] overlay={overlay_path}")


if __name__ == "__main__":
    main()
