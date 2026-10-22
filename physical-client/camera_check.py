"""Inspect the physical camera mapping without connecting to a robot or loading SAM."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from calibration.geometry import load, project, pixel_to_base
import sort_config as S


def check_frame_size(size, cam):
    w, h = size
    cw, ch = cam["size"]
    if w <= 0 or h <= 0 or w * ch != h * cw:
        raise ValueError("Use a whole, uncropped table-camera frame at its original aspect ratio")


def mapping_error(landmarks, size, cam):
    """Landmarks: [{pixel: [u,v], base_mm: [x,y], optional z_base_mm: ...}]."""
    check_frame_size(size, cam)
    scale = np.array(cam["size"], float) / np.array(size, float)
    records = []
    for mark in landmarks:
        uv = np.asarray(mark["pixel"], float)
        xy = np.asarray(mark["base_mm"], float)
        if uv.shape != (2,) or xy.shape != (2,) or not np.all(np.isfinite([uv,xy])):
            raise ValueError("Each landmark needs two finite pixel and base_mm coordinates")
        if not (0 <= uv[0] < size[0] and 0 <= uv[1] < size[1]):
            raise ValueError("Landmark pixel is outside this image")
        z = float(mark.get("z_base_mm", S.SURFACE_Z * 1000)) / 1000
        predicted = pixel_to_base(cam, uv*scale, z=z)[:2] * 1000
        records.append({"pixel": uv.tolist(), "expected_base_mm": xy.tolist(),
                        "predicted_base_mm": predicted.tolist(),
                        "error_mm": float(np.linalg.norm(predicted-xy))})
    if not records:
        raise ValueError("Provide at least one measured landmark")
    return records


def draw_mapping(image, cam, output):
    """Projected outlines on the raw frame; drawing never changes the policy mapping."""
    check_frame_size(image.size, cam)
    image = image.convert("RGB").copy()
    draw = ImageDraw.Draw(image)
    w, h = image.size
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", max(12, round(w/95)))
    except OSError:
        font = ImageFont.load_default()
    scale = np.array([w, h]) / cam["size"]

    def pixels(points):
        return project(cam, points) * scale

    def label(point, text, color, dx=7, dy=-20):
        u,v = pixels([point])[0]
        if 0 <= u < w and 0 <= v < h:
            draw.ellipse((u-4,v-4,u+4,v+4),fill=color)
            box=draw.textbbox((u+dx,v+dy),text,font=font)
            draw.rectangle(box,fill=(255,255,255))
            draw.text((u+dx,v+dy),text,font=font,fill=color)

    xmin,xmax,ymin,ymax=S.WORKSPACE
    corners=[[xmin,ymin,S.SURFACE_Z],[xmax,ymin,S.SURFACE_Z],
             [xmax,ymax,S.SURFACE_Z],[xmin,ymax,S.SURFACE_Z]]
    outline=pixels(corners)
    draw.line([tuple(p) for p in [*outline,outline[0]]],fill="blue",width=3)
    label([xmin,ymin,S.SURFACE_Z],"pickup workspace",(0,60,200),dy=12)
    origin=[0,0,S.SURFACE_Z]
    label(origin,"robot base XY origin",(150,0,0),dx=-140,dy=-45)
    for end,text in [([.15,0,S.SURFACE_Z],"base +X"),([0,.15,S.SURFACE_Z],"base +Y")]:
        a,b=pixels([origin,end])
        draw.line([tuple(a),tuple(b)],fill=(150,0,0),width=3)
        label(end,text,(150,0,0),dy=8 if text.endswith("Y") else -20)
    angles=np.linspace(0,2*np.pi,65)
    for name,(x,y) in S.DROP_XY.items():
        ring=np.column_stack((x+S.DROP_RADIUS*np.cos(angles),
                              y+S.DROP_RADIUS*np.sin(angles),np.full(len(angles),S.SURFACE_Z)))
        draw.line([tuple(p) for p in pixels(ring)],fill=(0,120,40),width=2)
        label([x,y,S.SURFACE_Z],f"{name} ({x*1000:.0f},{y*1000:.0f}) mm",(0,100,30))
    label([*S.SCAN_XY,S.SURFACE_Z],"scan XY (not a drop)",(130,70,0))
    draw.rectangle((0,0,w,52),fill="white")
    draw.text((12,8),"Physical calibration projection; raw image orientation; positions require cell verification",
              font=font,fill="black")
    output=Path(output)
    output.parent.mkdir(parents=True,exist_ok=True)
    image.save(output)
    return output


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image",type=Path,help="Whole raw table-camera frame; do not crop or flip")
    parser.add_argument("--output",type=Path,default=Path("camera_mapping.png"))
    parser.add_argument("--landmarks",type=Path,help="JSON list of measured pixel/base_mm pairs")
    parser.add_argument("--tolerance-mm",type=float,default=3.0)
    args=parser.parse_args()
    cam=load()["table"]
    image=Image.open(args.image).convert("RGB") if args.image else Image.new("RGB",cam["size"],"#eeeeee")
    draw_mapping(image,cam,args.output)
    print(f"Saved {args.output}")
    if args.landmarks:
        if not args.image:
            parser.error("Measured landmarks must be checked against an actual --image")
        if not np.isfinite(args.tolerance_mm) or args.tolerance_mm <= 0:
            parser.error("--tolerance-mm must be positive")
        records=mapping_error(json.loads(args.landmarks.read_text()),image.size,cam)
        args.output.with_suffix(".json").write_text(json.dumps(records,indent=2))
        error=max(x["error_mm"] for x in records)
        print(f"Maximum measured mapping error: {error:.3f} mm")
        if error > args.tolerance_mm:
            raise SystemExit("Mapping exceeds the selected tolerance; check calibration/frame orientation")
    else:
        print("Projection preview only: no measured real-frame landmarks were supplied.")


if __name__=="__main__":
    main()
