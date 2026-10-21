"""Organizer's supplied camera geometry. Base coordinates are metres.

UF0 TCP coordinates are mm: z_UF0 = 1000*z_base - 330.
"""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
SURFACE_Z = -0.0051
UF0_Z_MM = -330.0
CHAIN = [((0., 0., .33), "z", +1), ((0., 0., 0.), "y", +1),
         ((0., 0., .26), "y", -1), ((0., 0., .02), "x", -1),
         ((.29, 0., 0.), "y", -1), ((0., 0., 0.), "x", -1)]


def _rot(axis, a):
    c, s = np.cos(a), np.sin(a)
    return {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
            "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def flange_pose(joints):
    R, p = np.eye(3), np.zeros(3)
    for (off, axis, sign), q in zip(CHAIN, np.radians(np.asarray(joints, float)[:6])):
        p = p + R @ np.array(off)
        R = R @ _rot(axis, sign * q)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T


def load(path=HERE / "cameras.json"):
    raw = json.loads(Path(path).read_text())
    cams = {}
    for k, v in raw.items():
        if not isinstance(v, dict):
            continue
        cam = {"T": np.array(v.get("T_base_cam", v.get("T_flange_cam"))),
               "fx": v["fx"], "size": tuple(v["size"]),
               "frame": "base" if "T_base_cam" in v else "flange"}
        if "cx" in v:
            cam |= {"fy": v["fy"], "cx": v["cx"], "cy": v["cy"], "depth": True}
        else:
            cam |= {"k1": v["k1"]}
        cams[k] = cam
    return cams


def project(cam, pts):
    pts = np.asarray(pts, float).reshape(-1, 3)
    T = np.linalg.inv(cam["T"])
    pc = pts @ T[:3, :3].T + T[:3, 3]
    x, y = pc[:, 0] / pc[:, 2], pc[:, 1] / pc[:, 2]
    d = 1 + cam["k1"] * (x * x + y * y)
    w, h = cam["size"]
    return np.stack([cam["fx"] * x * d + w / 2, cam["fx"] * y * d + h / 2], -1)


def depth_to_base(cam, depth_mm, stride=1, with_pixels=False, joints=None):
    if not cam.get("depth"):
        raise ValueError("depth_to_base needs a depth camera")
    T = cam["T"]
    if cam["frame"] == "flange":
        if joints is None:
            raise ValueError("Wrist depth needs the observation's joints")
        T = flange_pose(joints) @ T
    d = np.asarray(depth_mm)[::stride, ::stride].astype(np.float64)
    v, u = np.mgrid[0:d.shape[0], 0:d.shape[1]] * stride
    m = d > 0
    z = d[m] / 1000.0
    pc = np.stack([(u[m]-cam["cx"])/cam["fx"]*z, (v[m]-cam["cy"])/cam["fy"]*z, z], -1)
    pts = pc @ T[:3, :3].T + T[:3, 3]
    return (pts, np.stack([u[m], v[m]], -1)) if with_pixels else pts


def pixel_to_base(cam, uv, z=SURFACE_Z):
    if cam.get("depth") or cam["frame"] != "base":
        raise ValueError("pixel_to_base needs a fixed RGB camera")
    w, h = cam["size"]
    xd, yd = (uv[0]-w/2)/cam["fx"], (uv[1]-h/2)/cam["fx"]
    x, y = xd, yd
    for _ in range(10):
        d = 1 + cam["k1"] * (x*x + y*y)
        x, y = xd/d, yd/d
    ray = cam["T"][:3, :3] @ np.array([x, y, 1.0])
    o = cam["T"][:3, 3]
    return o + ray * (z-o[2]) / ray[2]
