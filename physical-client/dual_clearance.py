"""Conservative tabletop clearance for the physical dual-zone profile.

Both finger rails and an assumed carried-object footprint are screened against
supplied RGB detections/reservations and optional measured static rectangles.
This is a 2.5-D local guard, not simulator ground truth or full-arm planning.
Unknown, missed and moved objects remain limitations of the physical camera.
The caller must supply every interpolated TCP row, including the current pose.
"""
from __future__ import annotations

import math
import numpy as np

import sort_config as S
import gripper_profile as G


class ClearanceError(ValueError):
    """A target/path cannot be screened as clear; no motion should be queued."""


def _number(value, name, *, positive=False):
    result = float(value)
    if not math.isfinite(result) or (positive and result <= 0):
        raise ClearanceError(f"{name} must be finite" + (" and positive" if positive else ""))
    return result


def polygons_overlap(a, b, clearance_m=0.):
    """Convex-polygon SAT, conservatively treating touching as overlap."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    for points in (a, b):
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3 or not np.all(np.isfinite(points)):
            raise ClearanceError("Invalid obstacle/rail polygon")
    margin = _number(clearance_m, "clearance")
    if margin < 0:
        raise ClearanceError("Clearance must be nonnegative")
    for points in (a, b):
        edges = np.roll(points, -1, axis=0) - points
        for edge in edges:
            length = np.linalg.norm(edge)
            if length < 1e-12:
                continue
            axis = np.array([-edge[1], edge[0]]) / length
            pa, pb = a @ axis, b @ axis
            if pa.max() + margin < pb.min() or pb.max() + margin < pa.min():
                return False
    return True


def _xy(record):
    value = np.asarray(record["world_xy"], float)
    if value.shape != (2,) or not np.all(np.isfinite(value)):
        raise ClearanceError("Obstacle/source XY must contain two finite values")
    return value


def _same_source(record, target):
    return (target is not None and record.get("kind") == target.get("kind")
            and "world_xy" in record and "world_xy" in target
            and np.linalg.norm(_xy(record)-_xy(target)) < .008)


def _obstacle(record):
    """Known detection or explicit static bounds -> label, polygon, base, top."""
    label = str(record.get("label", record.get("kind", record.get("name", "static obstacle"))))
    if "kind" in record:
        _xy(record)
        polygon = G.object_rectangle(record)
        kind = record["kind"]
        nominal = {
            "cube": max(getattr(S, "DUAL_ZONE_CUBE_WIDTH_RANGE_MM", (9., 16.)))/1000,
            "vial": (getattr(S, "DUAL_ZONE_V2_VIAL_DIAMETER_MM", 27.18)
                     if G.profile_name() == "dual_zone_v2" else
                     getattr(S, "DUAL_ZONE_VIAL_DIAMETER_MM", 17.))/1000,
            "cap": G.CAP_HEIGHT_M,
            "screw": .006,
        }.get(kind)
        if nominal is None:
            nominal = record.get("height_m", float("nan"))
        height = max(_number(nominal, "nominal obstacle height", positive=True),
                     _number(record.get("height_m", nominal), "observed obstacle height", positive=True))
    else:
        bounds = np.asarray(record.get("bounds_xy"), float)
        if bounds.shape != (2, 2) or not np.all(np.isfinite(bounds)) or np.any(bounds[1] <= bounds[0]):
            raise ClearanceError("Static obstacle needs increasing finite bounds_xy")
        (x0, y0), (x1, y1) = bounds
        polygon = np.array([[x0,y0], [x1,y0], [x1,y1], [x0,y1]])
        height = _number(record.get("height_m", float("nan")), "static obstacle height", positive=True)
    base = _number(record.get("base_z_m", S.SURFACE_Z), "obstacle base height")
    max_height = _number(getattr(S, "DUAL_ZONE_MAX_OBSTACLE_HEIGHT_M", .045),
                         "maximum screened obstacle height", positive=True)
    if base < S.SURFACE_Z - 1e-9 or base + height > S.SURFACE_Z + max_height + 1e-9:
        raise ClearanceError(f"{label} exceeds the tabletop height model; bridge/wrist clearance is unknown")
    return label, polygon, base, base + height


def check_path(poses, target, obstacles, carrying=False):
    """Reject a dual-zone path that overlaps a supplied obstacle.

    ``target`` is the intended source detection, excluded by kind/location from
    obstacle checks. All other detections, including ungraspable screws/caps,
    remain obstacles. ``carrying`` screens a possible payload only on closed-
    gripper rows, allowing a complete pick/drop/open plan to be checked first.
    Its presence is an assumption for clearance, never a pickup-success claim.
    """
    profile = G.profile_name()
    if profile not in ("dual_zone", "dual_zone_v2"):
        return
    try:
        rows = np.asarray(list(poses), float)
        if rows.ndim != 2 or rows.shape[1] != 7 or not len(rows) or not np.all(np.isfinite(rows)):
            raise ClearanceError("Expected finite seven-element TCP rows for clearance")
        margin = _number(getattr(S, "DUAL_ZONE_CLEARANCE_MM", 2.), "clearance margin")/1000
        if margin < 0:
            raise ClearanceError("Clearance margin must be nonnegative")
        records = list(obstacles) + list(getattr(S, "DUAL_ZONE_STATIC_OBSTACLES", ()))
        prepared = [_obstacle(o) for o in records if not _same_source(o, target)]
        max_top = max((top for _, _, _, top in prepared), default=S.SURFACE_Z)
        spec = G.grasp_spec(target) if carrying and target is not None else None
        if carrying and target is None:
            raise ClearanceError("Carried-object screening requires a target")
        primary = np.asarray(getattr(S, "DUAL_ZONE_PRIMARY_OFFSET_TCP_M", (0.,0.,0.)), float)
        delta = np.asarray(getattr(S, "DUAL_ZONE_CAP_OFFSET_TCP_M", (.040,0.,0.)), float)
        low_height = S.SURFACE_Z + _number(
            getattr(S, "DUAL_ZONE_MAX_OBSTACLE_HEIGHT_M", .045),
            "maximum screened obstacle height", positive=True) + margin
        for pose in rows:
            primary_contact = G.tcp_to_contact_base(pose, {"contact_offset_m": primary})
            cap_contact = G.tcp_to_contact_base(pose, {"contact_offset_m": primary+delta})
            tip_z = min(primary_contact[2], cap_contact[2])
            if tip_z < S.SURFACE_Z + .001 - 1e-9:
                raise ClearanceError("Finger tips would cross the configured table clearance")
            payload = spec is not None and pose[6] >= .5
            contact = G.tcp_to_contact_base(pose, spec) if payload else None
            bottom = contact[2] - spec["pick_height_m"] if payload else tip_z
            carried = None
            if payload:
                yaw = pose[5] - S.TCP_R_FOR_AXIS_X
                # A near-end vial grasp places its centre far from the active
                # rail. Using the rail contact as the centre misses its far end.
                object_xy = G.tcp_to_object_xy(pose, spec)
                carried = G.object_rectangle(target, xy=object_xy, yaw_deg=yaw)
                if profile == "dual_zone_v2" and target["kind"] == "vial":
                    rotation = G.rotation_wpr(*pose[3:6])
                    if rotation[2,2] > -1 + 1e-6:
                        raise ClearanceError("Near-end vial clearance requires a downward vertical tool")
                    # The small zone cannot surround the real 27.18 mm vial.
                    # It must be past the vial end, even though the intended
                    # object is otherwise excluded from ordinary rail checks.
                    if polygons_overlap(G.rail_rectangles(pose)[0], carried, margin):
                        raise ClearanceError("Inactive primary rail overlaps the held vial; check near-end grasp geometry")
            if min(tip_z, bottom) <= low_height:
                rotation = G.rotation_wpr(*pose[3:6])
                if rotation[2,2] > -1 + 1e-6:
                    raise ClearanceError("Low-height clearance requires a downward vertical tool")
            # All screened objects are below the bridge. At adequate height,
            # this path row cannot intersect their tabletop envelopes.
            if min(tip_z, bottom) > max_top + margin:
                continue
            rails = G.rail_rectangles(pose)
            for label, polygon, base, top in prepared:
                if tip_z <= top + margin and tip_z + .076 >= base - margin:
                    if any(polygons_overlap(rail, polygon, margin) for rail in rails):
                        raise ClearanceError(f"Finger rail near {label}; defer target or choose another approach")
                if carried is not None and bottom <= top + margin and bottom + spec["object_height_m"] >= base - margin:
                    if polygons_overlap(carried, polygon, margin):
                        raise ClearanceError(f"Carried object near {label}; destination/path is occupied")
    except ClearanceError:
        raise
    except (ValueError, TypeError, KeyError, IndexError) as exc:
        raise ClearanceError(f"Invalid clearance geometry: {exc}") from exc
