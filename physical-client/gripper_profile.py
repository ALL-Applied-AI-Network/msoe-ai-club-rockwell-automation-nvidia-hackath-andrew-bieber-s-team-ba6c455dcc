"""Physical grasp geometry adapted from Asher's dual-zone prototype (1fbc58b).

No Isaac imports, semantic labels, object poses, or measured-success claims.
The original 8--20 / 14--26 mm and revised 5.2--17.2 / 18--30 mm gaps
are mechanical limits, not camera thresholds. The revised geometry is a new,
unvalidated hardware design; it is not evidence of physical pickup performance.
The real controller still receives only open=0 / close=1; simulated fractional
jaw positions are not physical commands.

TCP offsets are explicitly configurable. Asher's +Z finger-root direction is
NOT proof of the physical controller's local axes. The proposed +X cap offset
and all inventory dimensions require mounting/measurement confirmation before
motion. ``contact_offset_m`` means controller TCP origin -> active contact
centre, expressed in controller TCP coordinates.
"""
from __future__ import annotations

import math
import numpy as np

import sort_config as S


PRIMARY_GAP_MM = (8.0, 20.0)
CAP_GAP_MM = (14.0, 26.0)
V2_PRIMARY_GAP_MM = (5.2, 17.2)
V2_WIDE_GAP_MM = (18.0, 30.0)
V2_OPEN_OUTER_WIDTH_M = .0489  # common backplane +/-18.45 mm plus 6 mm/jaw
STROKE_PER_JAW_MM = 6.0
OPEN_CLEARANCE_PER_SIDE_MM = 1.0
SQUEEZE_PER_SIDE_MM = 0.3
CAP_FLANGE_THICK_M = 0.088 * 0.0254
CAP_HEIGHT_M = 0.01570
CAP_FLANGE_DIAMETER_M = 0.03175
CAP_PLUG_BASE_DIAMETER_M = 0.930 * 0.0254
CAP_PLUG_TIP_DIAMETER_M = 0.820 * 0.0254


def profile_name():
    name = getattr(S, "PHYSICAL_GRIPPER_PROFILE", "legacy")
    if name not in ("legacy", "dual_zone", "dual_zone_v2"):
        raise ValueError("PHYSICAL_GRIPPER_PROFILE must be legacy, dual_zone or dual_zone_v2")
    return name


def _positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def _vector(value, name, length=3):
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {length} finite coordinates")
    return result


def _offsets():
    primary = _vector(getattr(S, "DUAL_ZONE_PRIMARY_OFFSET_TCP_M", (0, 0, 0)),
                      "primary TCP contact offset")
    delta = _vector(getattr(S, "DUAL_ZONE_CAP_OFFSET_TCP_M", (.040, 0, 0)),
                    "cap rail TCP offset")
    if np.linalg.norm(delta[:2]) < .001 or abs(delta[2]) > 1e-6:
        raise ValueError("Cap rail offset must be a nonzero offset in the TCP XY plane")
    separation = np.linalg.norm(delta[:2])
    if abs(separation - .040) > .001 + 1e-9:
        raise ValueError("Measured rail separation must match the 40 mm CAD offset within 1 mm")
    # For W=-180,P=0 the local TCP XY plane is reflected in Y before R is
    # applied. At the configured R for a +X object, the rail-offset axis must
    # consequently lie along world +/-X. Otherwise the end-vial grasp moves
    # sideways away from its body even though a clearance check could pass.
    aligned = rotation_wpr(-180., 0., S.TCP_R_FOR_AXIS_X) @ delta
    error = math.degrees(math.atan2(abs(aligned[1]), abs(aligned[0])))
    if error > 1. + 1e-9:
        raise ValueError("TCP_R_FOR_AXIS_X must align the measured rail-offset direction with the object long axis (within 1 degree, modulo 180)")
    return primary, delta


def rotation_wpr(w, p, r):
    """Controller convention Rz(R) Ry(P) Rx(W), angles in degrees."""
    angles = np.deg2rad(_vector((w, p, r), "TCP WPR"))
    cw, cp, cr = np.cos(angles)
    sw, sp, sr = np.sin(angles)
    return np.array([
        [cr*cp, cr*sp*sw-sr*cw, cr*sp*cw+sr*sw],
        [sr*cp, sr*sp*sw+cr*cw, sr*sp*cw-cr*sw],
        [-sp, cp*sw, cp*cw],
    ])


def cap_width_at(height_m):
    """Nominal plug diameter at fingertip height; requires narrow end up."""
    height_m = _positive(height_m, "cap pickup height")
    if not CAP_FLANGE_THICK_M < height_m < CAP_HEIGHT_M:
        raise ValueError("Cap fingertips must be above the flange and below the plug top")
    fraction = (height_m - CAP_FLANGE_THICK_M) / (CAP_HEIGHT_M - CAP_FLANGE_THICK_M)
    return CAP_PLUG_BASE_DIAMETER_M + fraction * (
        CAP_PLUG_TIP_DIAMETER_M - CAP_PLUG_BASE_DIAMETER_M)


def _fit(width_mm, gap, opening_clearance_mm=OPEN_CLEARANCE_PER_SIDE_MM):
    width_mm = _positive(width_mm, "contact width")
    low, high = gap
    if width_mm < low + 2*SQUEEZE_PER_SIDE_MM - 1e-9:
        raise ValueError(f"{width_mm:g} mm object cannot be clamped by the {low:g} mm closed gap")
    if width_mm + 2*opening_clearance_mm > high + 1e-9:
        raise ValueError(f"{width_mm:g} mm object lacks opening clearance in the {high:g} mm gap")
    return width_mm


def grasp_spec(target):
    """Return JSON-safe geometry for an observed target, or reject the grasp.

    The policy owns allowed kinds, pose bounds, operator confirmation and motion
    gating. Recognition still comes from the physical RGB/SAM pipeline. Nominal
    vial/cap dimensions describe the configured inventory, not measurements made
    by this function. A known-fit cube uses the declared inventory size range to
    avoid repeating the real camera's erroneous upper-width veto.
    """
    name = profile_name()
    kind = target["kind"]
    measured = _positive(target.get("short_mm", float("nan")), "camera grasp width")
    if name == "legacy":
        if measured > S.PHYSICAL_MAX_GRASP_WIDTH_MM and kind not in S.PHYSICAL_VERIFIED_FIT_KINDS:
            raise ValueError("grasp wider than the configured jaw clearance")
        return dict(profile=name, zone="primary", pick_height_m=S.PICK_HEIGHT[kind],
                    release_height_m=S.DROP_HEIGHT, contact_offset_m=[0., 0., 0.],
                    contact_from_center_tcp_m=[0., 0., 0.],
                    width_mm=measured,
                    object_length_m=max(measured, float(target.get("long_mm", measured)))/1000,
                    object_width_m=measured/1000,
                    object_height_m=float(target.get("height_m", measured/1000)))

    revised = name == "dual_zone_v2"
    if kind not in (("screw", "cube", "vial", "cap") if revised else ("cube", "vial", "cap")):
        raise ValueError("Dual-zone shaft grasp unsupported: 8 mm closed gap exceeds the M6 shaft; head grasp requires separate target logic")
    primary, delta = _offsets()
    heights = {"screw": .0025, "cube": .0025, "vial": .003, "cap": .005}
    heights.update(getattr(S, "DUAL_ZONE_PICK_HEIGHTS_M", {}))
    pick_height = _positive(heights[kind], "dual-zone pickup height")
    zone, offset = "primary", primary
    contact_from_center = np.zeros(3)
    primary_gap = V2_PRIMARY_GAP_MM if revised else PRIMARY_GAP_MM
    primary_clearance = .5 if revised else OPEN_CLEARANCE_PER_SIDE_MM
    if kind == "cube":
        if kind in S.PHYSICAL_VERIFIED_FIT_KINDS:
            limits = _vector(getattr(S, "DUAL_ZONE_CUBE_WIDTH_RANGE_MM", (9., 16.)),
                             "confirmed cube size range", length=2)
            low, high = limits
            if low > high:
                raise ValueError("Cube size range must be increasing")
            _fit(low, primary_gap, primary_clearance)
            width = _fit(high, primary_gap, primary_clearance)
        else:
            width = _fit(measured, primary_gap, primary_clearance)
        length = diameter = object_height = width / 1000
    elif kind == "screw":
        # V2 intentionally narrows the primary pads to grip the existing shaft
        # target. Its finite camera width can be distorted by the screw head.
        declared = getattr(S, "DUAL_ZONE_V2_SCREW_DIAMETER_MM", 6.)
        width = _fit(declared if kind in S.PHYSICAL_VERIFIED_FIT_KINDS else measured,
                     primary_gap, primary_clearance)
        length = _positive(getattr(S, "DUAL_ZONE_V2_SCREW_LENGTH_MM", 30.), "screw length")/1000
        diameter = object_height = width/1000
        if length <= diameter:
            raise ValueError("Configured screw length must exceed its shaft diameter")
    elif kind == "vial":
        if revised:
            width = _fit(getattr(S, "DUAL_ZONE_V2_VIAL_DIAMETER_MM", 27.18), V2_WIDE_GAP_MM)
            length = _positive(getattr(S, "DUAL_ZONE_V2_VIAL_LENGTH_MM", 203.), "vial length")/1000
            zone, offset = "wide", primary + delta
            inset = _positive(getattr(S, "DUAL_ZONE_V2_VIAL_END_INSET_M", .020), "vial end-grasp inset")
            if inset >= length/2:
                raise ValueError("Vial end-grasp inset must be less than half its length")
            # Move the active wide rail toward the end opposite the offset:
            # the inactive primary rail then sits beyond that same vial end.
            # For 203 mm length, 20 mm inset and 40 mm rail offset the primary
            # centre is 20 mm past the end; its 3 mm half-thickness leaves 17 mm.
            contact_from_center = -delta/np.linalg.norm(delta) * (length/2-inset)
        else:
            width = _fit(getattr(S, "DUAL_ZONE_VIAL_DIAMETER_MM", 17.), PRIMARY_GAP_MM)
            length = _positive(getattr(S, "DUAL_ZONE_VIAL_LENGTH_MM", 140.), "vial length")/1000
        diameter = object_height = width/1000
        if length <= diameter:
            raise ValueError("Configured vial length must exceed its diameter")
    else:
        if not getattr(S, "DUAL_ZONE_CAP_NARROW_END_UP", False):
            raise ValueError("Cap orientation must be confirmed narrow-end-up for the offset plug grasp")
        width = _fit(cap_width_at(pick_height)*1000, V2_WIDE_GAP_MM if revised else CAP_GAP_MM)
        zone, offset = "wide" if revised else "cap", primary + delta
        length = diameter = CAP_FLANGE_DIAMETER_M
        object_height = CAP_HEIGHT_M
    if pick_height >= object_height:
        raise ValueError("Pickup height must remain below the configured object top")
    return dict(profile=name, zone=zone, pick_height_m=pick_height,
                release_height_m=pick_height, contact_offset_m=offset.tolist(),
                contact_from_center_tcp_m=contact_from_center.tolist(),
                width_mm=width, object_length_m=length, object_width_m=diameter,
                object_height_m=object_height)


def contact_to_tcp_base(xy, z, r, spec):
    """Desired active contact centre -> controller TCP position in base metres."""
    xy = _vector(xy, "contact XY", length=2)
    desired = _vector((*xy, z), "contact XYZ")
    offset = _vector(spec["contact_offset_m"], "contact offset")
    return desired - rotation_wpr(-180., 0., r) @ offset


def object_to_contact_base(xy, r, spec):
    """Desired object-centre XY -> contact XY for the selected grasp and yaw.

    Most grasps are centred. The V2 vial is grasped near one end so the inactive
    narrow rail clears its body; keeping this transform separate prevents a
    release target from accidentally being interpreted as the contact point.
    """
    xy = _vector(xy, "object centre XY", length=2)
    offset = _vector(spec.get("contact_from_center_tcp_m", (0., 0., 0.)),
                     "contact from object centre")
    return xy + (rotation_wpr(-180., 0., r) @ offset)[:2]


def tcp_to_contact_base(pose, spec):
    """Controller [UF0 mm XYZ, WPR degrees, optional grip] -> active contact metres."""
    pose = np.asarray(pose, dtype=float)
    if pose.shape not in ((6,), (7,)) or not np.all(np.isfinite(pose)):
        raise ValueError("Expected a finite six/seven element TCP pose")
    xyz = pose[:3]/1000
    xyz[2] -= S.UF0_Z_MM/1000
    return xyz + rotation_wpr(*pose[3:6]) @ _vector(spec["contact_offset_m"], "contact offset")


def tcp_to_object_xy(pose, spec):
    """Actual controller pose -> nominal carried-object centre XY."""
    contact = tcp_to_contact_base(pose, spec)
    offset = _vector(spec.get("contact_from_center_tcp_m", (0., 0., 0.)),
                     "contact from object centre")
    return contact[:2] - (rotation_wpr(*pose[3:6]) @ offset)[:2]


def _rectangle(centre, axis, length, width, margin):
    centre = _vector(centre, "rectangle centre", length=2)
    axis = _vector(axis, "rectangle axis", length=2)
    norm = np.linalg.norm(axis)
    if norm < 1e-9:
        raise ValueError("Rectangle axis must be nonzero")
    axis /= norm
    side = np.array([-axis[1], axis[0]])
    length, width = _positive(length, "rectangle length"), _positive(width, "rectangle width")
    margin = float(margin)
    if not math.isfinite(margin) or margin < 0:
        raise ValueError("Rectangle margin must be finite and nonnegative")
    return np.array([centre + a*(length/2+margin)*axis + b*(width/2+margin)*side
                     for a, b in ((-1,-1), (1,-1), (1,1), (-1,1))])


def rail_rectangles(pose, margin_m=0.):
    """Conservative low-object XY envelopes for BOTH open finger rails.

    Each solid envelope includes the empty jaw gap: callers should exclude only
    the intended grasp object while screening other observed objects. It spans
    6 mm along the rail-offset axis and 36 mm across the opening in the original
    design (12 mm stock + 6 mm travel per side). Final V2 CAD uses a common flat
    backplane at closed Y +/-18.45 mm, giving 48.9 mm open envelopes at BOTH
    rails. The lowest bridge begins 56 mm above
    the fingertip in the original and 47.7 mm above in V2. These footprints do
    not model that bridge, the wrist, or the full arm.
    """
    name = profile_name()
    if name not in ("dual_zone", "dual_zone_v2"):
        return []
    primary, delta = _offsets()
    pose = np.asarray(pose, dtype=float)
    if pose.shape not in ((6,), (7,)) or not np.all(np.isfinite(pose)):
        raise ValueError("Expected finite TCP pose")
    rotation = rotation_wpr(*pose[3:6])
    # These XY footprints are intended only for the policy's downward tool pose.
    if abs(abs(rotation[2,2])-1) > 1e-6:
        raise ValueError("Rail footprint requires a vertical tool axis")
    axis = (rotation @ delta)[:2]
    centres = [tcp_to_contact_base(pose, {"contact_offset_m": x})[:2]
               for x in (primary, primary+delta)]
    open_width = V2_OPEN_OUTER_WIDTH_M if name == "dual_zone_v2" else .036
    return [_rectangle(centre, axis, .006, open_width, margin_m) for centre in centres]


def object_rectangle(target, xy=None, yaw_deg=None, margin_m=0.):
    """Observed object footprint expanded to include configured nominal sizes.

    This helper does not require the object to be graspable: screws and unknown
    recognized shapes can still obstruct the dual rails. Prefer the larger of
    observed and nominal extents, so a camera-width override does not shrink a
    collision obstacle. These remain estimated 2-D footprints, not 3-D sensing.
    """
    kind = target["kind"]
    long = _positive(target.get("long_mm", target.get("short_mm", float("nan"))), "object length")/1000
    short = _positive(target.get("short_mm", float("nan")), "object width")/1000
    if kind == "cube":
        limits = _vector(getattr(S, "DUAL_ZONE_CUBE_WIDTH_RANGE_MM", (9., 16.)), "cube size range", 2)
        long, short = max(long, limits.max()/1000), max(short, limits.max()/1000)
    elif kind == "vial":
        if profile_name() == "dual_zone_v2":
            length = getattr(S, "DUAL_ZONE_V2_VIAL_LENGTH_MM", 203.)
            diameter = getattr(S, "DUAL_ZONE_V2_VIAL_DIAMETER_MM", 27.18)
        else:
            length = getattr(S, "DUAL_ZONE_VIAL_LENGTH_MM", 140.)
            diameter = getattr(S, "DUAL_ZONE_VIAL_DIAMETER_MM", 17.)
        long = max(long, _positive(length, "vial length")/1000)
        short = max(short, _positive(diameter, "vial diameter")/1000)
    elif kind == "screw" and profile_name() == "dual_zone_v2":
        long = max(long, _positive(getattr(S, "DUAL_ZONE_V2_SCREW_LENGTH_MM", 30.), "screw length")/1000)
        short = max(short, _positive(getattr(S, "DUAL_ZONE_V2_SCREW_DIAMETER_MM", 6.), "screw diameter")/1000)
    elif kind == "cap":
        long, short = max(long, CAP_FLANGE_DIAMETER_M), max(short, CAP_FLANGE_DIAMETER_M)
    angle = math.radians(float(target["yaw_base_deg"] if yaw_deg is None else yaw_deg))
    if not math.isfinite(angle):
        raise ValueError("Object yaw must be finite")
    return _rectangle(target["world_xy"] if xy is None else xy,
                      (math.cos(angle), math.sin(angle)), long, short, margin_m)
