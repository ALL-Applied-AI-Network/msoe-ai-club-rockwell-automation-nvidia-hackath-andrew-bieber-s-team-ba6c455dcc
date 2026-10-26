"""Flat padded finger sets (padded / cap / universal).  Dimensions in metres.

Profile coordinates match gripper_geometry.finger_profile():
  u = along the finger (0 = carriage end, FINGER_LENGTH = tip)
  v = across it (0 = inner root face, FINGER_WIDTH = outer face)

Each finger is a 76 x 12 x 6 mm bar.  From u = PAD_STEP_X to the tip the inner face
is relieved back to v = PAD_FACE and a PAD_T rubber pad sits on that relief, with a
small lead-in chamfer on the pad's tip edge.  Pad-to-pad gap:

    closed = 2 * PAD_FACE                 (root faces touch)
    open   = 2 * (JAW_OPEN + PAD_FACE)

Supported sets: scallop (original), padded (8--20 mm), dual (primary
8--20 mm plus an offset 14--26 mm cap zone). Both padded sets retain 6 mm/jaw.
The former cap/universal doc examples were not implemented in scene_config.
Run FINGER_SET=dual python dual_zone.py for the two-zone fit report.
"""

import sys

import scene_config as C


def enabled():
    return C.FINGER_SET in ("padded", "dual")


def finger_sections(sign, finger_phys_path, pad_phys_path):
    """Sections for cell.attach_gripper: (polygon, name, visual_material, physics_material)."""
    L, W = C.FINGER_LENGTH, C.FINGER_WIDTH
    x0, face, t = C.PAD_STEP_X, C.PAD_FACE, C.PAD_T
    back = face + t  # support face behind the pad
    body = [(0.0, 0.0), (x0, 0.0), (x0, back), (L, back), (L, W), (0.0, W)]
    c = min(C.PAD_LEAD_IN, (L - x0) / 2.0)
    f = 0.8 * t  # the tip edge flares away from the part by this much
    if c > 0.0:
        pad = [(x0, face), (L - c, face), (L, face + f), (L, back), (x0, back)]
    else:
        pad = [(x0, face), (L, face), (L, back), (x0, back)]

    def signed(poly):
        return [(u, sign * v) for u, v in poly]

    return [
        (signed(body), "Finger", "alu_machined", finger_phys_path),
        (signed(pad), "SoftTip", "soft_tip", pad_phys_path),
    ]


# ---------------------------------------------------------------- grasp helpers
def closed_gap():
    return 2.0 * C.PAD_FACE


def open_gap():
    return 2.0 * (C.JAW_OPEN + C.PAD_FACE)


def fits_open(width, clearance=0.001):
    """True if a part of this width passes between the open pads."""
    return width + 2.0 * clearance <= open_gap()


def can_squeeze(width, margin=0.0006):
    """True if the closed pads are narrow enough to actually clamp this part."""
    return width >= closed_gap() + margin


def jaw_for_width(width, squeeze=0.0003):
    """Per-side jaw command (metres, same scale as C.JAW_OPEN) that grips `width`.

    Pad-face half-gap = jaw + PAD_FACE, so jaw = width/2 - PAD_FACE - squeeze.
    """
    if not enabled():
        raise RuntimeError("jaw_for_width is for the padded sets; set FINGER_SET")
    if not can_squeeze(width):
        raise ValueError(
            f"{width * 1000:.1f} mm part is not wider than the closed pad gap "
            f"{closed_gap() * 1000:.1f} mm: these fingers cannot clamp it"
        )
    jaw = width / 2.0 - C.PAD_FACE - squeeze
    if jaw > C.JAW_OPEN:
        raise ValueError(
            f"{width * 1000:.1f} mm part is wider than the open pad gap "
            f"{open_gap() * 1000:.1f} mm"
        )
    return max(C.JAW_CLOSED, jaw)


def cap_width_at(height_above_surface):
    """Diameter of the tapered cap plug at a height above the work surface.

    The cap lies flat, narrow end up.  Flat pads touch it where they are lowest.
    """
    h = min(max(height_above_surface - C.CAP_FLANGE_THICK, 0.0), C.CAP_PLUG_LENGTH)
    base, tip = 2 * C.CAP_PLUG_BASE_RADIUS, 2 * C.CAP_PLUG_TIP_RADIUS
    return base - (base - tip) * h / C.CAP_PLUG_LENGTH


def jaw_for_object(object_type, cap_pick_height=0.005):
    """Jaw command for 'cube', 'vial' or 'cap' (narrow end up), for the pick script."""
    if object_type == "cube":
        width = C.CUBE_EDGE
    elif object_type == "vial":
        width = 2 * C.VIAL_RADIUS
    elif object_type == "cap":
        width = cap_width_at(cap_pick_height)
    else:
        raise ValueError(f"unknown object type {object_type!r}")
    return jaw_for_width(width)


def report(extra_mm=()):
    print(f"finger set : {C.FINGER_SET}")
    if not enabled():
        print("(set FINGER_SET=padded, cap or universal to see a padded-finger window)")
        return
    left = (C.FINGER_WIDTH - C.PAD_FACE - C.PAD_T) * 1000
    print(f"PAD_FACE   : {C.PAD_FACE * 1000:.2f} mm, pad {C.PAD_T * 1000:.1f} mm, "
          f"support left behind pad {left:.1f} mm")
    print(f"stroke     : {C.JAW_OPEN * 1000:.1f} mm per side")
    print(f"pad gap    : closed {closed_gap() * 1000:.1f} mm, open {open_gap() * 1000:.1f} mm\n")
    parts = [
        ("cube", C.CUBE_EDGE),
        ("vial", 2 * C.VIAL_RADIUS),
        ("cap plug @ pick height", cap_width_at(0.005)),
        ("cap plug top", 2 * C.CAP_PLUG_TIP_RADIUS),
        ("cap plug base", 2 * C.CAP_PLUG_BASE_RADIUS),
    ] + [(f"your part {w:g} mm", w / 1000.0) for w in extra_mm]
    print(f"{'part':24s} {'width':>7s}  {'clears open':12s} {'can clamp':10s} {'jaw cmd':>9s}")
    for name, w in parts:
        clears, clamps = fits_open(w), can_squeeze(w)
        try:
            cmd = f"{jaw_for_width(w) * 1000:.2f} mm"
        except ValueError:
            cmd = "-"
        print(f"{name:24s} {w * 1000:6.2f}  {'yes' if clears else 'NO':12s} "
              f"{'yes' if clamps else 'NO':10s} {cmd:>9s}")
    print("\n'cap plug top' only has to clear the open pads; the pads grip it at pick height.")


if __name__ == "__main__":
    report([float(a) for a in sys.argv[1:]])