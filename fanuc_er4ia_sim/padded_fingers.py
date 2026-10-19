"""Padded flat finger set, taken from Padded_Gripper_Test.STEP.  Dimensions in metres.

Profile coordinates match gripper_geometry.finger_profile():
  u = along the finger (0 = carriage end, FINGER_LENGTH = tip)
  v = across it (0 = inner root face, FINGER_WIDTH = outer face)

Each finger is a 76 x 12 x 6 mm bar.  From u = PAD_STEP_X to the tip the inner
face is relieved back to v = PAD_FACE, and a PAD_T rubber pad sits on that relief.
With the jaws on the SMC's 6 mm-per-side stroke the pad-to-pad gap is

    closed = 2 * PAD_FACE                 (root faces touch)
    open   = 2 * (JAW_OPEN + PAD_FACE)

STEP values (PAD_FACE = 4 mm) give 8 mm closed / 20 mm open, which is what the
designer reported.

Enable with FINGER_SET=padded (see scene_config.py).  PAD_FACE_MM=<mm> overrides
the relief so you can try other designs without touching code.

Run it standalone (no Isaac needed) to see what fits:
    FINGER_SET=padded python padded_fingers.py
"""

import scene_config as C


def enabled():
    return C.FINGER_SET == "padded"


def finger_sections(sign, finger_phys_path, pad_phys_path):
    """Sections for cell.attach_gripper: (polygon, name, visual_material, physics_material).

    Same shape the scallop code builds, so the visual prisms, the ear-clipped
    convex-hull colliders and the soft-tip friction material all work unchanged.
    The polygons are non-convex (L shape) but each collider piece is a triangle.
    """
    L, W = C.FINGER_LENGTH, C.FINGER_WIDTH
    x0, face, t = C.PAD_STEP_X, C.PAD_FACE, C.PAD_T
    back = face + t  # support face behind the pad
    body = [(0.0, 0.0), (x0, 0.0), (x0, back), (L, back), (L, W), (0.0, W)]
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


def jaw_for_width(width, squeeze=0.0003):
    """Per-side jaw command (metres, same units as C.JAW_OPEN) that grips `width`.

    Pad face half-gap = jaw + PAD_FACE, so jaw = width/2 - PAD_FACE - squeeze.
    Use this instead of C.CUBE_JAW_HALF_TARGET for caps and vials.
    """
    if not enabled():
        raise RuntimeError("jaw_for_width is for the padded set; set FINGER_SET=padded")
    jaw = width / 2.0 - C.PAD_FACE - squeeze
    if jaw > C.JAW_OPEN:
        raise ValueError(
            f"{width * 1000:.1f} mm is wider than the open pad gap {open_gap() * 1000:.1f} mm"
        )
    return max(C.JAW_CLOSED, jaw)


def report():
    print(f"finger set : {C.FINGER_SET}")
    if not enabled():
        print("(set FINGER_SET=padded to see the padded-finger window)")
        return
    print(f"PAD_FACE   : {C.PAD_FACE * 1000:.2f} mm   pad {C.PAD_T * 1000:.1f} mm"
          f"   support left behind pad {(C.FINGER_WIDTH - C.PAD_FACE - C.PAD_T) * 1000:.1f} mm")
    print(f"pad gap    : closed {closed_gap() * 1000:.1f} mm, open {open_gap() * 1000:.1f} mm\n")
    parts = [
        ("cube (sim)", C.CUBE_EDGE),
        ("vial (sim collider)", 2 * C.VIAL_RADIUS),
        ("vial (CAD 27.18)", 0.02718),
        ("vial (your 28 mm)", 0.028),
        ("cap plug tip", 2 * C.CAP_PLUG_TIP_RADIUS),
        ("cap plug base", 2 * C.CAP_PLUG_BASE_RADIUS),
    ]
    print(f"{'part':22s} {'width':>8s}  {'clears open?':13s} {'jaw cmd':>9s}")
    for name, w in parts:
        ok = fits_open(w)
        try:
            cmd = f"{jaw_for_width(w) * 1000:.2f} mm" if ok else "-"
        except ValueError:
            cmd = "-"
        print(f"{name:22s} {w * 1000:7.2f}  {'yes' if ok else 'NO':13s} {cmd:>9s}")
    print("\nPAD_FACE needed for 1 mm clearance per side when open:")
    for name, w in parts[1:]:
        pf = (w + 0.002) / 2.0 - C.JAW_OPEN
        left = (C.FINGER_WIDTH - pf - C.PAD_T) * 1000
        print(f"  {name:22s} PAD_FACE_MM >= {pf * 1000:5.2f}   support left {left:5.1f} mm")


if __name__ == "__main__":
    report()