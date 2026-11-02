# Dual-zone V2 printable finger prototype

> **V2.2 candidate (November 1):** the primary screw rail and pad now run
> **24 mm** along −X instead of 6 mm, while their existing +X edge stays fixed.
> This extends the contact away from the wide zone after the October 31
> benchmark recorded 50 lost screw attempts caused by the round 6 mm shank
> slowly rotating between the flat pads during carry. The pad remains flat:
> cutting a V into the 1 mm finished TPU stack would change its squeeze and the
> already-successful cube contact. This is unbenchmarked and physically untested.
>
> V2.1's 50 mm wide-zone rail remains unchanged; it resists vial pivot. The
> bridge is 20 mm tall (Z 25.8–45.8) and now supports both longer rails. Reprint
> both bodies and both pad sizes for this candidate.

This revision keeps Asher's two fixed contact zones on one parallel jaw mechanism,
with the same **6 mm travel per jaw**, and changes their gaps to cover the actual
CSI objects. It is an **engineering fit prototype**, not hardware validated for
robot operation. The original `feature/mvp2` and Asher designs remain separate.

## What to print and what to open

- `cad/mounting_coupon.stl`: print this small part first to check the installed
  jaw face and two mounting holes. Its flat backing face is already on Z = 0.
- `cad/left_body.stl` and `cad/right_body.stl`: one of each. Units are millimetres;
  do not scale. Left means the positive-Y stock jaw, viewed in the official STEP
  coordinate frame. Both STLs are oriented with the flat outer face on the bed.
- `cad/left_body_tinker_logo.stl`: the same left body with the Tinker robot
  face engraved 0.6 mm deep on its outer face, behind the wide rail. It is
  cosmetic only. Print this **or** `left_body.stl`, not both. Regenerate it with
  `python engrave_tinker_logo.py` (needs `manifold3d` and NumPy). There is
  no STEP version.
- `cad/contact_pad_primary_quantity_2.stl`: two narrow-zone pads, **24 × 32 × 1 mm**.
- `cad/contact_pad_wide_quantity_2.stl`: two wide-zone pads, **50 × 32 × 1 mm**.
  The finished compliant pad-plus-adhesive stack must total **1 mm**. A 1 mm
  printed pad plus additional adhesive changes both designed gaps. The same
  finished thickness can instead be obtained from accurately cut compliant sheet.
- `cad/left_body.step`, `right_body.step`, `contact_pad_primary.step` and
  `contact_pad_wide.step`: editable solids
  in the original actuator coordinate frame, rather than print coordinates.
- `cad/assembly_closed.step` and `assembly_open.step`: the complete geometric
  assembly, including the organizer's original actuator body and stock jaws.
- `dimensions.json` and `build_cad.py`: editable parameters and the CadQuery
  generator. `assembly_preview.png` and `dimensions.png` are exact CAD views.

The bodies alone **do not** have the intended gripping gaps: the four finished
1 mm contact pads are part of the design. Pad retention, material, print process,
air pressure and gripping force must be chosen and checked with the lab. This
package does not specify a rated polymer, infill, torque or pneumatic pressure.

## Grip geometry

| Feature | Narrow zone | Wide zone |
|---|---:|---:|
| Gap fully closed, with pads | 5.2 mm | 18.0 mm |
| Gap fully open, with pads | 17.2 mm | 30.0 mm |
| Contact length along zone-offset axis | 24 mm, flat | 50 mm, flat |
| Intended objects | M6 shaft; 9–16 mm cubes | Cap plug; Ø27.18 mm vial |
| Modeled entry clearance per side | 0.5 mm minimum | 1.0 mm minimum |
| Modeled squeeze allowance per side | 0.3 mm | 0.3 mm |
| Usable modeled width interval with those margins | 5.8–16.2 mm | 18.6–28.0 mm |

The two zones are **40 mm apart** along actuator STEP +X. Both fingertips end at
**Z = 83.5 mm**, nominally 76 mm beyond the stock CAD body's front plane at Z = 7.5.
The primary rail spans X = −21–3 mm (24 mm), retaining its original +3 mm edge
and extending away from the wide zone. The wide rail remains at X = 37–87 mm
(50 mm), leaving 34 mm between their nearest edges. The open exterior span across the closing axis
is 48.9 mm; collision screening must include that exterior, not just the pad gap.
The root bridge ends 37.7 mm above the fingertips. This clearance applies only
with the fingers directed down and does not constitute a full-arm collision model.

The real vial is **27.18 mm diameter × 203 mm long**. The organizer explicitly
states that starter v0.5 uses a smaller 17 × 140 mm simulation vial. A 17 mm vial
is not covered by the above margin intervals; do not treat the old miniature
simulation result as validation of the real-vial grip.

For the long vial, using the wide zone at its centre would bring the inactive
narrow zone into the vial. The proposed physical plan grips **81.5 mm
from the vial centre** (20 mm in from the end), with the narrow zone a further 40 mm
toward that same end. This leaves 17 mm nominal axial clearance from the narrow
rail's nearest edge to the 203 mm vial end. In V2.1 the 50 mm wide pad covers
34.5–84.5 mm from the vial centre, so friction along the pad resists the vial's
weight twisting it out of the grip. It is a geometric proposal; stable
retention and the actual vial's end shape still require physical testing. A
centre grasp requires a different finger arrangement or larger rail separation.

Caps are grasped externally on the tapered plug, narrow end up, with fingertips
5 mm above the support. The 31.75 mm flange stays below the tips. Neither zone
opens wide enough to grasp the full flange with clearance. The source model's
plug diameter at that height is approximately 23.05 mm.

## Mounting to the actual gripper

The mounting interface was built against the organizer's **SMC MHZL2-16D2 STEP**,
not the decorative screw locations in the simulator. The D2 stock jaw tongue is
5 mm thick and contains two Ø3.4 mm through-holes per jaw, in the closing Y
direction, spaced **7 mm** along Z. Their STEP-frame centres are X = 0 and
Z = 13.8 / 20.8 mm. The printed holes are Ø3.6 mm clearance holes.

The printed plates sit on the **outside faces** of the stock tongues:
Y = ±18.45 mm fully open and ±12.45 mm fully closed. Each printed mounting plate
is 6 mm thick. Use through-fasteners with suitable metal nuts/retention; there
are no plastic threads in this design. The nominal material stack is 5 mm stock
jaw + 6 mm printed plate, before washers and nut. **M3 × 16 mm is a candidate
length to check against the actual head/washer/nut stack, not a verified BOM.**
Do not use a metal-jaw torque limit as a validated tightening torque for a print.

Print and fit the coupon before printing the complete pair. Confirm that the
installed gripper is the D2 stock interface shown by the official CAD and that
any existing adapter can be removed or accommodated. Check both screw access and
full jaw travel. The supplied files cannot establish the state of the installed
custom fingers, adapter, mounting faces or fasteners.

After installation, measure the physical primary TCP, the wide-zone offset and
the finished open/closed gaps. The old controller UTOOL1 cannot be assumed to
match this attachment merely because the nominal blade length is 76 mm.

## Checks actually completed

- One valid connected solid per printed body and coupon; positive volumes.
- Exported body, pad and coupon STLs have no boundary or non-manifold edges.
- Thirteen equally spaced jaw positions from fully closed to fully open were
  checked against the organizer's stock actuator and each corresponding jaw.
- No positive-volume intersection was found with those reference parts or the
  opposing printed finger at those sampled positions. Intended mounting faces
  touch without interpenetrating.
- `cad_validation.json` records the checks and the reference STEP SHA256.

These are geometric checks of the V2.2 candidate. No new 50-trial benchmark or
physical carry test has measured whether it improves screw retention. The
October 31 baseline remains 84/98 screws placed after retries (85.7%).

These checks do not validate loads, fatigue, pad adhesion, print anisotropy,
physical fit, full-arm clearance, object recognition or grasp success. No new
physical test is claimed. The packaged software keeps motion confirmation gates
separate from this geometric design.

## Regeneration

With CadQuery 2.7, NumPy and Matplotlib installed:

```bash
python build_cad.py --reference reference/SMC-MHZL2-16D2.step
python render_cad.py --reference reference/SMC-MHZL2-16D2.step
```

The supplied STEP files remain directly editable in compatible CAD tools even
without the generator. Changes to `dimensions.json` must also be reflected in
the simulation and physical gripper profile before a motion test.

## Sources and scope

- Organizer CAD and real-vial correction: https://hackathon.sidclab.org/#cad
- Official stock model supplied by the organizer:
  https://hackathon.sidclab.org/files/SMC-MHZL2-16D2.step
- Design lineage: Asher's `feature/fanuc-scan-sort` at `1fbc58b`, including
  `dual_zone.py`, `padded_fingers.py` and `scene_config.py`.

The reference STEP is included solely as the mounting and interference reference.
This is a proposed custom attachment; it is not an SMC-approved design.

