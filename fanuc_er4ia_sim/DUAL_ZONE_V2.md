# Dual-zone revision for the physical object dimensions

`FINGER_SET=dual` retains Asher's original prototype: an 8–20 mm primary
gap, a 14–26 mm secondary gap, and the original photo-scaled 17 × 140 mm vial.
The optional `dual_v2` profile uses the revised CAD contact geometry and the
organizer's 27.18 × 203 mm vial dimensions. Both profiles retain 6 mm travel
per jaw and a 40 mm offset between gripping zones.

| Property | `dual_v2` |
|---|---:|
| Primary closed/open gap | 5.2 / 17.2 mm |
| Secondary closed/open gap | 18 / 30 mm |
| Pad thickness | 1 mm |
| Rail outer face, measured from closed jaw centreline | 18.45 mm |
| Total open width across both jaws | 48.9 mm |
| Finger length / plate thickness | 76 / 6 mm |
| Root bridge length | 28.3 mm |
| Primary/secondary zone separation | 40 mm |

The primary zone can close on a nominal 6 mm screw shaft and admits measured
16 mm cubes with a 0.5 mm entry allowance per side when aligned to the jaws.
The secondary zone grips the real vial diameter or the cap's narrow plug.
It cannot pass around the cap's 31.75 mm flange; caps must stand narrow-end-up,
with the tips above the flange.

The vial's wider diameter would intersect the inactive primary fingers in a
centre grasp. The v2 plan therefore places the secondary contact 81.5 mm from
the vial centre, 20 mm in from its end. Its virtual object-centre tool offset
is 121.5 mm from the primary zone. The inactive primary rail and root bridge
then clear the nominal vial envelope. This offset is part of the grasp plan;
it does not introduce another actuator. Grip stability, friction and mounting
strength remain unverified by these numerical checks.

The v2 simulator also expands the vial enclosure to 270 × 180 mm at
(200, 390) mm, with slots at (200, 350) and (200, 430) mm. Anti-roll stops
have 2.5 mm nominal side clearance. These simulator fixtures are separate
from the physical client's table layout.

## Run on ROSIE

From `fanuc_er4ia_sim/`:

```bash
FINGER_SET=dual_v2 "$HOME/venvs/isaac-check/bin/python" -u demo_scan_sort_spaced.py --check-only
FINGER_SET=dual_v2 "$HOME/venvs/isaac-check/bin/python" -u demo_scan_sort_spaced.py
```

Add `--headless` for a run without the window. Use `FINGER_SET=dual` for
Asher's original dimensions and destination fixtures. A fit report needs
only NumPy:

```bash
FINGER_SET=dual_v2 python dual_zone.py
```

The demo still presents six 10 mm cubes, two vials and two caps. It scans before
each pickup. Screw support and the 16 mm cube are checked by the fit arithmetic;
they have not been added to this ten-object simulator demonstration.

## Vial grip fix (October 30)

Vials were dropping or not lifting. Changes:

- **Pad contact patch.** Finger pads and the vial collider now set a 3 mm
  PhysX torsional patch radius (`GRIP_PATCH_RADIUS`), matching half the 6 mm rail.
  Without it, rigid contacts barely resist the vial twisting about the closing
  axis. In `dual_v2`, the vial is gripped 81.5 mm off-centre and pivots out.
- **Grip force.** Vial pad squeeze is 0.6 mm (`VIAL_GRIP_SQUEEZE`), about
  24 N per finger instead of 12 N.
- **Gentler motion.** Vial close, dwell and lift are slower.
- **Diagnostics.** Each object report now records `grip_slip_mm` and
  `grip_rotation_deg` for lift, carry and lower. These show whether a vial slid,
  pivoted, or was never gripped.
- `python dual_zone.py` prints the static hold margins (slip and twist).
- **Faceted vial collider.** Isaac 6 simulates `UsdGeom.Cylinder` as an exact
  round shape, which touches a flat pad at one point and rolls freely. The vial
  now uses a 24-sided prism (flats at the real 27.18 mm), so pads grip
  face-to-face and the vial rests on a flat. With the round collider the vial
  lifted but pivoted 47° in the grip.
- **Long wide-zone pad (V2.1 hardware).** Even with the faceted collider the
  vial pivoted ~44° in 6 mm pads. The wide rail and pad now run 50 mm along the
  vial toward its centre (`dual_zone.WIDE_RAIL_LENGTH`; CAD in
  `hardware/dual_zone_v2`). Vial cradle stops moved to 16–24 mm from the slot
  centre to clear the pad, and the planner tries more pick/place finger
  directions so the long rail stays inside the sorting-area walls.
- **Scan/park and settling fixes.** The scan park pose no longer hides the vial,
  task objects never sleep, and pick-time handles no longer reset object poses.

Quick vial-only test (other objects stay in the scene as obstacles):

```bash
FINGER_SET=dual_v2 "$HOME/venvs/isaac-check/bin/python" -u demo_scan_sort_spaced.py --only vial
FINGER_SET=dual    "$HOME/venvs/isaac-check/bin/python" -u demo_scan_sort_spaced.py --only vial
```

Then run the full sort without `--only`. Physics behaviour is not yet confirmed
in Isaac; check `runs/scan_sort_*/sort_report.json` and the per-object JSON files.

## Validation boundary

The revised code passed 33 existing host tests and nine new profile tests.
Both `dual` and `dual_v2` passed all ten nominal numerical trajectories and
wall/object envelope checks, with reported tip error at most 0.100 mm. The new
tests also cover real-size rendered-mask fitting, actuator limits, invalid
widths, profile preservation, virtual tool offsets, inactive-finger/vial
clearance and vial-fixture dimensions.

No new Isaac dynamics or physical-robot run was performed in this integration
environment. Numerical checks do not verify friction, grasp retention, dynamic
release, physical mounting or full-arm collisions. Simulator identities still
come from semantic labels, XY/yaw come from rendered masks, and simulator
poses are used for evaluation. The physical client uses its separate camera
recognition and controller implementation.
