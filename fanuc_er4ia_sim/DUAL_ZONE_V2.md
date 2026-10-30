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

## All lab objects (October 30)

The spaced layout now contains every object on the organizer's CAD page
(https://hackathon.sidclab.org/#resources/cad-models) at real size:

| Object | Size | Count | Destination |
|---|---|---:|---|
| M6 × 30 countersunk screw (ISO 10642) | 30 mm, head Ø12, shank Ø6 | 2 | `screw` area (0.135, −0.50), no walls |
| Cube | 10 mm | 6 | yellow / red / blue areas |
| Cube | 16 mm | 1 | `cube_large` spot (0.06, −0.50), no walls |
| Vial | Ø27.18 × 203 mm | 2 | `vial` area |
| Vial cap | Ø31.75 flange, 15.7 mm | 2 | `cap` area |

Screws use two explicit convex colliders (cone head + shank) and are gripped on
the shank at mid-length with the narrow zone. Screws need `FINGER_SET=dual_v2`
(the original `dual` narrow zone closes to 8 mm), so the layout skips them
otherwise. The overhead scan fits a screw-shaped silhouette and checks each
cube against its own size. The 50 × 50 × 55 mm bins are not added yet.

Quick tests per object type, then the full sort:

```bash
FINGER_SET=dual_v2 "$HOME/venvs/isaac-check/bin/python" -u demo_scan_sort_spaced.py --only screw
FINGER_SET=dual_v2 "$HOME/venvs/isaac-check/bin/python" -u demo_scan_sort_spaced.py --only cube
FINGER_SET=dual_v2 "$HOME/venvs/isaac-check/bin/python" -u demo_scan_sort_spaced.py --only cap
FINGER_SET=dual_v2 "$HOME/venvs/isaac-check/bin/python" -u demo_scan_sort_spaced.py
```

## Real bins (step 3)

Destinations now use the cell's real bins (organizer CAD: 50 x 50 mm outside,
55 mm tall, 35 x 35 mm pocket 25 mm deep), built as a solid base plus four walls:

| Bin | Centre (m) | Receives |
|---|---|---|
| yellow | (0.16, -0.36) | 2 yellow 10 mm cubes |
| red | (0.26, -0.36) | 2 red 10 mm cubes |
| blue | (0.36, -0.36) | 2 blue 10 mm cubes + the 16 mm cube |
| screw | (0.13, -0.48) | 2 M6 screws |
| cap_a, cap_b | (0.38, 0.33), (0.46, 0.30) | 1 cap each (Ø31.75 flange in a 35 mm pocket) |

Vials (203 mm) do not fit a pocket and keep the walled vial tray. The gripper
cannot reach into a 35 mm pocket (open fingers ~49 mm across), so it releases
with the fingertips 12 mm above the rim; the object drops ~30 mm to the pocket
floor. A binned object counts as placed when it rests anywhere inside the right
pocket, and later scans skip it (bin walls can hide it; the bin is the obstacle).
`SORT_BINS=0` restores the earlier floor areas.

Host checks: all 13 objects plan into their bins clear of all bins and walls;
fingers stay 12 mm and held objects at least 7 mm above the rim at release; the
parked arm stays at least 80 mm clear of every bin for the scan. Not yet run
in Isaac: whether every drop lands in its pocket (caps have 1.6 mm per side).

## Speed profile

Default timing trims time where nothing is held or nothing can slip:
free-space moves (approach, retreat, home) at 60%; cube/screw carries at up to
0.3 m/s^2 (caps and vials stay at 0.1); shorter dwell, release and settle
(bin drop settle 1 s, vial settle 3 s); 3 scan samples 1/3 s apart instead of
5 samples 1/2 s apart; 4 s start-up wait. Planned simulated time for the
13-object sort: 6.2 -> 4.5 min. `--careful` restores the earlier timing.
The run ends with sorting time and wall-clock seconds per simulated second.
For timing runs that need no video, `--headless` (no viewport) is much faster;
`--viewport-fps 10` is a middle ground.

## Recovery after a drop or miss

A failed pick no longer ends the run. If an object drops, never lifts, is
nudged before the fingers close, or misses its slot, the arm opens its jaws,
rises 100 mm, returns to the scan pose, rescans and plans a fresh pick from
wherever the object landed. Each object may fail `--max-recoveries` times
(default 3). After that it is reported failed and the sort continues with the
other objects. Sorted objects that get knocked out of their slot are re-queued.
The console shows `[recover]` lines; `sort_report.json` lists `recoveries` and
`failed_objects`, and the last line reports `N/M objects placed; K recoveries`.

To see it work, `--force-drop` opens the jaws halfway through the first
object's carry; the arm should back off, rescan and pick it up again:

```bash
FINGER_SET=dual_v2 "$HOME/venvs/isaac-check/bin/python" -u demo_scan_sort_spaced.py --only cube --force-drop
```

Failures are detected with simulator poses (evaluation only); the retry target
always comes from a fresh camera scan. On the physical cell, detection would
need a camera check after each placement instead.

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
