# Physical dual-zone integration

This branch starts from Andrew's batch MVP `732f43b` and incorporates Asher's
`1fbc58b` simulator work. Keep `feature/mvp2` as the original updated MVP and
use **`feature/mvp2-dual-zone`** for this integration. The approved live client
folder is a separate installation; this package does not update it or submit
a robot request.

## Select the hardware explicitly

| `SORT_GRIPPER_PROFILE` | Hardware and scope |
|---|---|
| `legacy` (default) | Existing lab fingers; original cube/screw width override and batch recovery |
| `dual_zone` | Asher's original 8–20 / 14–26 mm prototype, including his smaller 17 × 140 mm vial; existing shaft grasp disabled |
| `dual_zone_v2` | Revised printable 5.2–17.2 / 18–30 mm fingers; M6 shaft, 9–16 mm cubes, cap plug, real 27.18 × 203 mm vial |

The revised CAD, mounting coupon, STEP assemblies and printer-oriented STLs
are in `../hardware/dual_zone_v2/`. The four finished 1 mm compliant pads are
required parts of the specified gap geometry. Print and fit the mounting
coupon first. The design is matched geometrically to the organizer's stock
SMC MHZL2-16D2 STEP, including its 7 mm mounting-hole pitch. It has not been
mounted, load tested or used to grasp objects on the physical robot.

The original Asher fingers can potentially grip a suitably sized screw head,
but that is a different grasp from our shaft-centre target. V2 changes the
small-zone geometry so the existing shaft target fits the modeled gap.

## Physical control behavior

SAM2 still segments the actual RGB image. Colour and geometric rules classify
objects; no simulator identities or true object positions enter this client.
The camera remains in its original physical orientation. Calibration and the
UF0 conversion are retained. Vial projection uses the configured physical
diameter instead of deriving its height from the distorted mask width.

In batch mode, one SAM call supplies a queue and obstacle estimates. Fresh RGB
checks run before cached picks without SAM, including the inactive rail and
landing footprints. The planner tries opposite approaches (also perpendicular
ones for a round cap), screens an entire pickup and delivery before publishing
the pickup, and defers blocked targets while trying other queued objects.
After the pass it rescans and retries within the existing two-attempt location
budget and global 20-attempt limit. A final scan still runs at the limit.

The narrow zone handles all configured 9–16 mm cubes, so the camera's incorrect
upper-width estimate does not choose a different rail or veto an otherwise
known-fitting cube. Finite positive widths, classification, inventory limits,
motion bounds and clearance checks still apply. Oversized silhouettes can
conservatively block a neighbouring grasp; this is distinct from a jaw-width
veto. Unknown sizes are not established as graspable by an override.

Caps use the wide zone 40 mm from the primary contact, with their narrow end
up. The fingertips contact the tapered plug 5 mm above the table, above its
31.75 mm flange. RGB classification does not establish cap orientation.

The real vial is grasped 81.5 mm from its centre, 20 mm in from an end. This
keeps the inactive small rail past that end. Object centre, gripping point and
controller TCP are separate coordinates; both pickup and release include the
offset. The nominal primary rail clears the vial end by 17 mm. Near-end
retention, torque and slip have not been physically validated.

The controller receives binary `0=open`, `1=close`; simulation's analogue jaw
positions are never sent as physical gripper fractions. New-profile releases
return to the per-kind contact height (screw/cube 2.5 mm, vial 3 mm, cap 5 mm).
Legacy mode retains its 10 mm release setting. Pickup/release logs remain
unverified; absence in a final image is not a measured success score.

## Local checks on ROSIE

```bash
cd ~/innovation-lab/mvp2-dual-zone/physical-client
export PATH="$HOME/.local/physical-uv/bin:$PATH"
export SORT_SAM_PYTHON="$HOME/venvs/sam2-check/bin/python"
SORT_GRIPPER_PROFILE=legacy uv run --locked python -m unittest -v \
  test_sorting.py test_physical.py test_workflow.py test_batch.py test_batch_watch.py \
  test_gripper_profile.py test_dual_clearance.py test_dual_policy.py test_dual_pixels.py
SORT_GRIPPER_PROFILE=dual_zone_v2 uv run --locked preflight.py
```

These checks need no GPU, robot or SAM weights. To inspect a saved whole real
table frame on a GPU allocation:

```bash
SORT_GRIPPER_PROFILE=dual_zone_v2 uv run policy.py --image /path/to/table.png
```

The actual revised simulator uses `FINGER_SET=dual_v2` (a different variable
from the physical selector). From `fanuc_er4ia_sim/`, run:

```bash
FINGER_SET=dual_v2 "$HOME/venvs/isaac-check/bin/python" -u demo_scan_sort_spaced.py --check-only
```

Its ten-object demonstration still scans before each pickup and uses simulator
semantic labels. It contains six 10 mm cubes, two vials and two caps. Screw
and 16 mm cube support in that demo is fit arithmetic, not a simulated pickup.
See `../fanuc_er4ia_sim/DUAL_ZONE_V2.md` for the profile comparison.

## Before a later hardware session

All motion flags remain false. Measure the mounted primary contact relative
to the controller's existing UTOOL1 and set `DUAL_ZONE_PRIMARY_OFFSET_TCP_M`.
Measure the wide-zone offset in that same TCP frame; the proposed `(0.040,0,0)`
is not a measured axis mapping. Its direction and `TCP_R_FOR_AXIS_X` must agree
modulo 180 degrees; code rejects inconsistent settings.

Confirm the finished pad gaps, actual cube/screw/vial dimensions and cap
orientation. Configure real landing areas and any walls/stops in
`DUAL_ZONE_STATIC_OBSTACLES` (metre `bounds_xy`, `height_m`, optional `base_z_m`).
The simulator's enlarged vial bin and anti-roll stops do not exist physically
merely because they are present in Isaac. The physical defaults remain the
previous proposed bare-table zones and need review for the longer vial.

The custom mount, inventory and layout each have a separate confirmation flag
in `sort_config.py`, in addition to `PHYSICAL_EXECUTE` and
`PHYSICAL_POSES_CONFIRMED`. `DUAL_ZONE_CAP_NARROW_END_UP` is separate because
the current recognizer cannot verify that orientation. These flags document
checks of the actual installed arrangement; changing them is not calibration.

The previous submission describes no custom hardware and cubes/screws only.
A new approved setup must describe the printed fingers, added objects and
destinations. Existing request credentials and submission files are retained;
`preflight.py --submission` refuses to validate the old task under a dual
profile. No submission or physical motion is part of installation.

## Evidence and limits

Host tests cover legacy regression, geometry, actual physical action rows,
alternate approach bounds, bounded recovery, source-return payload clearance,
pixel changes near the inactive rail, and synthetic real-size vial projection.
The mock server validates transport only. SAM settings and T4 allocation
behavior were not optimized by this change.

The local clearance guard includes both 48.9 mm open outer rail envelopes,
possible carried objects, supplied detections and configured static obstacles.
It is restricted to upright tabletop obstacles up to 45 mm and vertical
low-height approaches. It does not establish full-arm clearance or cover
missed objects, depth uncertainty, moving clutter or physical finger deflection.
No new Isaac dynamics, loaded SAM inference or physical-arm test is claimed.

Source for the real-vial correction and stock CAD: https://hackathon.sidclab.org/#cad
