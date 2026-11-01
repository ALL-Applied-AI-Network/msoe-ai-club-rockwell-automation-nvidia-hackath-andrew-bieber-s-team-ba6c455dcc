# MVP2 physical client — separate dual-zone integration

Current branch: **`feature/mvp2-dual-zone`**, based on the batch MVP `732f43b`
and Asher's `1fbc58b`. Start with [DUAL_ZONE_HANDOFF.md](DUAL_ZONE_HANDOFF.md)
for the new printable fingers and physical port. `feature/mvp2` remains the
separate updated MVP. The default profile here is `legacy`; explicitly select
`SORT_GRIPPER_PROFILE=dual_zone_v2` for the revised hardware. Motion is disabled.

Prepared from the supplied ROSIE source snapshot after commit `a2feb54` and the
organizer client at `ff7534e`. This folder runs independently of Isaac Sim.
`run.py`, `cell_client.py` and `mock_cell.py` are unchanged organizer files.

The prior [BATCH_HANDOFF.md](BATCH_HANDOFF.md) records the batch-only update.
Commit `91ecca7` preserves the earlier lab-tested physical implementation;
`732f43b` adds the subsequently tested software batch changes. Neither the new
batch workflow nor the revised printed fingers has a new physical test yet.
The legacy installation and request instructions below also cover a new setup.

## What is ready

The policy reads the physical table camera, uses SAM2 masks plus the existing
shape/color rules, projects to robot-base coordinates, and returns UF0 TCP
actions. The enabled scope is screws and red, blue and yellow cubes in the
presented sizes confirmed to fit the fingers at the physical test.

`PHYSICAL_SCAN_MODE="batch"` is the default: one full SAM scan saves the target
poses, each cached target gets one attempt during that pass, and a full scan
at the end finds leftovers for a later retry. The final scan also runs when the
total attempt limit has been reached. A location has at most two attempts.
After each release the arm returns to the same clear scan pose. A cheap RGB
comparison checks the next target against that pass's reference image, without
running SAM. Changed targets wait for the next full scan while unchanged
neighbors can proceed. Repeated passes with no attempted targets stop for review.

Set `PHYSICAL_SCAN_MODE="per_pick"` to restore the earlier scan after lifting,
before dropping, and a new scan before each pickup. Both modes can correct
recognized objects in the wrong destination. Optional watch mode waits without
blocking `act()` and clears old retry history only after two absent full scans.
Miscellaneous-object depth checks are not enabled in this physical client.
SAM settings are unchanged; fewer SAM calls do not establish a faster individual
scan or remove the T4 memory allocation warnings.

All five supplied camera calibrations numerically match the organizer copy.
The table calibration already puts the robot base above the pickup workspace
in the image. **Keep the physical RGB image in its raw orientation. Do not add
a flip to match the simulation display.** Image resizing is accounted for once.
This is a source/numerical check, not validation against measured cell landmarks.

This is a separate physical configuration: the Isaac files are untouched.
The proposed no-cup release height is 10 mm above the table instead of 100 mm.
Real scan/drop poses, pickup/release heights, camera mapping and jaw alignment
still need checking at the cell. `PHYSICAL_EXECUTE=False` and
`PHYSICAL_POSES_CONFIRMED=False` remain the defaults.

## Install and check on ROSIE (Linux)

Extract the archive under `~/innovation-lab`, creating this `physical-client`
folder. After an interrupted install, use a NEW parent directory so partial files,
an earlier `.venv`, edited gates or a prior request cannot blend into this release.
In a ROSIE terminal after uploading the archive to your home folder:

```bash
mkdir -p ~/innovation-lab
physical_test_dir=$(mktemp -d "$HOME/innovation-lab/physical-test-XXXXXXXX")
tar -xzf ~/physical-client-ready.tar.gz -C "$physical_test_dir"
cd "$physical_test_dir/physical-client"
export PATH="$HOME/.local/physical-uv/bin:$PATH"
bash setup_rosie.sh && pwd  # continue only after success; save this directory
export SORT_SAM_PYTHON="$HOME/venvs/sam2-check/bin/python"
uv run preflight.py --sam --require-cuda
```

The host and mock checks do not require a GPU or the robot. Real SAM inference
uses the existing SAM environment and weights; use your allocated GPU session.
`uv` installs only the small physical-client dependencies in this folder.
No Isaac process is needed. Keep the client and its connection running during
the approved test slot; the robot server sends observations to this process.
If setup itself is interrupted, return to this same printed directory and rerun
`bash setup_rosie.sh`. `uv sync --locked` resumes dependency synchronization;
the setup script never submits or connects to the physical cell. If you already
submitted from an older folder, keep its `.request.json` private and reuse that
folder/request rather than submitting duplicates. Do not overwrite it with the archive.

The October 22 update handles ROSIE's Snap uv permission failure: setup prefers
`~/.local/physical-uv/bin/uv`, or installs it through a Python venv if uv is
missing or resolves to Snap. It leaves the cluster's Snap install untouched.
Export the PATH above again after a new SSH login or GPU allocation. A failed
setup is a stop; an email prompt from a later command does not prove installation.

The mock deliberately skips perception and exercises the organizer protocol.
Its “Ready for the real robot” message means format/connection checks passed.
Unit tests exercise synthetic perception, routing, geometry and motion states.
Neither check demonstrates physical success.
The standalone `SORT_SAM_PYTHON` interpreter must already have torch, SAM2, OpenCV,
NumPy and Pillow. `preflight.py --sam` only checks imports and reports CUDA availability;
`--require-cuda` additionally fails when CUDA is unavailable.
it does not load weights or run segmentation. Use a ROSIE allocated GPU session for
the saved-image inference and live preview; don't leave SAM running on a login node.

## Camera check

`camera_mapping.png` is a labeled projection on a blank background. It shows
the orientation and proposed regions; it is not a photograph or physical proof.
For a saved whole-frame real table image (never a cropped screenshot):

```bash
uv run camera_check.py --image table.png --output real_camera_mapping.png
uv run policy.py --image table.png
```

The second command uses SAM without any robot connection. Look under the
printed `runs/physical_*/cycle_000/` folder for `targets.png`, `targets.json`,
`camera_mapping.png` and the saved frame. Targets JSON separates detections
from objects currently allowed to move. Camera-estimated width was unreliable
in the physical test. `PHYSICAL_VERIFIED_FIT_KINDS=("screw", "cube")` bypasses
the configured 11 mm width upper bound for the presented sizes confirmed to fit.
It does not remove the finite positive width requirement, classification,
allowed-kind restrictions or pose bounds. The estimated dimensions remain in
the saved detections. Set the verified-fit tuple to `()` to restore the width
upper bound for every kind. This override does not establish that other cube or
screw sizes fit the fingers.

For measured accuracy, write a JSON list such as:

```json
[{"pixel":[640,400],"base_mm":[390,0]}]
```

Replace BOTH the pixel and base coordinates with measurements of the SAME
table landmark from the saved frame. Use at least four spread-out landmarks
to check the whole pickup region. Then:

```bash
uv run camera_check.py --image table.png --landmarks landmarks.json --output checked_mapping.png
```

Default tolerance is 3 mm and can be set with `--tolerance-mm`. This reports
errors; it does not silently recalibrate or enable movement. A real image may
look inverted relative to the simulation while still using the correct mapping.

## Prepare and request the 30-minute test

`bash request_test.sh` prompts for your real contact and GENERATES the files below;
it does not submit. Alternatively:

```bash
uv run prepare_submission.py --contact 'Andrew Bieber, YOUR_ACTUAL_EMAIL'
uv run preflight.py --submission
cat submission/submission.yaml
cat submission/placement.yaml
uv run run.py --submit
```

Replace `YOUR_ACTUAL_EMAIL` before running. Use `--task-id` with your assigned
team suffix if the organizer requires it. The request proposes four single-object
tests (screw, red cube, blue cube, yellow cube), followed by one mixed test.
It requires 10 mm cubes of those colors and marked bare-table zones; the
organizer must confirm availability and accept or revise that setup. There
are no cups, bins, cap/vial trials or custom-finger claims in this first request.
Screw placement is its HEAD XY and yaw toward its tip, per the organizer format.

Review the YAML before submission. The request is local until `--submit`.
That command returns a request ID and saves `.request.json` containing its
secret. Keep that file on ROSIE; do not paste it into chat or commit it.

Use the request ID at https://hackathon.sidclab.org/#calendar and choose an
available 30-minute slot. Booking requires organizer approval. Slots may
change. `uv run run.py --status` shows the request state.

## At the supervised test

Start with the gates false and `uv run run.py`: it writes a real camera overlay
and holds the reported pose. Inspect the overlay and verify measured positions,
clearances, fingertip height, jaw angle and the proposed drop layout with the
operator. To test another preview after changes, restart the client as directed
by the operator. Only then change both gates in `sort_config.py` to true.

The physical scope excludes caps/vials and miscellaneous objects from motion.
Preview still reports recognized caps/vials. Batch mode supports only cubes and
screws. Other kinds require `per_pick` mode and checked fingers and handling
before broadening `PHYSICAL_ALLOWED_KINDS`; cap/vial attempts retain the existing
operator pause. Misc would additionally need real, validated depth evidence.

Record every attempt and operator intervention. Source disappearance does not
prove a grasp, and a release command does not prove release: the JSONL event
log explicitly preserves those distinctions. Camera occlusion may hide a failed
pick. The motion checks enforce configured numerical bounds and endpoint settling;
they do not perform collision planning or establish gripper contact.
The local guards include the organizer's UF0 TCP action box, radial/height
limits and every interpolated row, and reject a complete plan before queuing if
any row is invalid. This guards command bounds, not the swept volume of the arm.
The known-fit width override above does not change jaw alignment or estimated
pickup positions. The jaw yaw offset still needs a real alignment check.

The default run ends its picking loop when no eligible targets remain, and holds
for operator scoring. Optional `PHYSICAL_WATCH=True` rescans after 60 seconds;
`MAX_PICKS` remains a cap on attempts (0 removes that cap). Do not increase the
first-test object count until the tabletop destinations and potential piles
have been checked. The supplied mixed layout has one object per destination.

See `ORGANIZER_README.md` and `calibration/FRAMES.md` for the original interface.
See [BATCH_HANDOFF.md](BATCH_HANDOFF.md) for the current handoff.
`MORNING_HANDOFF.md` preserves the October 22 morning instructions as history.
