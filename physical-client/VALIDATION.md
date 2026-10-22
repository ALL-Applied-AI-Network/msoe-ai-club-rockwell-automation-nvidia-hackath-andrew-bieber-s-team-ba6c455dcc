# Review and validation — October 22, 2026

This report concerns the standalone physical client. The original ROSIE archive
and simulation repository were not edited. No account login, ROSIE connection,
physical cell connection, submission, booking or hardware motion was performed.

## Pass 1: independent static review

Reviewed base metres → UF0 mm, camera full-frame scaling and orientation,
metric mask geometry and jaw exclusions, interpolated motion, endpoint settling,
retry retirement, preview/enable gates and bare-table placement.

Findings and fixes:

- The published calibration already describes the reversed physical camera view.
  Keep raw RGB; adding a flip would apply an extra reversal. All five numeric
  calibrations match the supplied source. Current official calibration/geometry
  and run.py, cell_client.py, mock_cell.py match byte for byte (commit ff7534e,
  still origin/main when fetched for this review).
- The pickup check previously ran directly above the source. Now it lifts and
  moves to the configured scan point with grip closed and yaw preserved before
  checking for a miss. Failed-source return and destination approach cross at
  clearance, then descend vertically. Visibility still does not prove grasp success.
- Local radial/height bounds omitted the organizer TCP action box. The client
  now rejects targets outside its UF0 box before selection and checks all rows.
  A whole motion plan is built/validated before any prefix enters the queue.
- request_test.sh previously submitted immediately. It now generates and prints
  the request for review; only an explicit `uv run run.py --submit` sends it.
- No-cup release remains the proposed 10 mm above the table. Both motion gates
  remain false. Screws and 10 mm cubes are the initial scope; cap/vial/misc motion
  stays disabled. Width uses the whole mask and can conservatively reject a wide
  screw head even where the shaft would fit. No untested width expansion was made.

## Pass 2: fresh environment and behavioral tests

- Fresh extraction with no .venv; `uv sync --locked` installed the nine locked
  host packages successfully under CPython 3.12.14.
- Original baseline: 30 tests passed before modifications.
- Expanded suite: **38 tests passed** after fixes, including full synthetic
  missed-pick → fresh scan → retry → release, with every emitted chunk accepted
  by the organizer's actual validate_tcp function; off-source verification;
  clearance on failed-pick return; consecutive settling after lag; wrapped yaw;
  controller box/NaN bounds; all-or-nothing queuing; and existing camera, mask,
  routing, width, retry/watch, gates and submission tests.
- Python compile checks and `bash -n` on both shell helpers passed.
- `preflight.py --sam` was attempted locally and FAILED as expected: the clean
  host environment has no torch (and no ROSIE SAM environment). This is an
  outstanding environment prerequisite, not a passed inference check.
- Organizer network mock passed with no rejected actions; the final-package setup log under audit/ records its action count. This mock uses synthetic
  images and deliberately bypasses SAM; it establishes protocol, not grasp skill.

## Pass 3: recovery and submission workflow

- Offline shell tests injected a dependency-sync failure: setup exited at the
  failed sync, then restarted successfully. A preexisting private request file
  remained unchanged. Request generation made no submit/status/cell call.
- Real setup passed from the release archive in a second clean directory. SHA256
  manifest verification, clean dependency synchronization, all tests, preflight,
  camera projection generation and local mock are checked from that extraction.
- Request YAML generation/placement count and external SAM interpreter selection
  are checked locally; test contacts are fixtures only and never submitted.
- The live official dashboard was read October 22: --submit → request ID →
  30-minute Calendar slot (Central) → organizer approval → run/status. No current
  slot availability or reservation is claimed.

## Not verified here

ROSIE installation, existing SAM environment/GPU, downloaded weights, actual SAM
inference, real full-frame landmarks, physical TCP/jaw alignment, swept clearances,
contact, release, clutter and real controller response were not established.
No robot was connected or moved. Mathematical camera consistency and synthetic
tests do not establish hardware reliability. The first connected run stays preview-only.
