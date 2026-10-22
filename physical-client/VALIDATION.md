# Review and validation — October 22, 2026

This morning review started from the current saved prepared archive (version 1),
not surviving scratch state. The unmodified source archive remains untouched.
No ROSIE login, robot connection, real submission, booking or hardware motion
occurred. The public organizer repository was cloned without authentication.

## Pass 1 — static motion, perception and failure review

Reviewed base metres to UF0 millimetres, native/resized pixel conversion,
orientation, shape fitting, jaw width, target interpolation, controller lag,
settling, retries, preview gates and the proposed bare-table destinations.

- Current public organizer HEAD is ff7534e730f2b3db261c7f22d3d9b56a40c00eed.
  run.py, cell_client.py, mock_cell.py, calibration geometry, frames and cameras
  match it byte for byte. Evidence: audit/morning-official-comparison.json.
- Keep raw physical RGB. Calibration puts the base above the pickup region;
  resized pixels are restored once. No image flip was added.
- The existing off-source pickup verification, clearance-first transfers,
  all-or-nothing plan validation, controller box checks and bounded retries
  were retained. A new test establishes that an unsettled TCP times out,
  empties the queued descent and holds for the operator.
- Both motion gates remain False. Screws and cubes <=11 mm detected width
  remain the enabled kinds. 16 mm cubes, caps, vials and misc stay excluded.
  Whole-mask width can reject a screw head even when its shaft would fit.
- The proposed fingertip release is still 10 mm above bare table. Jaw yaw,
  actual clearance/contact and placement accuracy require operator checks.
  Source disappearance does not prove pickup; opening does not prove release.

## Pass 2 — clean installation and behavioral checks

- A new .venv was created from the materialized archive; uv sync --locked
  installed all nine locked host dependencies under CPython 3.12.14.
- Baseline: 38 tests passed. After this review: 44 tests passed.
  See audit/morning-baseline-setup.log and audit/morning-final-setup.log.
- New regression coverage: non-Snap uv selection; recovery from the Snap
  failure; repeated setup preserving an existing private request; settle
  timeout preventing queued descent; official submission serialization through
  an offline transport stub with request-file mode 0600; missing SAM interpreter;
  and CPU-only SAM rejection when --require-cuda is requested.
- The retained tests exercise synthetic masks at both image resolutions,
  orientation reversal detection, retry/release motion paths against the actual
  organizer TCP validator, all-or-nothing plans, width exclusions, watch mode,
  consecutive settling after lag, preview gates and request generation.
- Full revised setup passed: 44 tests, geometric preflight, projected camera
  diagram, and organizer network mock: 25 accepted actions, zero rejections.
  Mock images are synthetic and the policy deliberately bypasses SAM there.
- Python compilation and bash syntax checks passed.
- Real local SAM import check FAILED: ModuleNotFoundError: torch. This runtime
  has no access to ~/venvs/sam2-check on ROSIE. No weights or real inference ran.
  Evidence: audit/morning-sam-check.log. CPU/missing-interpreter unit fixtures
  test failure reporting only; they are not SAM installations.

## Pass 3 — interrupted setup, request and booking workflow

- Fixed setup_rosie.sh: it prefers ~/.local/physical-uv/bin/uv, otherwise
  accepts non-Snap uv or creates that private uv via a Python venv. It no longer
  blindly accepts the Snap executable that failed on ROSIE. It still stops
  immediately on dependency/check failure and does not alter system uv.
- Added preflight.py --sam --require-cuda so an allocated-GPU check cannot
  succeed merely by printing CUDA: False. Imports alone do not prove inference.
- Fault-injection tests passed for failed dependency sync followed by restart,
  existing private request preservation, and generation without remote submission.
  The real organizer submit serializer was also exercised with an offline stub;
  that is not validation by the live submission service.
- Official dashboard was freshly read October 22: generate/review YAML,
  --submit, use request ID for a 30-minute Calendar slot in Central time,
  await approval/status, run preview, then operator-verified supervised motion.
  https://hackathon.sidclab.org/ and its #calendar tab remain authoritative.
  No available slot or confirmed booking is claimed.
- MORNING_HANDOFF.md documents a fresh installation path, non-Snap PATH,
  external SAM interpreter, preview and deliberate enablement. The revised
  archive is verified from a fresh extraction before delivery.

## Still unverified

Installation on ROSIE; real GPU/SAM environment and weights; real segmentation;
measured full-frame calibration landmarks; TCP/jaw yaw; swept robot clearances;
pickup/contact/release; real controller lag; proposed no-cup destinations and
object/color availability. No collision planning or measured hardware success
is claimed. There is no known submission request ID or reserved slot.
