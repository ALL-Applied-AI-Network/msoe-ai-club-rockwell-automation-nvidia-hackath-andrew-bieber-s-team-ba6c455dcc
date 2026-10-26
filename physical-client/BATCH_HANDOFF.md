# Physical batch sorting update

This update starts from the physical MVP saved as `91ecca7` on `feature/mvp2`.
Andrew reported successful physical cube and screw sorting on October 22,
including one screw alignment failure. No measured success rate is claimed.
The new batch motion workflow has host and synthetic-image validation only;
it has not yet been tested on the physical robot.

## Behavior

1. Move to the existing clear scan pose and run SAM2 once.
2. Save each eligible cube/screw's XY, yaw and destination.
3. Before each queued pick, compare its image region with the original frame.
   This uses fresh RGB and CPU arithmetic, with no SAM inference. The arm
   returns to the same clear scan pose between picks for this comparison.
4. Attempt each unchanged target once, then carry and release at a clear
   landing position without another source-verification SAM pass.
5. Once the queue is drained, run SAM again. Remaining eligible objects get
   another pass, within the existing two-attempt location budget and global
   20-attempt limit. A final scan is saved even when that global limit is reached.

Changed targets are deferred to the next full scan without consuming a grasp
attempt. Two consecutive passes with only deferrals cause a hold for inspection.
Watch mode remains off; when enabled, empty waiting scans do not consume this
deferral budget. A final scan with no eligible targets means no eligible objects
were detected; it is not a measured success score or proof that releases worked.

Camera-estimated width cannot veto the operator-confirmed fitting cube/screw
sizes. Finite positive width, recognized kind, TCP bounds, height/reach limits,
interpolation, settling checks and large-object pauses remain enforced. Other
sizes have not been declared physically graspable. Batch mode supports cubes
and screws only; use `PHYSICAL_SCAN_MODE = "per_pick"` for the original workflow.

Landing spots retain 40 mm spacing and stay within the configured zones and
TCP bounds. Correctly sorted recognized objects still block their occupied
positions, and commanded releases conservatively reserve their landing spots
for the rest of the rollout. A 35 mm centre-clearance check is a simple local
guard, not collision planning. A full destination causes a hold.

## Install and verify on ROSIE

Keep the physical working baseline commit. Extract this UPDATE archive into
a separate directory, then run its `apply_update.py` against the repository's
`physical-client` folder. The installer checks original file hashes, backs up
replaced files under `runs/before_batch_update_*`, and writes only listed source,
tests, documentation and checksums. It does not touch private request files,
saved runs, dependencies, submission identity, or robot connection settings.

```bash
export PATH="$HOME/.local/physical-uv/bin:$PATH"
cd ~/innovation-lab/team-repo/physical-client
uv run --locked python -m unittest -v test_sorting.py test_physical.py test_workflow.py test_batch.py test_batch_watch.py
uv run --locked preflight.py
```

These checks need no GPU or robot. The organizer mock still bypasses SAM and
the motion policy, so its acceptance counts are protocol evidence only.
Review and commit the changed files to `feature/mvp2` separately from `91ecca7`.

For a future approved physical test, apply the same update to the existing
approved client folder only while that client is stopped. Reuse its request.
Both motion gates in this update are **False**: first check a real preview and
the new behavior with the operator, then enable them deliberately in that
specific client folder. Compare `batch` and `per_pick` on the same object layout.

## Limits and measurements

No model, mask-generator, precision or GPU batch settings were changed.
This update reduces the number of SAM calls; it does not establish a faster
individual inference or eliminate T4 allocation warnings. The lab's roughly
40-second scan time is an observation from the previous configuration.

Capture scan counts, scan timing, total elapsed time, attempts, misses,
correct final placements and interventions. State-machine tests with three
objects demonstrate two SAM calls, versus the original before/after picking
workflow. Misses are found at the end of the batch rather than immediately.
RGB checks can also defer objects because of lighting, shadows or occlusion;
they cannot guarantee every displacement is detected. Logs continue to mark
`pickup_verified=False` and `release_verified=False`.

See `VALIDATION.md` for this update's actual checks and retained historical audit.
