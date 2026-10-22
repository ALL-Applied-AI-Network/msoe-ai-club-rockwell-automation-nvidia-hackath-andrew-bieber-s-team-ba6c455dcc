# Mixed-object sorting update

## Separate destinations, batch scanning and automatic corrections (October 20, 2026)

The first screw test on ROSIE measured a **129.8 mm lift, pickup=True**.
The later screenshot shows `pickup=True released=True` for a screw and a
yellow cube. Andrew reports that all screws were sorted in that run.
The new batch mode below has host tests but still needs its own Isaac run.

Install this incremental update with its `install_update.py --sim-dir PATH`.
It updates six named files, backs up replaced files under `runs/update_backup_*`,
and preserves the existing scene builder, physical policy, calibration and assets.
Close the old Isaac window before starting another instance.

From `~/innovation-lab/team-repo/fanuc_er4ia_sim`, run with a GUI:

```bash
"$HOME/venvs/isaac-check/bin/python" -u vision_sort_objects.py \
  --execute --include-misc --misc-demo --fast --batch-scan \
  --watch --idle-seconds 60 --max-picks 0 \
  --screws 6 --seed 42 \
  --sam-python "$HOME/venvs/sam2-check/bin/python"
```

`--batch-scan` runs SAM once, queues the eligible targets, and attempts each
one once. Order remains screws, cubes, small misc, caps, then vials. Ordinary
failed grasps and IK failures do not restart SAM halfway through the batch.
A cheap fresh RGB comparison precedes each cached pickup. Changed targets
are deferred to the next full scan; other queued targets continue. The batch
assumes a mostly static table. These image checks cannot guarantee that every
movement or occlusion is detected.

When the queue is exhausted, the arm has returned home and a full scan checks
for remaining objects. That scan starts the next pass if targets are eligible.
Each type/location gets at most two attempts while it remains detected. There
is no retry of the same queued target twice within a batch. A detected hooked
object or failed motion tracking still stops the run.

With `--watch`, an empty/exhausted verification scan starts a 60-second idle
wait, then another scan. This is a delay after the last scan, not a fixed
wall-clock alarm or a parallel timer. Scans and movement run sequentially;
no idle scan interrupts a workflow and no missed timer events pile up.
Eligible objects can be in the pickup workspace OR in the wrong destination
area. Objects already in their own destination are left alone. Objects outside
all configured areas remain excluded. New arrivals during a batch wait for
its end.

Watch mode retires a location's retry history after two consecutive full scans
without that kind of object there, so later arrivals can use the same location.
A still-visible failed object is not retried forever. This is location-based
memory: replacing an object at the same spot before its absence is observed
will not necessarily reset it. A missed detection is not proof of removal.

`--max-picks 0` removes the overall attempt cap. For a finite first run, omit
`--watch` and use `--max-picks 50`. The cap counts attempts, and if reached it
ends the run before any further verification scan. Close the window or use
Ctrl+C to stop. “No eligible targets” does not prove the entire table is clear.
The `RESULT` line reports measured simulation pickup/release and cycle time.
New movable props added while Isaac is playing must be properly configured
rigid bodies. The optional `--misc-demo` provides one such example at startup.

### Separate destinations

Every recognized label now has its own destination in `sort_config.py`:

| Detected label | Destination | Base XY, metres |
|---|---|---|
| Red cube | red | (0.20, 0.25) |
| Blue cube | blue | (0.20, 0.00) |
| Yellow cube | yellow | (0.37, 0.34) |
| Screw | screws | (0.20, -0.25) |
| Cap | caps | (0.35, -0.35) |
| Vial | vials | (0.17, -0.43) |
| Accepted unfamiliar small object | misc | (0.17, 0.43) |

Red, blue and screws keep their previous positions. Yellow, caps, vials and
misc now have separate positions. Each zone uses a 65 mm centre tolerance;
the circles do not overlap. The centres are on the supplied tabletop and
within the configured 540 mm radial limit. These are initial simulation
targets: that geometric check is not an IK, collision or hardware validation.
Random screw placement avoids every active drop, so seed 42's positions can
change. No cups are added to the scene.

The new rule is label-specific: a red cube in the screw zone is picked up and
sent to red, while a screw in that same zone is ignored. This also applies to
the misc zone when enabled. Logs show `from screws -> red`, for example.
Unknown shapes still have to pass the misc graspability/depth checks below.

SAM now inspects the pickup workspace AND destination areas. The camera ROI
includes their bounds; if needed the virtual camera widens its lens and the
pixel/depth geometry uses the updated intrinsics. A custom ROI cannot hide
destinations. A supplied homography is never silently reused after reframing.
The wider view/ROI may increase scan cost or change small-object segmentation;
check the new overlays on ROSIE. Piles and occlusion can hide misplaced objects:
the sorter can only correct objects it can detect and grasp.

Misc is for unfamiliar **small, graspable objects**: footprint length 5–40 mm,
width 4–10 mm, aspect <=5, compact/confident masks, and measured top height
4–25 mm from the simulated RGB-D camera. Oversized or flat/invalid-depth masks
are rejected with a reason. Fragments overlapping a recognized object are
excluded. `--misc-demo` adds a purple 16 x 7 x 8 mm block to exercise this path.
This is a graspability filter, not a semantic guarantee for arbitrary clutter.

`--fast` makes these simulation-only changes:

- Without `--batch-scan`, reuse a full SAM scan for at most three successful picks. Capture a fresh
  image before every pick; if the next target's pixels changed, run SAM again.
  Refresh after failure, after caps/vials, or after exhausting the queue. Nearby
  candidates within 60 mm of a picked object are removed from the cached queue.
- Use CUDA autocast: BF16 when PyTorch reports support, otherwise FP16. CPU
  stays FP32. Keep 1920 x 1080 resolution, the 64 x 64 point grid and crop layer.
- Keep four prompts per batch and release SAM's unused CUDA cache after scans.
- Shorten approach/carry/home durations by 25%; retain descent, gripper, dwell,
  lift, lower and release timing. The arm still returns to a clear camera pose.

A new object elsewhere may wait until the next full scan. Cached picks assume
a mostly static table; omit `--batch-scan` and use `--rescan-every 1` when
objects are being disturbed. The two scan-mode flags cannot be combined.
For precision-related detection changes, add `--sam-precision fp32`.
Omit `--fast` for the earlier motion timing and FP32. Scan once per pick is
the default when neither `--fast` nor `--batch-scan` is present.
The report now records scan, planning, motion and cycle seconds. Measure these
on ROSIE; no full-system speedup or OOM fix has been established here.

Nineteen host tests pass, including the OpenCV memory-layout regression, misc
height/size filtering, known-object fragment rejection, cached-image checks and
queue invalidation, full-batch retries, deferred targets, and retry reset after
two absent scans. Routing checks cover all 49 combinations of seven labels
and seven zones: correct-zone objects stay, wrong-zone objects get their own
destination. USD construction of six screws plus the misc prop with the new
zones passed, as did expanded-lens pixel/world round trips without rendering.
New Isaac motion, GPU mixed precision and physical operation have not been run
in this environment. The physical policy retains its existing enable gates,
full scans and cap/vial pause. The shared configuration supplies the new
destinations, but batch/watch and wrong-zone correction are enabled only by the
Isaac runner. The physical policy still excludes all destination areas and has
no misc recognition. This update does not lower the 100 mm cup-release height
for the planned no-cup physical test.

Implementation references: [SAM 2 mixed-precision examples](https://github.com/facebookresearch/sam2),
[PyTorch autocast](https://docs.pytorch.org/docs/stable/amp.html).

After a successful ROSIE run, commit only the updated files from the sim folder:

```bash
git add -- sort_config.py sort_vision.py vision_sort_objects.py add_random_screws.py test_sorting.py README_MVP2.md
git commit -m "Add separate sort zones, batch scans, and misplaced-object correction"
git push origin feature/mvp2
```

Then pull `feature/mvp2` on Windows before making further edits there.

## Original setup and physical-client notes

Start with `vision_sort_objects.py` in your existing Isaac repo. It uses the
motion planner from your uploaded runner and adds image-based labels, random
screws, separate drop locations, and a maximum of two attempts per object location.

With misc disabled, the order is **screws → cubes → caps → vials**, with smaller footprints first
within each group. This is an explicit practical priority, not a claim that a
30 mm screw is shorter than a 10 mm cube. Every cycle takes a fresh image.

## Files and where they go

| File | Purpose / destination |
|---|---|
| `vision_sort_objects.py` | New runner beside your existing `cell.py`, `fk.py`, `ik.py`, and `scene_config.py` |
| `sort_vision.py` | Shared SAM worker and shape/color filtering; needed in both repos |
| `sort_config.py` | Pickup order, drops, class mapping, physical settings; needed in both repos |
| `add_random_screws.py`, `assets/` | Add screws to the Isaac scene |
| `policy.py` | Copy beside `run.py` in the organizer's physical client |
| `calibration/` | Geometry and camera values supplied in your message; for the physical client |
| `test_sorting.py` | Host checks; no robot connection |

You do not need to replace your existing `scene_config.py` or original demo.
This bundle still needs your repo's `cell.py`, `fk.py`, `ik.py` and its assets;
those files were not attached and are not recreated here.

## 1. Run the simulation preview

Copy the first five rows' relevant files into the Isaac repo. Use the same
Isaac Python environment you used for your previous MVP:

```bash
python vision_sort_objects.py --headless --screws 6 --seed 42
```

The SAM environment path defaults to the path from your uploaded script:
`~/venvs/sam2-check/bin/python`. If yours differs:

```bash
python vision_sort_objects.py --headless --screws 6 --seed 42 --sam-python /your/sam2/env/bin/python
```

The SAM environment needs the existing official `sam2` install, PyTorch,
`numpy`, `opencv-python-headless`, and `Pillow`. The Isaac interpreter needs
`numpy` and `Pillow`. Model loading uses your existing Hugging Face cache or
downloads `facebook/sam2.1-hiera-small` if absent. No retraining is required.
The new simulation camera covers the full pickup region at 1920 × 1080;
the worker crops to that region with a margin before segmenting. Returned
pixels are shifted back into the full image, so coordinate conversion stays
consistent. The physical detector uses the same crop/restore approach.

Open the newest `runs/vision_pick_*/cycle_00/targets.png` and `vision_pick_report.json`.
The numbered overlay shows the class and assigned drop. The JSON contains the
metric grasp positions; the overlay's cross marks the mask centroid, which can
be slightly different from the corrected metric grasp center for long objects.

Then test the new screws in simulation:

```bash
python vision_sort_objects.py --headless --execute --screws 6 --seed 42 --target-type screw
```

Then the mixed sequence:

```bash
python vision_sort_objects.py --headless --execute --screws 6 --seed 42 --max-picks 30
```

Use `--screws 0` to test the existing objects alone. Omit `--headless` for a window.
If a narrow screw receives no mask, first inspect `masks.png`; try
`--points-per-side 96` or a tighter `--roi x1,y1,x2,y2` before changing shape rules.
The SAM worker is supplied here because the original `vision/sam2_worker.py`
was not attached. It follows the same persistent-process pattern and adds
mask contours and pixel colors to the response.

Screws use the organizer's actual M6x30 STL. The physics model uses separate
shaft/head colliders, estimated 6 g mass, and estimated contact friction.
This validates the scene construction, not physical grip reliability.

## 2. Set the destinations

Edit `DROP_XY`, `MISC_DROP_XY`, and `BIN_FOR` in `sort_config.py` for this.
The current seven assignments are listed above. Keep their tolerance circles
separate so a correctly sorted object's destination is unambiguous.

Containers are not added to the simulation or visually located. Objects are
released 100 mm above the work surface at the configured XY positions. Before
a physical run, measure reachable container centers, rim clearance, the clear
scan pose, and the jaw orientation offset. These initial values have not been
physically taught or checked for collisions. A reach/height check is not a
collision planner.

**The vial needs a container that fits it.** Your sim config uses 140 × 17 mm
vials; the supplied real specification is 203 × 27.18 mm. A vial held horizontally
will not fit into the organizer's 35 × 35 mm bin pocket. Use a suitable larger
tray for the shared destination or a separate vial drop. The detector accepts
both sizes, but that does not make their grasp/release mechanics equivalent.

For the physical submission, include the actual bins/tray and their measured
positions in `submission/placement.yaml`. Its positions use **millimetres**;
e.g., the example bin centers above are `[[200,250],[200,0],[200,-250]]`.
The organizer's screw placement convention is its **head** position and yaw
pointing toward the tip; our detector and sim spawn report object centers.
Also describe custom fingers or a larger tray in `custom_hardware`.

## 3. Test the physical client

Copy `policy.py`, `sort_config.py`, `sort_vision.py`, and `calibration/` into the
organizer's client, next to `run.py`. If organizers have since supplied newer
calibration, use their updated files instead. The provided calibration values
were retained; comments/formatting were shortened.

The client already depends on NumPy and OpenCV. Add Pillow for image export:

```bash
uv add pillow
uv run run.py --mock
```

The mock uses synthetic images and initially reports no TCP. This policy's
`--mock` path sends a fixed local mock pose and then holds; it checks the wire
format only, and intentionally skips vision. It does not prove grasp success
or test the real trajectory. The host tests separately exercise motion sequencing.

For an actual saved **whole-frame table camera** image, set the existing SAM
interpreter and preview without any robot connection:

```bash
export SORT_SAM_PYTHON="$HOME/venvs/sam2-check/bin/python"
uv run policy.py --image table.png
```

In PowerShell, set the environment variable with:

```powershell
$env:SORT_SAM_PYTHON = "C:\path\to\sam2-env\Scripts\python.exe"
uv run policy.py --image table.png
```

For physical runs, use the organizer's supported client environment (their
published instructions specify Linux/macOS). Keep `PHYSICAL_EXECUTE = False`
for the first connected run: it saves an overlay and targets and holds the
current pose. During your approved slot, the normal command is `uv run run.py`.

After checking the real overlay and setting the measured scan/drop poses and
orientation, set `PHYSICAL_POSES_CONFIRMED = True` and `PHYSICAL_EXECUTE = True`.
Every TCP action uses base metres converted to UF0 millimetres once. Resized
image pixels are restored to the calibration's original resolution before
projection. Motion uses small absolute targets, <=100 per chunk, and waits
for the reported endpoint before proceeding. The gripper opens/closes at a
stationary target, rather than during descent/carry.

## What is still experimental

- SAM supplies masks; size/shape rules supply object labels. These have not
  been tuned on a real image of your screws, clear vials, mat, or lighting.
  Unknown shapes are skipped. Touching objects and partially visible objects
  may be missed or misclassified.
- Grasp heights are initial values from your existing setup. A new detector
  cannot fix a gripper that is too wide/narrow or catches a cap/vial.
- For physical small-object picks, seeing the source object still there
  triggers a return/open/rescan. Disappearance is not positive pickup proof;
  logs explicitly mark pickup and release as unverified.
- Physical cap/vial attempts pause after release/retreat for an operator check.
  If one remains hooked, clear it before starting another rollout. A new rollout
  resets the attempt history. Keeping this pause is appropriate until those
  releases work reliably; `PAUSE_AFTER_LARGE_OBJECT` controls it.
- Simulation body positions are used only to measure pickup/release and report
  errors. Normal object selection/classification uses image evidence. The
  optional `--target-body` is an explicit simulation diagnostic override.

## Checks completed here

- The original eight host tests covered calibration/image resizing, UF0 conversion, synthetic
  masks for all six labels, rotated-cube alignment, pick order, location-based
  retries/drop exclusion, action chunking/gripper timing, lag handling, and policy
  execution/hold behavior.
- USD generation passed for six seeded screws: rigid bodies, two colliders per
  screw, and the organizer mesh. Two seeds were checked for reproducible placement.
- The organizer's local mock accepted 17 actions with zero rejections in a short
  rollout. This was the format-only mock path described above.
- Python syntax checks passed. Full SAM inference, full Isaac physics/rendering,
  and the real robot were **not** run here; the GPU environment and your repo's
  `cell.py`/`fk.py`/`ik.py` were unavailable.

Run the host checks yourself with `python -m unittest -v test_sorting.py` in an
environment containing NumPy, OpenCV, and Pillow.

Sources: [organizer client](https://github.com/SIDCLab/PhysicalAI-Hackathon-2026),
[organizer screw STL](https://hackathon.sidclab.org/files/ISO10642-M6x30-countersunk-screw.stl),
[SAM 2](https://github.com/facebookresearch/sam2). The physical policy interface,
coordinate conventions, and calibration came from your supplied message.
