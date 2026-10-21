# Mixed-object sorting update

Start with `vision_sort_objects.py` in your existing Isaac repo. It uses the
motion planner from your uploaded runner and adds image-based labels, random
screws, three drop locations, and a maximum of two attempts per object location.

The order is **screws → cubes → caps → vials**, with smaller footprints first
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

## 2. Set the three drops

Edit only `sort_config.py` for this:

| Drop | Initial base XY, metres | Initial contents |
|---|---|---|
| left | `(0.20, 0.25)` | Red cubes |
| middle | `(0.20, 0.00)` | Blue cubes |
| right | `(0.20, -0.25)` | Screws, yellow cubes, caps, vials |

This is a starting grouping: five requested categories cannot each have an
exclusive destination with only three cups. Your supplied scene also contains
yellow cubes, which are detected. Change `BIN_FOR` to group things differently,
or add drop entries for more containers.

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

- Eight host tests passed: calibration/image resizing, UF0 conversion, synthetic
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
