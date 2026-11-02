# IRIS: Intelligent Robotic Identification and Sorting

IRIS is a vision-guided sorting system for the FANUC ER-4iA. It was developed for Dataset 3: Grasping and Sorting in the Rockwell Automation + NVIDIA Innovation Lab, Fall 2026.

[Watch the project video](https://www.youtube.com/watch?v=nz326F-zYiA)

## The problem

Conventional industrial robots repeat motions taught at fixed coordinates. That works while every part arrives in the expected pose, but a shifted, rotated, mixed-in, or dropped part can stop the process and require an operator to reset the cell or reteach the robot.

IRIS replaces fixed pickup coordinates with a camera-driven loop that locates supported objects, plans a collision-checked grasp, sorts each object to its destination, verifies the result, and rescans after a failure.

## Approach

### Camera-guided, closed-loop sorting

The benchmarked simulation parks the arm for an overhead scan before each pickup. Rendered masks provide each object's measured XY position and orientation; simulator semantic labels provide the object category. The selected pose goes through inverse-kinematics, object-clearance, and bin-wall checks before the robot moves. After each attempt, another overhead scan determines whether the object reached its destination. A failed check triggers a rescan and retry.

Simulator ground truth is retained for grading and diagnostics, not for correcting the commanded pickup pose. This benchmark is therefore camera-driven within the digital twin, but its category segmentation is simulation-assisted rather than learned RGB perception. A separate experimental RGB path in [`vision_sort_objects.py`](fanuc_er4ia_sim/vision_sort_objects.py) uses SAM 2.1 masks with color and metric-shape rules; the benchmark results below do not claim performance for that separate path.

### Dual-zone gripper

The custom finger design works within the SMC gripper's existing 6 mm travel per jaw. Each rigid finger has two fixed contact zones separated by 40 mm:

- A narrow zone with a 5.2–17.2 mm padded gap for M6 screw shafts and 9–16 mm cubes.
- A wide zone with an 18.0–30.0 mm padded gap for cap plugs and the approximately 27 mm vial.
- A 50 mm-long wide pad intended to resist the off-center vial's tendency to twist during a carry.

The design adds no powered linkage and no extra actuator stroke. Printable CAD, dimensions, validation checks, and assembly notes are in [`hardware/dual_zone_v2/`](hardware/dual_zone_v2/README.md). It is an engineering-fit prototype, not a hardware-validated end effector.

### Recovery and reproducible testing

Each benchmark seed fixes both the object layout and the physics variation, so a failed trial can be replayed. The sorter continues after a failed grasp, uses the overhead camera to find the object again, and retries up to its configured recovery limit. Results and logs are written after every trial so an interrupted benchmark can resume.

## Verified simulation results

On October 31, 2026, IRIS ran 50 randomized Isaac Sim 6.0.1 trials on an NVIDIA T4. Each trial contained 13 objects—two M6 screws, seven cubes, two vials, and two caps—with randomized positions and, where applicable, orientations. Object friction and each object type's mass were independently scaled between 0.8× and 1.2×. The complete evidence, plots, seeds, and failure analysis are in [`BENCHMARK_RESULTS.md`](fanuc_er4ia_sim/BENCHMARK_RESULTS.md).

| Metric | Verified result |
| --- | ---: |
| Full 13-object sorts completed | **37 of 50 (74%)** |
| Objects placed after retries | **623 of 637 (97.8%)** |
| Camera checks agreeing with simulator grading | **672 of 673** |
| Automatic recoveries | **81 recoveries across 25 trials** |
| Mean time for a completed full sort | **12.9 minutes** |

Results by object type:

| Object | Tested | Placed first try | Placed after retries |
| --- | ---: | ---: | ---: |
| Cubes | 343 | **343 (100.0%)** | **343 (100.0%)** |
| Vials | 98 | **98 (100.0%)** | **98 (100.0%)** |
| Caps | 98 | **98 (100.0%)** | **98 (100.0%)** |
| Screws | 98 | 69 (70.4%) | **84 (85.7%)** |

All metrics in these tables come from the checked-in [50-trial benchmark report](fanuc_er4ia_sim/BENCHMARK_RESULTS.md). Twelve trials ended incomplete after the sorter exhausted its screw retries, and one trial crashed before sorting. The recorded screw failure mode was a clean pickup followed by the round 6 mm shank slowly turning between flat pads and slipping out during the carry.

## Run the benchmark

The benchmark is the clearest reproducible path for the final project. It requires Linux, Python 3 with `venv`, a separate Isaac Sim installation, a compatible NVIDIA GPU, and access to the FANUC ER-4iA Isaac asset. Model weights and generated runs are not stored in this repository.

```bash
git clone <repository-url>
cd msoe-ai-club-rockwell-automation-nvidia-hackath-andrew-bieber-s-team-ba6c455dcc/fanuc_er4ia_sim

# Host dependencies, generated textures, and host-side tests
./bootstrap.sh

# Point this at the Python launcher for your Isaac Sim environment
export ISAAC_PYTHON=/path/to/isaac/python

# Three-trial pilot with a small viewport
python3 run_benchmark.py \
  --gui \
  --trials 3 \
  --out runs/bench_pilot \
  --isaac-python "$ISAAC_PYTHON"

python3 bench_summary.py runs/bench_pilot
```

To replay a particular randomized trial directly:

```bash
FINGER_SET=dual_v2 "$ISAAC_PYTHON" -u demo_scan_sort_spaced.py \
  --layout-seed 1007 \
  --physics-seed 1007
```

The October 31 run used the viewport mode because repeated headless trials on the T4 encountered a documented GPU-memory failure. See [`BENCHMARK.md`](fanuc_er4ia_sim/BENCHMARK.md) for the full benchmark procedure, resume behavior, output layout, and replay instructions. See [`SETUP.md`](fanuc_er4ia_sim/SETUP.md) for the broader Isaac Sim and optional model setup.

## Repository guide

| Path | Purpose |
| --- | --- |
| [`fanuc_er4ia_sim/demo_scan_sort.py`](fanuc_er4ia_sim/demo_scan_sort.py) | Camera-scan sorting loop, motion checks, verification, and recovery |
| [`fanuc_er4ia_sim/run_benchmark.py`](fanuc_er4ia_sim/run_benchmark.py) | Seeded multi-trial benchmark runner |
| [`fanuc_er4ia_sim/BENCHMARK_RESULTS.md`](fanuc_er4ia_sim/BENCHMARK_RESULTS.md) | Final metrics, plots, seeds, and failure analysis |
| [`fanuc_er4ia_sim/vision_sort_objects.py`](fanuc_er4ia_sim/vision_sort_objects.py) | Separate SAM 2.1 RGB mixed-object sorting path |
| [`hardware/dual_zone_v2/`](hardware/dual_zone_v2/README.md) | Printable dual-zone gripper CAD and engineering checks |
| [`physical-client/`](physical-client/) | Guarded physical-cell client and audit material; not evidence of a completed real-arm sort |

## Limitations

- The reported results are from Isaac Sim, not the physical FANUC arm. This repository does not establish a completed autonomous real-arm sort.
- The benchmark uses simulator semantic labels for object categories and rendered-mask geometry for pose estimation. Its 672-of-673 camera agreement result measures camera verification against simulator grading, not real-camera recognition accuracy.
- The dual-zone CAD passed the documented geometry and interference checks, but physical fit, pad adhesion, grip force, loads, fatigue, TCP calibration, and full-arm clearance still require supervised hardware testing.
- Screws remain the main reliability gap: 14 of 98 were not placed after retries. A longer narrow pad, a smaller minimum gap, or a V-groove is a documented next test.
- One of the 50 benchmark trials crashed before sorting, and full-sort success was 74% even though object-level placement was 97.8%.

## Team and credits

**Andrew Bieber's Team**

- Andrew Bieber — mentor and team lead
- Peter Gerrits
- Asher Jaenchen
- Kshitij Batra
- Zachary Millmann
- Evan Ohme
- Khalil Spires

Built for the **Rockwell Automation + NVIDIA Innovation Lab, Fall 2026**, with the MSOE AI Club and UWM Connected Systems Institute.

The project uses NVIDIA Isaac Sim and its FANUC ER-4iA asset for the digital twin. The mounting design references the organizer-supplied SMC MHZL2-16D2 STEP model. The separate experimental RGB perception path uses Meta's `facebook/sam2.1-hiera-small` model. The repository also retains the supplied starter scene, geometry, and task materials; provenance and measurements are documented in [`scene_config.py`](fanuc_er4ia_sim/scene_config.py) and the [dual-zone hardware notes](hardware/dual_zone_v2/README.md).

Event requirements and organizer links are retained in [`MSAIC_INSTRUCTIONS.md`](MSAIC_INSTRUCTIONS.md).
