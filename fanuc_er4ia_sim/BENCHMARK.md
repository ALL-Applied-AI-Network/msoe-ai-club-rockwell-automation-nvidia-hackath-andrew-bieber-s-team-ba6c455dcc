# Robustness benchmark

Unattended, headless full-sort trials on randomized setups. Each trial:

- **Layout:** all 13 lab objects (2 M6 screws, 6 x 10 mm cubes, 1 x 16 mm cube,
  2 vials, 2 caps) at random positions and angles in the reachable pick area,
  clear of the bins, the vial tray and the parked arm. A layout is used only if
  every object passes the same IK and clearance check the sorter runs.
- **Physics:** object friction and each object type's mass scaled by a random
  0.8-1.2x.
- **Sorting:** camera verification and drop recovery on (defaults).

The seed fixes both, so any trial can be replayed exactly.

## Run on Rosie

From `fanuc_er4ia_sim/`, with any Python 3 (the runner starts Isaac itself):

```bash
# Pilot: 3 trials, to measure time per trial and catch problems (~30 min)
python3 run_benchmark.py --trials 3 --out runs/bench_pilot
python3 bench_summary.py runs/bench_pilot

# Main benchmark, unattended (~8-9 min per trial; 50 trials ~7.5 h)
nohup python3 run_benchmark.py --trials 50 > runs/benchmark.log 2>&1 &
tail -f runs/benchmark.log
python3 bench_summary.py runs/benchmark
```

Results are saved after every trial (`runs/benchmark/results.jsonl`); rerunning
the same command skips finished seeds, so a session that ends early loses at
most one trial. `--retry-crashed` re-runs crashed or timed-out seeds. Each
trial's console log and reports are in `runs/benchmark/trial_<seed>/`.

Replay one trial with the viewport:

```bash
FINGER_SET=dual_v2 "$HOME/venvs/isaac-check/bin/python" -u demo_scan_sort_spaced.py \
  --layout-seed 1007 --physics-seed 1007
```

## What the summary reports

Full-sort success rate with a 95% confidence interval; objects placed; pick
attempts, first-try success and final placement per object type; recoveries and
their reasons; camera-vs-simulator agreement on every pick check; time per pick,
per full sort and per trial; and every trial that did not fully pass, with its
seed.
