#!/usr/bin/env python3
"""Unattended robustness benchmark: N randomized, headless full-sort trials.

Each trial is a fresh Isaac process running demo_scan_sort_spaced.py with a
random planner-checked layout (--layout-seed) and physics variation
(--physics-seed), camera verification and drop recovery on. One JSON line per
trial is appended to <out>/results.jsonl as soon as it finishes, and seeds
already recorded are skipped, so the benchmark can be stopped and resumed.

    python3 run_benchmark.py --trials 50
    python3 run_benchmark.py --trials 3 --out runs/bench_pilot     # pilot
    python3 bench_summary.py runs/benchmark                          # results

Standard library only; run it with any Python 3 (not the Isaac interpreter).
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent


def summarize(seed, report, status, wall_s, returncode):
    objects = []
    for item in report.get('objects', []):
        objects.append(dict(object=item.get('object'), index=item.get('index'), group=item.get('group'),
                            attempt=item.get('attempt'), complete=item.get('complete'),
                            camera_verified=item.get('camera_verified'),
                            gpu_memory_mb=item.get('gpu_memory_mb'),
                            sim_verified=item.get('sim_verified'), cycle_s=item.get('cycle_s'),
                            reason=item.get('camera_reason') or item.get('error')))
    bench = report.get('benchmark') or {}
    return dict(seed=seed, status=status, returncode=returncode, wall_s=round(wall_s, 1),
                placed=(report.get('placed_objects') if report.get('placed_objects') is not None
                        else len({(o['object'], o['index']) for o in objects if o['complete']}) if report else None),
                expected=report.get('expected_objects'),
                complete=bool(report.get('complete')), failed_objects=report.get('failed_objects', []),
                recoveries=report.get('recoveries', []),
                camera_sim_agreement=report.get('camera_sim_agreement'),
                sim_final_audit_ok=report.get('sim_final_audit_ok'),
                sort_seconds=report.get('sort_seconds'), performance=report.get('performance'),
                layout_attempt=bench.get('layout_attempt'), physics=bench.get('physics'),
                error=report.get('error'), objects=objects)


def explain_crash(console):
    """Print the per-pick GPU memory readings and the first fatal error from a trial log."""
    try:
        lines = console.read_text(errors='replace').splitlines()
    except OSError:
        return
    memory = [l.split('GPU memory in use')[-1].strip() for l in lines if 'GPU memory in use' in l]
    print(f"[bench]   GPU memory after each pick: {', '.join(memory) or 'none logged'}", flush=True)
    keys = ('out of gpu memory', 'out_of_device_memory', 'fatal', 'traceback', 'error')
    first = next((l for l in lines if any(k in l.lower() for k in keys[:3])), None)
    first = first or next((l for l in lines if any(k in l.lower() for k in keys[3:])), None)
    if first:
        print(f'[bench]   first error: {first.strip()[:200]}', flush=True)
    print(f'[bench]   last line: {lines[-1].strip()[:200] if lines else "(empty log)"}', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--trials', type=int, default=50)
    parser.add_argument('--start-seed', type=int, default=1000)
    parser.add_argument('--out', type=Path, default=HERE/'runs'/'benchmark')
    parser.add_argument('--isaac-python', default=os.path.expanduser('~/venvs/isaac-check/bin/python'))
    parser.add_argument('--timeout', type=float, default=1800, help='Seconds before a trial is killed')
    parser.add_argument('--no-physics-variation', action='store_true')
    parser.add_argument('--gui', action='store_true',
                        help='Run trials with a small 10 fps viewport instead of headless (needs a desktop)')
    parser.add_argument('--retry-crashed', action='store_true', help='Re-run seeds that crashed or timed out')
    parser.add_argument('extra', nargs='*', help='Extra arguments for the sorter (after --)')
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    results = args.out/'results.jsonl'
    done = {}
    if results.exists():
        for line in results.read_text().splitlines():
            if line.strip():
                row = json.loads(line)
                done[row['seed']] = row['status']
    seeds = [s for s in range(args.start_seed, args.start_seed+args.trials)
             if s not in done or (args.retry_crashed and done[s] in ('crash', 'timeout', 'error'))]
    try:
        commit = subprocess.run(['git', 'log', '--oneline', '-1'], cwd=HERE, capture_output=True,
                                text=True, timeout=10).stdout.strip()
    except Exception:
        commit = 'unknown'
    print(f"[bench] code: {commit or 'unknown'}; mode: {'viewport 10 fps' if args.gui else 'headless'}",
          flush=True)
    print(f'[bench] {len(seeds)} trials to run ({args.trials-len(seeds)} already recorded) -> {results}',
          flush=True)
    durations = []
    for number, seed in enumerate(seeds, 1):
        run_dir = args.out/f'trial_{seed:05d}'
        run_dir.mkdir(parents=True, exist_ok=True)
        cmd = [args.isaac_python, '-u', str(HERE/'demo_scan_sort_spaced.py'),
               *(['--viewport-fps', '10'] if args.gui else ['--headless']), '--exit-on-finish',
               '--layout-seed', str(seed), '--run-dir', str(run_dir)]
        if not args.no_physics_variation:
            cmd += ['--physics-seed', str(seed)]
        cmd += args.extra
        env = dict(os.environ, FINGER_SET=os.environ.get('FINGER_SET', 'dual_v2'))
        started = time.monotonic()
        status, returncode = 'crash', None
        with open(run_dir/'console.log', 'w') as log:
            try:
                returncode = subprocess.run(cmd, cwd=HERE, env=env, stdout=log, stderr=subprocess.STDOUT,
                                            timeout=args.timeout).returncode
            except subprocess.TimeoutExpired:
                status = 'timeout'
        wall = time.monotonic()-started
        report_path = run_dir/'sort_report.json'
        report = json.loads(report_path.read_text()) if report_path.exists() else {}
        if status != 'timeout' and report:
            # Isaac can exit 0 after an exception: judge by the report, not the exit code.
            if report.get('placed_objects') is not None:
                status = 'pass' if report.get('complete') else 'fail'   # finished the sort
            elif report.get('error'):
                status = 'error'   # sorter stopped with an error message (in the report)
            else:
                status = 'crash'   # process died mid-run (see console.log)
        row = summarize(seed, report, status, wall, returncode)
        with open(results, 'a') as out:
            out.write(json.dumps(row)+'\n')
        durations.append(wall)
        left = (len(seeds)-number)*sum(durations)/len(durations)
        print(f"[bench] trial {number}/{len(seeds)} seed {seed}: {status.upper()} "
              f"{row['placed']}/{row['expected']} placed, {len(row['recoveries'])} recoveries, "
              f"{wall/60:.1f} min; about {left/3600:.1f} h left", flush=True)
        if status in ('crash', 'timeout', 'error'):
            explain_crash(run_dir/'console.log')
    print(f'[bench] Done. Summary: python3 {HERE/"bench_summary.py"} {args.out}', flush=True)


if __name__ == '__main__':
    sys.exit(main())
