#!/usr/bin/env python3
"""Summarize a benchmark: python3 bench_summary.py runs/benchmark

Prints a Markdown report and writes <dir>/summary.json. Standard library only.
"""
import json
import math
from pathlib import Path
import statistics
import sys


def wilson(successes, n, z=1.96):
    """95% Wilson score interval for a success rate."""
    if n == 0:
        return (0.0, 0.0)
    p = successes/n
    centre = (p + z*z/(2*n))/(1 + z*z/n)
    half = z*math.sqrt(p*(1-p)/n + z*z/(4*n*n))/(1 + z*z/n)
    return (max(0.0, centre-half), min(1.0, centre+half))


def summarize(rows):
    trials = len(rows)
    passed = sum(r['status'] == 'pass' for r in rows)
    ran = [r for r in rows if r['status'] in ('pass', 'fail')]
    by_type = {}
    for r in ran:
        for o in r['objects']:
            t = by_type.setdefault(o['object'], dict(attempts=0, first_try=0, objects=0, placed=0))
            t['attempts'] += 1
            if o['attempt'] == 1:
                t['objects'] += 1
                t['first_try'] += bool(o['complete'])
            if o['complete']:
                t['placed'] += 1  # first try or after recovery
    agreement = [r['camera_sim_agreement'] for r in ran if r.get('camera_sim_agreement')]
    agree = sum(a['agree'] for a in agreement)
    checks = sum(a['checks'] for a in agreement)
    cycle = [o['cycle_s'] for r in ran for o in r['objects'] if o.get('cycle_s') is not None]
    sort_s = [r['sort_seconds'] for r in ran if r.get('sort_seconds')]
    wall = [r['wall_s'] for r in rows]
    factor = [r['performance']['wall_per_sim_second'] for r in ran
              if r.get('performance') and r['performance'].get('wall_per_sim_second')]
    placed = sum(r['placed'] or 0 for r in ran)
    expected = sum(r['expected'] or 0 for r in ran)
    low, high = wilson(passed, trials)
    return dict(
        trials=trials, passed=passed, pass_rate=passed/trials if trials else 0.0,
        pass_rate_95ci=[low, high],
        statuses={s: sum(r['status'] == s for r in rows) for s in ('pass', 'fail', 'crash', 'timeout')},
        objects_placed=placed, objects_expected=expected,
        object_placement_rate=placed/expected if expected else 0.0,
        by_type=by_type,
        recoveries_total=sum(len(r['recoveries']) for r in ran),
        trials_with_recovery=sum(bool(r['recoveries']) for r in ran),
        recovery_reasons=sorted({rec['reason'][:80] for r in ran for rec in r['recoveries']}),
        camera_sim_agreement=dict(agree=agree, checks=checks),
        mean_pick_seconds=statistics.mean(cycle) if cycle else None,
        mean_sort_minutes=statistics.mean(sort_s)/60 if sort_s else None,
        mean_trial_wall_minutes=statistics.mean(wall)/60 if wall else None,
        mean_wall_per_sim_second=statistics.mean(factor) if factor else None,
        problem_trials=[dict(seed=r['seed'], status=r['status'], placed=r['placed'],
                             failed_objects=r['failed_objects'], error=r.get('error'))
                        for r in rows if r['status'] != 'pass'])


def markdown(s):
    lines = [f"# Robustness benchmark: {s['trials']} randomized trials", '',
             f"- **Full sorts completed:** {s['passed']}/{s['trials']} "
             f"({100*s['pass_rate']:.0f}%, 95% CI {100*s['pass_rate_95ci'][0]:.0f}-{100*s['pass_rate_95ci'][1]:.0f}%)",
             f"- **Objects placed:** {s['objects_placed']}/{s['objects_expected']} "
             f"({100*s['object_placement_rate']:.1f}%)",
             f"- **Recoveries:** {s['recoveries_total']} in {s['trials_with_recovery']} trials",
             f"- **Camera vs simulator agreement:** {s['camera_sim_agreement']['agree']}/"
             f"{s['camera_sim_agreement']['checks']} pick checks"]
    if s['mean_pick_seconds']:
        lines.append(f"- **Time:** {s['mean_pick_seconds']:.0f} s per pick attempt, "
                     f"{s['mean_sort_minutes']:.1f} min per full sort, "
                     f"{s['mean_trial_wall_minutes']:.1f} min per trial incl. start-up "
                     f"({s['mean_wall_per_sim_second']:.2f} wall s per sim s)")
    lines += ['', '| Object | Pick attempts | First-try success | Placed |', '|---|---:|---:|---:|']
    for kind, t in sorted(s['by_type'].items()):
        first = f"{t['first_try']}/{t['objects']}" if t['objects'] else '-'
        lines.append(f"| {kind} | {t['attempts']} | {first} | {t['placed']}/{t['objects']} |")
    if s['recovery_reasons']:
        lines += ['', '**Recovery reasons:**'] + [f'- {r}' for r in s['recovery_reasons']]
    if s['problem_trials']:
        lines += ['', '**Trials that did not fully pass** (replay with `--layout-seed N --physics-seed N`):']
        lines += [f"- seed {p['seed']}: {p['status']}, placed {p['placed']}, gave up on "
                  f"{', '.join(p['failed_objects']) or '-'}{'; ' + p['error'][:120] if p.get('error') else ''}"
                  for p in s['problem_trials']]
    return '\n'.join(lines)


def main():
    directory = Path(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).resolve().parent/'runs'/'benchmark')
    rows = [json.loads(l) for l in (directory/'results.jsonl').read_text().splitlines() if l.strip()]
    latest = {}
    for row in rows:  # a re-run seed replaces its earlier row
        latest[row['seed']] = row
    summary = summarize(list(latest.values()))
    (directory/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(markdown(summary))


if __name__ == '__main__':
    main()
