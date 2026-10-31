#!/usr/bin/env python3
"""Where do screw picks fail? python3 screw_report.py runs/benchmark [--trace]

Reads every trial's sort_report.json and puts each screw attempt in one stage,
using the simulator's evaluation record (never used to command the robot):

  not lifted     the screw did not rise 25 mm with the fingers
  lost in carry  lifted, but not in the fingers above the bin at the end of the carry
  out of bin     reached the bin, but did not come to rest in the pocket
  placed         camera saw it in the bin

Also prints the in-grip rotation/slip per phase and the trial's random physics
factors, to see whether failures follow low friction or high mass.
Standard library only.
"""
import json
import math
from pathlib import Path
import statistics
import sys

SCREW_BIN = (0.13, -0.48)


def stage(o):
    if o.get('complete'):
        return 'placed'
    if not o.get('pickup_verified'):
        return 'not lifted'
    if not o.get('carry_verified'):
        return 'lost in carry'
    return 'out of bin'


def where(o):
    centre = o.get('final_object_centre_world_m')
    if not centre:
        return '-'
    return f"({centre[0]:.3f}, {centre[1]:.3f}) {1000*math.dist(centre[:2], SCREW_BIN):.0f} mm from bin"


def main():
    paths = [a for a in sys.argv[1:] if not a.startswith('--')]
    directory = Path(paths[0] if paths else 'runs/benchmark')
    rows = []
    for path in sorted(directory.glob('trial_*/sort_report.json')):
        report = json.loads(path.read_text())
        physics = (report.get('benchmark') or {}).get('physics') or {}
        for o in report.get('objects', []):
            if o.get('object') == 'screw':
                rows.append((path.parent.name, physics, o))
    if not rows:
        sys.exit(f'No screw attempts found under {directory}')

    counts = {}
    for _, _, o in rows:
        counts[stage(o)] = counts.get(stage(o), 0) + 1
    print(f'# Screw attempts: {len(rows)}')
    for name in ('placed', 'not lifted', 'lost in carry', 'out of bin'):
        print(f'- {name}: {counts.get(name, 0)}')

    print('\n## Failed attempts')
    print('trial | screw | try | stage | lift mm | rotation deg lift/carry/lower | slip mm carry | '
          'first turn | tilt | final position | friction x | screw mass x')
    for trial, physics, o in rows:
        s = stage(o)
        if s == 'placed':
            continue
        rot = o.get('grip_rotation_deg', {})
        slip = o.get('grip_slip_mm', {})
        turn = o.get('grip_first_turn') or {}
        first = f"{turn['phase']} {turn['frame']}/{turn['of']}" if turn else '-'
        f = lambda v: '-' if v is None else f'{v:.1f}'
        print(f"{trial} | {o.get('index')} | {o.get('attempt')} | {s} | {f(o.get('lift_height_mm'))} | "
              f"{f(rot.get('lift'))}/{f(rot.get('carry'))}/{f(rot.get('lower'))} | {f(slip.get('carry'))} | "
              f"{first} | {f(o.get('tilt_deg'))} | {where(o)} | "
              f"{physics.get('friction', '-')} | {physics.get('screw', '-')}")

    print('\n## Physics factors (first attempts)')
    for name in ('friction', 'screw'):
        ok = [p[name] for _, p, o in rows if o.get('attempt') == 1 and name in p and o.get('complete')]
        bad = [p[name] for _, p, o in rows if o.get('attempt') == 1 and name in p and not o.get('complete')]
        if ok and bad:
            print(f'- {name}: mean {statistics.mean(ok):.3f} when placed, {statistics.mean(bad):.3f} when failed')

    if '--trace' in sys.argv:
        # Frame-by-frame grip record (every 15 frames) for a few lost and placed attempts.
        lost = [(t, o) for t, _, o in rows if stage(o) == 'lost in carry'][:4]
        kept = [(t, o) for t, _, o in rows if o.get('complete')][:2]
        for label, group in (('LOST', lost), ('PLACED', kept)):
            for trial, o in group:
                print(f"\n### {label} {trial} screw {o.get('index')} try {o.get('attempt')}: "
                      f"jaw/tip errors {o.get('tip_errors_mm')}, sim checks {o.get('sim_check_failures')}")
                for r in o.get('grip_trace', []):
                    print(f"{r['phase']:5} {r['frame']:3}/{r['of']}: rotation {r['rotation_deg']:5.1f} deg, "
                          f"slip {r['slip_mm']:6.1f} mm, joint lag {r['max_joint_lag_deg']:.2f} deg")

    print('\n## Placed attempts: typical grip')
    placed = [o for _, _, o in rows if o.get('complete')]
    for phase in ('lift', 'carry', 'lower'):
        values = [o['grip_rotation_deg'][phase] for o in placed if phase in o.get('grip_rotation_deg', {})]
        if values:
            print(f'- rotation in {phase}: median {statistics.median(values):.1f} deg, max {max(values):.1f} deg')


if __name__ == '__main__':
    main()
