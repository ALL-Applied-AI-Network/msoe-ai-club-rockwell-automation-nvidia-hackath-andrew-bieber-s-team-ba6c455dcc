#!/usr/bin/env python3
"""Add [dbg] lines to a pick script: measured vs commanded jaw position, and where
the cap sits relative to the fingertip, at the end of descend / close / dwell / lift.

    python3 add_grip_debug.py demo_vision_pick_caps.py

Only prints when a padded finger set is active.  Keeps a .orig backup; safe to re-run.
"""

import shutil
import sys
from pathlib import Path

OLD = """                for q, jaw, position in steps:
                    tick(q, jaw, render=True)
                actual_tip = fk.tool_pose(measured()[arm_idx], tool)[1]
"""
NEW = """                for q, jaw, position in steps:
                    tick(q, jaw, render=True)
                if name in ("descend", "close", "dwell", "lift") and C.FINGER_SET != "scallop":
                    _now = measured()
                    _targets = cell.jaw_targets(jaw)
                    _meas = [round(float(_now[joints[k]]) * 1000, 2) for k in _targets]
                    _cmd = [round(float(v) * 1000, 2) for v in _targets.values()]
                    _tip = fk.tool_pose(_now[arm_idx], tool)[1]
                    _rel = (np.asarray(body.get_world_pose()[0], dtype=float) - _tip) * 1000
                    print(
                        f"[dbg] {name:8s} jaw_mm measured={_meas} cmd={_cmd} "
                        f"cap-minus-tip mm=({_rel[0]:.1f}, {_rel[1]:.1f}, {_rel[2]:.1f})",
                        flush=True,
                    )
                actual_tip = fk.tool_pose(measured()[arm_idx], tool)[1]
"""

for name in sys.argv[1:] or ["demo_vision_pick_caps.py"]:
    path = Path(name)
    if not path.exists():
        print(f"{name}: not found")
        continue
    text = path.read_text(encoding="utf-8")
    if NEW in text:
        print(f"{name}: already patched")
    elif text.count(OLD) != 1:
        print(f"{name}: phase loop not found exactly once ({text.count(OLD)}); left unchanged")
    else:
        shutil.copy2(path, str(path) + ".dbg.orig")
        path.write_text(text.replace(OLD, NEW), encoding="utf-8")
        print(f"{name}: patched")