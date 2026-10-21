#!/usr/bin/env python3
"""Make a pick script use per-object jaw widths when a padded finger set is active.

    python3 patch_pick_script.py demo_vision_pick.py demo_vision_pick_caps.py

Only the plan_trajectory(...) call changes: instead of always closing to the cube's
width (args.jaw_close) it asks padded_fingers.jaw_for_object().  With FINGER_SET unset
(old fingers) behaviour is identical.  A .orig backup is kept; safe to re-run.
"""

import shutil
import sys
from pathlib import Path

OLD = """            phases, worst = plan_trajectory(
                target,
                initial,
                home,
                limits,
                tool,
                pick_z,
                args.drop_xy,
                drop_z,
                args.jaw_close,
                release_slide,
            )
"""
NEW = """            jaw_close = args.jaw_close
            if C.FINGER_SET != "scallop":
                import padded_fingers

                jaw_close = padded_fingers.jaw_for_object(object_type, args.cap_pick_height)
                print(f"[vision] jaw close {jaw_close * 1000:.2f} mm per side", flush=True)
            phases, worst = plan_trajectory(
                target,
                initial,
                home,
                limits,
                tool,
                pick_z,
                args.drop_xy,
                drop_z,
                jaw_close,
                release_slide,
            )
"""

for name in sys.argv[1:] or ["demo_vision_pick.py"]:
    path = Path(name)
    text = path.read_text(encoding="utf-8")
    if NEW in text:
        print(f"{name}: already patched")
    elif text.count(OLD) != 1:
        print(f"{name}: plan_trajectory call not found exactly once ({text.count(OLD)}); left unchanged")
    else:
        shutil.copy2(path, str(path) + ".orig")
        path.write_text(text.replace(OLD, NEW), encoding="utf-8")
        print(f"{name}: patched")