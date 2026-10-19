#!/usr/bin/env python3
"""Wire the padded finger set into scene_config.py and cell.py.

Run from the repo folder (or pass the folder):  python apply_padded_fingers.py [dir]
Each edit must match exactly once or nothing is written.  Originals are saved as
<file>.orig.  Safe to re-run: edits already applied are skipped.
"""

import shutil
import sys
from pathlib import Path

SCENE_OLD = "FINGER_INNER_TIP = (SCALLOP_R**2 - SCALLOP_ABOVE_TIP**2) ** 0.5\n"
SCENE_NEW = SCENE_OLD + '''
# Finger set: "scallop" (original AA-pocket blades) or "padded" (flat blades with
# relieved, rubber-padded tips, from Padded_Gripper_Test.STEP).  Switch per run:
#   FINGER_SET=padded python <script>
# PAD_FACE_MM overrides the relief depth (STEP value: 4 mm -> pad gap 8 closed / 20 open).
import os as _os

FINGER_SET = _os.environ.get("FINGER_SET", "scallop")
PAD_STEP_X = 0.044  # relief starts here, measured from the carriage end (STEP: 44 mm)
PAD_T = 0.001  # rubber pad thickness
PAD_FACE = float(_os.environ.get("PAD_FACE_MM", "4")) / 1000.0
if FINGER_SET == "padded":
    assert PAD_STEP_X < FINGER_LENGTH
    assert PAD_FACE >= 0.0 and PAD_FACE + PAD_T <= FINGER_WIDTH - 0.003, (
        "PAD_FACE too deep: less than 3 mm of support would remain behind the pad"
    )
    # Flat pad faces replace the scallop's inner-tip half-gap at full close.
    FINGER_INNER_TIP = PAD_FACE
'''

CELL_EDITS = [
    (
        "from gripper_geometry import finger_profile, clip_profile, triangulate as _ear_clip\n",
        "from gripper_geometry import finger_profile, clip_profile, triangulate as _ear_clip\n"
        "import padded_fingers\n",
    ),
    (
        "        for section, name, visual_material, physics_material in sections:\n",
        "        if padded_fingers.enabled():\n"
        "            sections = padded_fingers.finger_sections(\n"
        "                sign, PHYS_MAT_PATH, SOFT_TIP_MAT_PATH\n"
        "            )\n"
        "        for section, name, visual_material, physics_material in sections:\n",
    ),
    (
        '            UsdGeom.Imageable(fprim).CreateVisibilityAttr().Set("invisible")\n'
        "        detailed_finger_visual(stage, jaw_path, sign, finger_x - carriage_x)\n",
        "            if not padded_fingers.enabled():\n"
        '                UsdGeom.Imageable(fprim).CreateVisibilityAttr().Set("invisible")\n'
        "        if not padded_fingers.enabled():\n"
        "            detailed_finger_visual(stage, jaw_path, sign, finger_x - carriage_x)\n",
    ),
]


def patch(path, edits):
    text = path.read_text(encoding="utf-8")
    pending = []
    for old, new in edits:
        if new in text:
            print(f"  already applied: {old.strip()[:60]}")
            continue
        n = text.count(old)
        if n != 1:
            sys.exit(f"{path.name}: expected 1 match, found {n} for:\n{old}\nNothing written.")
        pending.append((old, new))
    if not pending:
        return
    shutil.copy2(path, path.with_suffix(path.suffix + ".orig"))
    for old, new in pending:
        text = text.replace(old, new)
    path.write_text(text, encoding="utf-8")
    print(f"patched {path.name} ({len(pending)} edit(s)); original kept as {path.name}.orig")


if __name__ == "__main__":
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent
    patch(root / "scene_config.py", [(SCENE_OLD, SCENE_NEW)])
    patch(root / "cell.py", CELL_EDITS)