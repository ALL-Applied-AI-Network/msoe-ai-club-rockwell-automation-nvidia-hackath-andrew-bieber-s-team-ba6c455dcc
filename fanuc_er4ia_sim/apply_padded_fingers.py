#!/usr/bin/env python3
"""Wire the padded finger sets into scene_config.py and cell.py.

Run from the repo folder (or pass the folder):  python3 apply_padded_fingers.py [dir]
Handles a fresh repo, a repo patched by the earlier version of this script (it is
upgraded to the new presets), and a repo that is already up to date.  Originals are
saved as <file>.orig the first time a file changes.  Each edit must match exactly
once or that file is left untouched.
"""

import re
import shutil
import sys
from pathlib import Path

SCENE_ANCHOR = "FINGER_INNER_TIP = (SCALLOP_R**2 - SCALLOP_ABOVE_TIP**2) ** 0.5\n"
SCENE_BLOCK = '''
# Finger sets, chosen per run:  FINGER_SET=<name> python <script>
#   scallop    original AA-pocket blades (default)
#   padded     flat blades, 4 mm relief + 1 mm pad (Padded_Gripper_Test.STEP): 8 closed / 20 open
#   cap        7 mm relief: 14 closed / 26 open.  Caps and vials; too wide to clamp 10 mm cubes
#   universal  4.5 mm relief and 8.5 mm stroke per side: 9 closed / 26 open.  Picks everything,
#              but needs a longer-stroke gripper than the SMC MHZL2-16D2 (6 mm per side)
# PAD_FACE_MM and JAW_STROKE_MM override a preset for experiments.
import os as _os

FINGER_SET = _os.environ.get("FINGER_SET", "scallop")
_PRESETS = {  # name: (pad face offset from root face [m], per-side stroke [m] or None = SMC's)
    "padded": (0.004, None),
    "cap": (0.007, None),
    "universal": (0.0045, 0.0085),
}
PAD_STEP_X = 0.044  # relief starts here, from the carriage end (STEP: 44 mm)
PAD_T = 0.001  # rubber pad thickness
PAD_LEAD_IN = 0.002  # chamfer length on the pad's tip edge
PAD_FACE = 0.004
if FINGER_SET != "scallop":
    if FINGER_SET not in _PRESETS:
        raise ValueError(f"FINGER_SET={FINGER_SET!r}; use scallop, {', '.join(_PRESETS)}")
    PAD_FACE, _stroke = _PRESETS[FINGER_SET]
    if "PAD_FACE_MM" in _os.environ:
        PAD_FACE = float(_os.environ["PAD_FACE_MM"]) / 1000.0
    if "JAW_STROKE_MM" in _os.environ:
        _stroke = float(_os.environ["JAW_STROKE_MM"]) / 1000.0
    assert PAD_STEP_X < FINGER_LENGTH
    assert PAD_FACE >= 0.0 and PAD_FACE + PAD_T <= FINGER_WIDTH - 0.003, (
        "PAD_FACE too deep: less than 3 mm of support would remain behind the pad"
    )
    if _stroke is not None:
        JAW_STROKE = _stroke
        JAW_OPEN = JAW_CLOSED + JAW_STROKE
        # The invisible carriage proxies close toward each other (CAD open centres are
        # +/-13.9 mm); narrow them so a longer stroke cannot make them collide.
        CARRIAGE_W = min(CARRIAGE_W, 2.0 * (0.0139 - JAW_OPEN) - 0.0008)
        assert CARRIAGE_W > 0.004, "JAW_STROKE too long for the carriage proxies"
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

V1_BLOCK = re.compile(r"\n# Finger set:.*?\n    FINGER_INNER_TIP = PAD_FACE\n", re.S)


def save(path, text):
    backup = path.with_suffix(path.suffix + ".orig")
    if not backup.exists():
        shutil.copy2(path, backup)
    path.write_text(text, encoding="utf-8")


def patch_scene(path):
    text = path.read_text(encoding="utf-8")
    if "_PRESETS = {" in text:
        print("scene_config.py: already up to date")
        return
    if V1_BLOCK.search(text):
        text = V1_BLOCK.sub(lambda m: SCENE_BLOCK, text, count=1)
        action = "upgraded earlier padded-finger block to presets"
    elif text.count(SCENE_ANCHOR) == 1:
        text = text.replace(SCENE_ANCHOR, SCENE_ANCHOR + SCENE_BLOCK)
        action = "added finger presets"
    else:
        sys.exit(f"scene_config.py: anchor line found {text.count(SCENE_ANCHOR)} times, "
                 f"expected 1. Nothing written.")
    save(path, text)
    print(f"scene_config.py: {action}")


def patch_cell(path):
    text = path.read_text(encoding="utf-8")
    pending = []
    for old, new in CELL_EDITS:
        if new in text:
            continue
        if text.count(old) != 1:
            sys.exit(f"cell.py: expected 1 match, found {text.count(old)} for:\n{old}\nNothing written.")
        pending.append((old, new))
    if not pending:
        print("cell.py: already up to date")
        return
    for old, new in pending:
        text = text.replace(old, new)
    save(path, text)
    print(f"cell.py: {len(pending)} edit(s) applied")


if __name__ == "__main__":
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent
    patch_scene(root / "scene_config.py")
    patch_cell(root / "cell.py")