"""Settings shared by the Isaac runner and the physical policy. Distances: metres.

The separate drops below are simulation starting points, not measured cup positions.
Base coordinates are used everywhere here; policy.py alone converts to UF0 mm.
"""
from pathlib import Path
import os
import sys

ROOT = Path(__file__).resolve().parent
SAM_PYTHON = os.environ.get("SORT_SAM_PYTHON", sys.executable)
MODEL_ID = "facebook/sam2.1-hiera-small"
WORKSPACE = (0.22, 0.49, -0.27, 0.27)  # xmin, xmax, ymin, ymax; pickup area
MAX_RADIUS = 0.54  # extra margin inside the advertised ~0.55 m reach
DROP_XY = {
    "red": (0.20, 0.25),
    "blue": (0.20, 0.00),
    "yellow": (0.37, 0.34),
    "screws": (0.20, -0.25),
    "caps": (0.35, -0.35),
    "vials": (0.17, -0.43),
}
MISC_DROP_XY = (0.17, 0.43)  # seventh destination; simulation --include-misc
MISC_MAX_LENGTH_MM = 40.0
MISC_MAX_WIDTH_MM = 10.0  # leave clearance inside the nominal 12 mm jaw opening
DROP_RADIUS = 0.065  # zone centre tolerance; the sim leaves only matching labels here
BIN_FOR = {
    "cube_red": "red", "cube_blue": "blue", "cube_yellow": "yellow",
    "screw": "screws", "cap": "caps", "vial": "vials",
    "misc": "misc",
}
# Six labels because the supplied scene contains yellow cubes as well as red/blue.
# Each label has its own destination; the optional misc destination is separate too.
PICK_ORDER = {"screw": 0, "cube": 1, "misc": 2, "cap": 3, "vial": 4}
PICK_HEIGHT = {"screw": 0.0025, "cube": 0.0025, "misc": 0.0025, "cap": 0.005, "vial": 0.005}
DROP_HEIGHT = 0.100  # fingertip above table; confirm clearance over actual cup rims
TRAVEL_HEIGHT = 0.180
MAX_ATTEMPTS = 2  # attempts per location, across fresh detections
ATTEMPT_RADIUS = 0.020
MAX_PICKS = 20

# Real robot: explicitly enable only after checking an overlay and measured poses.
PHYSICAL_EXECUTE = False
PHYSICAL_POSES_CONFIRMED = False
# Both must be set intentionally; keeping them separate makes preview runs simple.
SURFACE_Z = -0.0051
UF0_Z_MM = -330.0
SCAN_XY = (0.18, -0.30)
SCAN_HEIGHT = 0.300
TCP_R_FOR_AXIS_X = 0.0  # R when screw/vial long axis is base +X; verify with real jaws
LINEAR_STEP_MM = 2.0  # 2 mm per 0.1 s target (~20 mm/s nominal)
ANGULAR_STEP_DEG = 2.0
POSITION_TOLERANCE_MM = 2.0
ANGLE_TOLERANCE_DEG = 3.0
MAX_SETTLE_CALLS = 40
PAUSE_AFTER_LARGE_OBJECT = True  # caps/vials can hook: inspect release before reset
