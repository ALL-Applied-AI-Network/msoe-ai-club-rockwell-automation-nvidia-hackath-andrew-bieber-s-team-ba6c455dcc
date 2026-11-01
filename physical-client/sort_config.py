"""Standalone physical-client settings, derived from the team snapshot. Distances: metres.

The separate drops below are proposed bare-table zones, not measured cell positions.
Base coordinates are used everywhere here; policy.py alone converts to UF0 mm.
"""
from pathlib import Path
import os
import sys

ROOT = Path(__file__).resolve().parent
_ROSIE_SAM_PYTHON = Path.home() / "venvs" / "sam2-check" / "bin" / "python"
SAM_PYTHON = os.environ.get("SORT_SAM_PYTHON",
    str(_ROSIE_SAM_PYTHON) if _ROSIE_SAM_PYTHON.is_file() else sys.executable)
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
DROP_HEIGHT = 0.010  # physical-client-only: proposed bare-table release, verify in the cell
TRAVEL_HEIGHT = 0.180
MAX_ATTEMPTS = 2
ATTEMPT_RADIUS = 0.020
MAX_PICKS = 20

# Real robot: explicitly enable only after checking an overlay and measured poses.
PHYSICAL_EXECUTE = False
PHYSICAL_POSES_CONFIRMED = False
# Both must be set intentionally; keeping them separate makes preview runs simple.
SURFACE_Z = -0.0051
UF0_Z_MM = -330.0
# Organizer client ff7534e TCP action box, in UF0 mm. Recheck if its protocol changes.
PHYSICAL_TCP_BOX_MM = ((150.0, 650.0), (-450.0, 450.0), (-334.0, 300.0))
SCAN_XY = (0.18, -0.30)
SCAN_HEIGHT = 0.300
TCP_R_FOR_AXIS_X = 0.0  # R when screw/vial long axis is base +X; verify with real jaws
LINEAR_STEP_MM = 2.0  # 2 mm per 0.1 s target (~20 mm/s nominal)
ANGULAR_STEP_DEG = 2.0
POSITION_TOLERANCE_MM = 2.0
ANGLE_TOLERANCE_DEG = 3.0
MAX_SETTLE_CALLS = 40
PAUSE_AFTER_LARGE_OBJECT = True  # caps/vials can hook: inspect release before reset

# Initial physical test scope. Preview still shows all recognized objects.
# Check the actual jaw clearance / custom fingers before broadening these.
PHYSICAL_GRIPPER_PROFILE = os.environ.get("SORT_GRIPPER_PROFILE", "legacy")
PHYSICAL_ALLOWED_KINDS = {
    "legacy": ("screw", "cube"),
    "dual_zone": ("cube", "vial", "cap"),
    "dual_zone_v2": ("screw", "cube", "vial", "cap"),
}.get(PHYSICAL_GRIPPER_PROFILE, ())
PHYSICAL_MAX_GRASP_WIDTH_MM = 11.0
PHYSICAL_INSPECT_DROP_AREAS = True
PHYSICAL_WATCH = False
PHYSICAL_IDLE_SECONDS = 60.0

# Physical lab operator confirmed that these presented cube/screw sizes fit.
# Ignore their camera-estimated width upper bound only; all other guards remain.
PHYSICAL_VERIFIED_FIT_KINDS = ("screw", "cube")

# Scan once, attempt each unchanged cached target once, then rescan for leftovers.
# 'per_pick' restores the original source scan before every drop.
PHYSICAL_SCAN_MODE = "batch"
PHYSICAL_BATCH_MAX_IDLE_PASSES = 2  # stop if every cached target keeps changing

# Custom fingers: proposed contact frames, NOT a measured physical UTOOL.
# Set these only after mounting, measuring inventory and reviewing real overlays.
DUAL_ZONE_MOUNT_CONFIRMED = False
DUAL_ZONE_OBJECTS_CONFIRMED = False
DUAL_ZONE_LAYOUT_CONFIRMED = False
DUAL_ZONE_PRIMARY_OFFSET_TCP_M = (0.0, 0.0, 0.0)
DUAL_ZONE_CAP_OFFSET_TCP_M = (0.040, 0.0, 0.0)
DUAL_ZONE_CUBE_WIDTH_RANGE_MM = (9.0, 16.0)
DUAL_ZONE_CAP_NARROW_END_UP = False
DUAL_ZONE_PICK_HEIGHTS_M = {"screw": .0025, "cube": .0025, "vial": .003, "cap": .005}
# Asher's original simulator inventory is preserved as a separate profile.
DUAL_ZONE_VIAL_DIAMETER_MM = 17.0
DUAL_ZONE_VIAL_LENGTH_MM = 140.0
# Organizer's physical CAD dimensions; v0.5's vial was scaled down.
DUAL_ZONE_V2_VIAL_DIAMETER_MM = 27.18
DUAL_ZONE_V2_VIAL_LENGTH_MM = 203.0
DUAL_ZONE_V2_VIAL_END_INSET_M = .020
DUAL_ZONE_V2_SCREW_DIAMETER_MM = 6.0
DUAL_ZONE_V2_SCREW_LENGTH_MM = 30.0
DUAL_ZONE_CLEARANCE_MM = 2.0
DUAL_ZONE_MAX_OBSTACLE_HEIGHT_M = .045
# Supply measured walls/stops as bounds_xy, height_m, optional base_z_m.
# The simulator's enclosures are not presumed present in the physical cell.
DUAL_ZONE_STATIC_OBSTACLES = []
if PHYSICAL_GRIPPER_PROFILE != "legacy":
    PAUSE_AFTER_LARGE_OBJECT = False  # batch mode; all three confirmation gates still apply
