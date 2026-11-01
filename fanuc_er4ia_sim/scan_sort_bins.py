"""Sorting destinations: the cell's real bins plus a walled vial tray.

Bins match the organizer CAD (https://hackathon.sidclab.org/#resources/cad-models):
50 x 50 mm outside, 55 mm tall, 35 x 35 mm pocket 25 mm deep. Screws, 10/16 mm
cubes and caps fit a pocket; 203 mm vials do not and keep a walled floor tray.
The gripper cannot reach into a 35 mm pocket (open fingers ~49 mm across, wide
rail 50 mm long), so objects are released just above the rim and drop in.

SORT_BINS=0 restores the earlier floor-marked areas (no bins).
"""
import os
import numpy as np
import scene_config as C

USE_BINS = os.environ.get('SORT_BINS', '1') != '0'

# ------------------------------------------------------------------ real bins
BIN_OUTER = 0.050
BIN_HEIGHT = 0.055
BIN_POCKET = 0.035
BIN_DEPTH = 0.025
BIN_FLOOR = BIN_HEIGHT - BIN_DEPTH          # pocket floor above the table
RELEASE_ABOVE_RIM = 0.012                   # fingertips this far above the rim

# name: (centre xy, colour). Positions: reachable, clear of the pick area and
# of the parked arm's camera view (checked by demo_scan_sort --check-only).
REAL_BINS = {
    'yellow': ((.16, -.36), (1., .75, .03)),
    'red':    ((.26, -.36), (.9, .06, .05)),
    'blue':   ((.36, -.36), (.05, .25, .9)),
    'screw':  ((.13, -.48), (.35, .35, .38)),
    'cap_a':  ((.38, .33), (.65, .15, .8)),
    'cap_b':  ((.46, .30), (.65, .15, .8)),
}
# Which bin each object of a group goes to, in pick order.
GROUP_BINS = {
    'cube_yellow': ['yellow', 'yellow'],
    'cube_red': ['red', 'red'],
    'cube_blue': ['blue', 'blue'],
    'cube_large': ['blue'],          # the 16 mm cube is blue
    'screw': ['screw', 'screw'],
    'cap': ['cap_a', 'cap_b'],       # one Ø31.75 mm cap per 35 mm pocket
}

# ------------------------------------------------------------------ floor areas
BINS = {  # walled floor areas: (centre, (w, d), colour)
    'vial': ((.23, .39), (.20, .18), (.02, .7, .7)),
}
SLOTS = {'vial': [(.23, .35), (.23, .43)]}
if C.FINGER_SET == 'dual_v2':
    # Real 203 mm vials plus the inactive primary rail need a wider destination.
    BINS['vial'] = ((.20, .39), (.27, .18), (.02, .7, .7))
    SLOTS['vial'] = [(.20, .35), (.20, .43)]
NO_WALLS = set()

if USE_BINS:
    for group, names in GROUP_BINS.items():
        SLOTS[group] = [REAL_BINS[n][0] for n in names]
else:
    BINS.update({
        'cube_yellow': ((.16, -.36), (.096, .16), (1., .75, .03)),
        'cube_red': ((.26, -.36), (.096, .16), (.9, .06, .05)),
        'cube_blue': ((.36, -.36), (.096, .16), (.05, .25, .9)),
        'cap': ((.41, .33), (.13, .14), (.65, .15, .8)),
        # Small parts that cannot roll away: floor marking only. The long wide
        # rail cannot reach between the walls of an area this small.
        'screw': ((.135, -.50), (.10, .05), (.35, .35, .38)),
        'cube_large': ((.06, -.50), (.035, .035), (.2, .5, .9)),
    })
    SLOTS.update({
        'cube_yellow': [(.16, -.32), (.16, -.40)],
        'cube_red': [(.26, -.32), (.26, -.40)],
        'cube_blue': [(.36, -.32), (.36, -.40)],
        'cap': [(.41, .30), (.41, .36)],
        'screw': [(.11, -.50), (.16, -.50)],
        'cube_large': [(.06, -.50)],
    })
    NO_WALLS = {'screw', 'cube_large'}

WALL_HEIGHT = .008
WALL_THICK = .003


# Drop points inside a cube bin, in fill order (offsets from the bin centre).
# Releasing every cube over the centre stacked them; the 16 mm cube then sat
# above the rim. Three corners keep 10, 10 and 16 mm cubes apart in 35 mm.
# 10 mm cubes at (+9,-9)/(-9,+9) mm clear the walls by 3.5 mm; the 16 mm cube
# at (+7.5,+7.5) clears them by 2 mm and does not overlap either.
CUBE_DROP_OFFSETS = [(.009, -.009), (-.009, .009), (.0075, .0075)]
CUBE_BINS = {'yellow', 'red', 'blue'}


def drop_xy(name, k):
    """Release XY for the k-th object dropped into a bin."""
    (x, y), _ = REAL_BINS[name]
    if name in CUBE_BINS:
        dx, dy = CUBE_DROP_OFFSETS[k % len(CUBE_DROP_OFFSETS)]
        return (x + dx, y + dy)
    return (x, y)  # screws and caps: centre (a long screw has no room to offset)


def bin_for(group, index):
    """Bin name for the index-th object of a group, or None (floor area)."""
    names = GROUP_BINS.get(group) if USE_BINS else None
    return names[index] if names else None


def release_tip_height():
    """Fingertip height above the work surface when releasing into a bin."""
    return BIN_HEIGHT + RELEASE_ABOVE_RIM


def in_bin(centre, name, margin=0.0):
    """True if an object centre (robot-base XY, world Z) rests inside a bin pocket.
    Objects resting on others may poke above the rim: allow the centre up to
    10 mm above it (nothing can rest there unless it is supported in the pocket)."""
    (x, y), _ = REAL_BINS[name]
    half = BIN_POCKET/2 - margin
    floor = C.TABLE_TOP_Z + C.SURFACE_DZ + BIN_FLOOR
    return bool(abs(centre[0]-x) < half and abs(centre[1]-y) < half
                and floor - 0.002 < centre[2] < floor + BIN_DEPTH + 0.010)


def bin_boxes(name):
    """Static boxes for one bin: solid base up to the pocket floor and 4 walls."""
    (x, y), colour = REAL_BINS[name]
    t = (BIN_OUTER - BIN_POCKET)/2
    z0 = C.SURFACE_DZ
    yield 'Base', np.array([x, y, z0+BIN_FLOOR/2]), np.array([BIN_OUTER, BIN_OUTER, BIN_FLOOR]), colour
    zc = z0 + BIN_FLOOR + BIN_DEPTH/2
    for tag, centre, size in (
            ('WallXm', (x-(BIN_POCKET+t)/2, y), (t, BIN_OUTER, BIN_DEPTH)),
            ('WallXp', (x+(BIN_POCKET+t)/2, y), (t, BIN_OUTER, BIN_DEPTH)),
            ('WallYm', (x, y-(BIN_POCKET+t)/2), (BIN_POCKET, t, BIN_DEPTH)),
            ('WallYp', (x, y+(BIN_POCKET+t)/2), (BIN_POCKET, t, BIN_DEPTH))):
        yield tag, np.array([*centre, zc]), np.array(size), colour


# End-region side stops leave the centre and offset cap rail unobstructed.
# No raised floor: the cylinder remains supported by the original table.
def vial_stops():
    # v2: the 50 mm wide pad covers 34.5-84.5 mm from the vial centre on one
    # side, so the stops sit near the middle (16-24 mm), clear of it.
    end_offset = .020 if C.FINGER_SET == 'dual_v2' else .060
    side_offset = C.VIAL_RADIUS + .0045
    for i, (x, y) in enumerate(SLOTS['vial']):
        for end in (-1, 1):
            for side in (-1, 1):
                yield ('vial', f'Cradle{i}_{end+1}_{side+1}',
                       np.array([x+end*end_offset, y+side*side_offset, C.SURFACE_DZ+.005]),
                       np.array([.008, .004, .010]), (.04, .40, .45))


def walls():
    """Every static obstacle: vial stops, floor-area walls and bins.
    Yields (group, name, centre, size, colour); centres are robot-base metres."""
    yield from vial_stops()
    for group, ((x, y), (w, d), color) in BINS.items():
        if group in NO_WALLS:
            continue
        for name, size, xy in [
                ('Left', (WALL_THICK, d+2*WALL_THICK, WALL_HEIGHT), (x-w/2-WALL_THICK/2, y)),
                ('Right', (WALL_THICK, d+2*WALL_THICK, WALL_HEIGHT), (x+w/2+WALL_THICK/2, y)),
                ('Bottom', (w, WALL_THICK, WALL_HEIGHT), (x, y-d/2-WALL_THICK/2)),
                ('Top', (w, WALL_THICK, WALL_HEIGHT), (x, y+d/2+WALL_THICK/2))]:
            yield group, name, np.array([*xy, C.SURFACE_DZ+WALL_HEIGHT/2]), np.array(size), color
    if USE_BINS:
        for name in REAL_BINS:
            for tag, centre, size, colour in bin_boxes(name):
                yield f'bin_{name}', tag, centre, size, colour


def build(stage):
    import cell
    from pxr import UsdGeom, Gf
    for group, ((x, y), (w, d), color) in BINS.items():
        # Colored floor is visual only. Original support height stays unchanged.
        cell.box(stage, f'/World/SortingAreas/{group}/Color', (w, d, .0001),
                 (x, y, C.TABLE_TOP_Z+C.SURFACE_DZ+.00006), color, collision=False)
    for group, name, center, size, color in walls():
        position = center+[0, 0, C.TABLE_TOP_Z]
        prim = cell.box(stage, f'/World/SortingAreas/{group}/{name}', tuple(size), tuple(position), color)
        cell.bind_physics_material(prim)
        cell.small_contact_offsets(prim)
    # Floor-slot outlines (bins need none); give all 5 vertices an explicit width.
    for group, slots in SLOTS.items():
        if USE_BINS and group in GROUP_BINS:
            continue
        for i, (x, y) in enumerate(slots):
            hx, hy = {'vial': (C.VIAL_LENGTH/2+.003, C.VIAL_RADIUS+.0035), 'cap': (.019, .019),
                      'screw': (C.SCREW_LENGTH/2+.003, C.SCREW_HEAD_D/2+.002)}.get(group, (.011, .011))
            z = C.TABLE_TOP_Z+C.SURFACE_DZ+.0002
            curve = UsdGeom.BasisCurves.Define(stage, f'/World/SortingAreas/{group}/Slot{i}')
            curve.CreateTypeAttr('linear')
            curve.CreateCurveVertexCountsAttr([5])
            curve.CreatePointsAttr([Gf.Vec3f(a, b, z) for a, b in
                                    [(x-hx, y-hy), (x+hx, y-hy), (x+hx, y+hy), (x-hx, y+hy), (x-hx, y-hy)]])
            curve.CreateWidthsAttr([.0006]*5)
            curve.CreateDisplayColorAttr([Gf.Vec3f(1, 1, 1)])
