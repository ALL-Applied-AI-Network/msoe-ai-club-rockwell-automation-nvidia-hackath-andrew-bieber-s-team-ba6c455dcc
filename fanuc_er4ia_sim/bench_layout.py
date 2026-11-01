"""Randomized, reproducible benchmark trials: object layout + physics variation.

random_layout(seed) places every lab object (2 M6 screws, 6 x 10 mm cubes,
1 x 16 mm cube, 2 vials, 2 caps) in the reachable pick area, clear of the bins,
the vial tray and the parked arm. apply_layout() writes it into scene_config,
and planner_accepts() runs the same IK + clearance check as the sorter. The
benchmark only uses layouts that pass: the attempt index k is part of the
recorded result, so every trial can be replayed exactly.

apply_physics(seed) scales object friction and each object type's mass by a
random factor in [0.8, 1.2].
"""
import math
import random

import numpy as np

# Pick area (robot-base metres). x >= 0.21 keeps objects out from under the
# parked arm; the far edge stays within comfortable reach.
X_RANGE = (0.21, 0.53)
Y_RANGE = (-0.27, 0.24)
MAX_RADIUS = 0.535
# Keep-out rectangles (x0, x1, y0, y1) around destinations, with margin.
MARGIN = 0.045


def keepouts():
    import scan_sort_bins as B
    boxes = []
    for (x, y), _ in B.REAL_BINS.values():
        h = B.BIN_OUTER/2 + MARGIN
        boxes.append((x-h, x+h, y-h, y+h))
    for (x, y), (w, d), _ in B.BINS.values():
        boxes.append((x-w/2-MARGIN, x+w/2+MARGIN, y-d/2-MARGIN, y+d/2+MARGIN))
    return boxes


def _segment_distance(p, a, b):
    ab = b - a
    t = 0.0 if not ab.any() else float(np.clip((p-a)@ab/(ab@ab), 0, 1))
    return float(np.linalg.norm(p - (a + t*ab)))


def random_layout(seed, attempt=0, vial_length=0.203):
    """One candidate layout; None if the random draw could not separate objects."""
    rng = random.Random(f'{seed}:{attempt}')
    boxes = keepouts()
    placed = []   # (centre xy, clearance radius)
    vials = []    # (a, b) clearance segments, extended past the ends

    def free(xy, radius, segment=None):
        x, y = xy
        if not (X_RANGE[0] <= x <= X_RANGE[1] and Y_RANGE[0] <= y <= Y_RANGE[1]):
            return False
        if math.hypot(x, y) > MAX_RADIUS:
            return False
        pts = [np.asarray(xy)] if segment is None else [segment[0], segment[1], np.asarray(xy)]
        for px, py in pts:
            for x0, x1, y0, y1 in boxes:
                if x0 - radius < px < x1 + radius and y0 - radius < py < y1 + radius:
                    return False
        for c, r in placed:
            d = (np.linalg.norm(np.asarray(xy)-c) if segment is None
                 else _segment_distance(c, *segment))
            if d < r + radius:
                return False
        for a, b in vials:
            d = (_segment_distance(np.asarray(xy), a, b) if segment is None
                 else min(_segment_distance(segment[0], a, b), _segment_distance(segment[1], a, b),
                          _segment_distance(a, *segment), _segment_distance(b, *segment)))
            if d < radius + 0.035:
                return False
        return True

    def draw(radius, tries=4000):
        for _ in range(tries):
            xy = (rng.uniform(*X_RANGE), rng.uniform(*Y_RANGE))
            if free(xy, radius):
                placed.append((np.asarray(xy), radius))
                return xy
        return None

    layout = dict(vials=[], caps=[], cubes=[], cube_yaws=[], screws=[])
    # Vials first (largest). Clearance segment extends 45 mm past each end for
    # the inactive narrow rail, which hangs beyond the gripped end.
    for _ in range(2):
        for _ in range(4000):
            xy = (rng.uniform(0.29, 0.41), rng.uniform(-0.24, 0.14))
            yaw = rng.uniform(-20.0, 20.0)
            u = np.array([math.cos(math.radians(yaw)), math.sin(math.radians(yaw))])
            half = vial_length/2 + 0.045
            seg = (np.asarray(xy)-u*half, np.asarray(xy)+u*half)
            if free(xy, 0.015, seg):
                vials.append(seg)
                layout['vials'].append((round(xy[0], 4), round(xy[1], 4), round(yaw, 2)))
                break
        else:
            return None
    for _ in range(2):
        xy = draw(0.045)
        if xy is None:
            return None
        layout['caps'].append((round(xy[0], 4), round(xy[1], 4)))
    colours = ['yellow', 'yellow', 'red', 'red', 'blue', 'blue', 'blue']
    for i, colour in enumerate(colours):
        xy = draw(0.045 if i < 6 else 0.05)
        if xy is None:
            return None
        entry = (round(xy[0], 4), round(xy[1], 4), colour)
        layout['cubes'].append(entry if i < 6 else entry + ('large',))
        layout['cube_yaws'].append(round(rng.uniform(-45.0, 45.0), 2))
    for _ in range(2):
        xy = draw(0.045)
        if xy is None:
            return None
        layout['screws'].append((round(xy[0], 4), round(xy[1], 4), round(rng.uniform(0, 180.0), 2)))
    return layout


def apply_layout(C, layout):
    C.SORT_VIALS = [tuple(v) for v in layout['vials']]
    C.SORT_CAPS = [tuple(c) for c in layout['caps']]
    C.SORT_CUBES = [(x, y, colour, C.CUBE_LARGE_EDGE) if rest else (x, y, colour)
                    for x, y, colour, *rest in layout['cubes']]
    C.SORT_CUBE_YAWS_DEG = list(layout['cube_yaws'])
    C.SORT_SCREWS = [tuple(s) for s in layout['screws']] if C.FINGER_SET == 'dual_v2' else []


def planner_accepts(verbose=False):
    """Same startup IK + clearance check as demo_scan_sort (no Isaac needed)."""
    import demo_scan_sort as S
    items = S.inventory()
    previous = S.park_pose()
    for i, pick in enumerate(items):
        obstacles = [dict(p, xy=p['place_xy'], scan_yaw_deg=0.) if j < i else p
                     for j, p in enumerate(items) if j != i and not (j < i and p.get('bin'))]
        try:
            plan = S.plan_any(pick, previous, obstacles)
        except RuntimeError as exc:
            if verbose:
                print(f"[bench] layout rejected: {exc}", flush=True)
            return False
        previous = plan[1][-1][1][-1][0].copy()
    return True


def valid_layout(C, seed, max_attempts=300, verbose=True):
    """First planner-accepted layout for this seed: (attempt, layout)."""
    for attempt in range(max_attempts):
        layout = random_layout(seed, attempt, C.VIAL_LENGTH)
        if layout is None:
            continue
        apply_layout(C, layout)
        if planner_accepts(verbose):
            return attempt, layout
    raise RuntimeError(f'No planner-accepted layout for seed {seed} in {max_attempts} attempts')


def apply_physics(C, seed):
    """Scale object friction and per-type masses by U[0.8, 1.2]; returns the factors."""
    rng = random.Random(f'physics:{seed}')
    factors = dict(friction=rng.uniform(0.8, 1.2),
                   **{k: rng.uniform(0.8, 1.2) for k in ('cube', 'screw', 'vial', 'cap')})
    C.OBJECT_STATIC_FRIC *= factors['friction']
    C.OBJECT_DYNAMIC_FRIC *= factors['friction']
    C.CUBE_MASS *= factors['cube']
    C.CUBE_LARGE_MASS *= factors['cube']
    C.SCREW_MASS *= factors['screw']
    C.VIAL_MASS *= factors['vial']
    C.CAP_MASS *= factors['cap']
    return {k: round(v, 3) for k, v in factors.items()}
