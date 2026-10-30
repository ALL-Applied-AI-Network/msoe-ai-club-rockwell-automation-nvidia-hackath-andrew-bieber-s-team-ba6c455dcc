"""Host checks for the randomized benchmark; no Isaac required."""
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest

PROBE = r'''
import json, math
import numpy as np
import scene_config as C
import bench_layout as L
import scan_sort_bins as B

a = L.random_layout(7, 0, C.VIAL_LENGTH)
b = L.random_layout(7, 0, C.VIAL_LENGTH)
attempt, layout = L.valid_layout(C, 1000, verbose=False)
points = ([v[:2] for v in layout['vials']] + list(layout['caps'])
          + [c[:2] for c in layout['cubes']] + [s[:2] for s in layout['screws']])
in_keepout = [any(x0 < x < x1 and y0 < y < y1 for x0, x1, y0, y1 in L.keepouts()) for x, y in points]
small = list(layout['caps']) + [c[:2] for c in layout['cubes']] + [s[:2] for s in layout['screws']]
gaps = [math.dist(p, q) for i, p in enumerate(small) for q in small[i+1:]]
base = (C.OBJECT_STATIC_FRIC, C.CUBE_MASS, C.VIAL_MASS)
factors = L.apply_physics(C, 1000)
print(json.dumps(dict(
    deterministic=a == b, attempt=attempt,
    counts=[len(layout[k]) for k in ('screws', 'cubes', 'vials', 'caps')],
    large_cubes=sum(len(c) > 3 for c in layout['cubes']),
    in_keepout=any(in_keepout), min_gap=min(gaps),
    radii=max(math.hypot(*p) for p in points),
    planner_ok=L.planner_accepts(),
    factors=factors, scaled=[C.OBJECT_STATIC_FRIC/base[0], C.CUBE_MASS/base[1], C.VIAL_MASS/base[2]])))
'''


class BenchmarkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        out = subprocess.run([sys.executable, '-c', PROBE], cwd=Path(__file__).resolve().parent,
                             env=dict(os.environ, FINGER_SET='dual_v2'),
                             text=True, capture_output=True, check=True).stdout
        cls.data = json.loads(out.strip().splitlines()[-1])
        sys.path.insert(0, str(Path(__file__).resolve().parent))

    def test_layouts_are_reproducible(self):
        self.assertTrue(self.data['deterministic'])

    def test_layout_has_every_lab_object(self):
        self.assertEqual(self.data['counts'], [2, 7, 2, 2])
        self.assertEqual(self.data['large_cubes'], 1)

    def test_layout_is_separated_reachable_and_clear_of_destinations(self):
        self.assertFalse(self.data['in_keepout'])
        self.assertGreaterEqual(self.data['min_gap'], .09)
        self.assertLessEqual(self.data['radii'], .535)

    def test_accepted_layout_passes_the_planner(self):
        self.assertTrue(self.data['planner_ok'])

    def test_physics_variation_within_20_percent(self):
        for value in self.data['factors'].values():
            self.assertTrue(.8 <= value <= 1.2)
        f = self.data['factors']
        for got, want in zip(self.data['scaled'], (f['friction'], f['cube'], f['vial'])):
            self.assertAlmostEqual(got, want, places=2)

    def test_wilson_interval(self):
        from bench_summary import wilson
        low, high = wilson(30, 30)
        self.assertGreater(low, .88)   # 30/30 -> at least ~89% at 95% confidence
        self.assertEqual(high, 1.0)
        low, high = wilson(45, 50)
        self.assertTrue(low < .9 < high)


if __name__ == '__main__':
    unittest.main()
