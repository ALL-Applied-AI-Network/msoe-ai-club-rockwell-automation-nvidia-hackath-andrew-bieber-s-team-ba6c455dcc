"""Host checks for every object on the organizer's CAD page; no Isaac required.

M6 x 30 countersunk screw, 10 mm and 16 mm cubes, real vial and cap: sizes,
gripper fit, overhead-scan silhouette fits and sorting destinations.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest

PROBE = r'''
import json
import numpy as np
import cv2
import demo_scan_sort_spaced as SP
C = SP.configure()
import dual_zone as D
import scan_sort_bins as B
import demo_scan_sort as S
from scan_sort_camera import fit_mask, screw_outline, RES, WIDTH, CENTER

px = lambda pts: np.column_stack(((pts[:, 0]-CENTER[0])/WIDTH*RES+RES/2-.5,
                                  RES/2-(pts[:, 1]-CENTER[1])/WIDTH*RES-.5))
screw_fits = []
for xy, yaw, head in (((.47, -.145), 0., True), ((.15, .24), 90., False), ((.3, .1), 33., True)):
    mask = np.zeros((RES, RES), np.uint8)
    for poly in screw_outline(xy, yaw, head):
        cv2.fillConvexPoly(mask, np.round(px(poly)).astype(int), 1)
    fit = fit_mask(mask.astype(bool), 'screw')
    error = abs((fit['yaw_deg']-yaw+90) % 180-90)
    screw_fits.append(dict(xy_error=float(np.linalg.norm(np.subtract(fit['xy'], xy))),
                           yaw_error=error, iou=fit['silhouette_iou']))

def square(edge, yaw):
    mask = np.zeros((RES, RES), np.uint8)
    a = np.radians(yaw)
    rot = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    corners = (np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])*edge/2)@rot.T+[.53, .01]
    cv2.fillConvexPoly(mask, np.round(px(corners)).astype(int), 1)
    return mask.astype(bool)

def rejects(mask, kind, size=None):
    try:
        fit_mask(mask, kind, size)
        return False
    except RuntimeError:
        return True

items = S.inventory()
print(json.dumps(dict(
    screw=[C.SCREW_LENGTH, C.SCREW_HEAD_D, C.SCREW_SHANK_D],
    cube_edges=sorted({p['size'] for p in items if p['category'] == 'cube'}),
    vial=[2*C.VIAL_RADIUS, C.VIAL_LENGTH], cap=[2*C.CAP_RADIUS, C.CAP_HEIGHT],
    jaw={'screw': D.jaw_target('screw'), 'cube16': D.jaw_target('cube', .016)},
    screw_fits=screw_fits,
    cube16_iou=fit_mask(square(.016, 15.), 'cube', .016)['silhouette_iou'],
    size_gate=[rejects(square(.016, 15.), 'cube', .010), rejects(square(.010, 15.), 'cube', .016)],
    counts={k: sum(p['category'] == k for p in items) for k in ('screw', 'cube', 'vial', 'cap')},
    groups=sorted({p['group'] for p in items}),
    destinations={p['prim']: p['place_xy'] for p in items},
    unwalled=sorted(B.NO_WALLS))))
'''


class LabObjectTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        result = subprocess.run([sys.executable, '-c', PROBE],
                                cwd=Path(__file__).resolve().parent,
                                env=dict(os.environ, FINGER_SET='dual_v2'),
                                text=True, capture_output=True, check=True)
        cls.data = json.loads(result.stdout.strip().splitlines()[-1])

    def test_cad_page_dimensions(self):
        self.assertEqual(self.data['screw'], [.030, .012, .006])
        self.assertEqual(self.data['cube_edges'], [.010, .016])
        self.assertEqual(self.data['vial'], [.02718, .203])
        self.assertAlmostEqual(self.data['cap'][0], .03175)
        self.assertAlmostEqual(self.data['cap'][1], .0157)

    def test_every_object_type_is_in_the_spaced_layout(self):
        self.assertEqual(self.data['counts'], {'screw': 2, 'cube': 7, 'vial': 2, 'cap': 2})
        self.assertIn('cube_large', self.data['groups'])
        self.assertIn('screw', self.data['groups'])

    def test_screw_and_16mm_cube_fit_the_narrow_zone(self):
        for kind, value in self.data['jaw'].items():
            with self.subTest(kind=kind):
                self.assertGreaterEqual(value, 0.)
                self.assertLessEqual(value, .006)

    def test_screw_silhouette_fit(self):
        for fit in self.data['screw_fits']:
            self.assertLess(fit['xy_error'], .0005)
            self.assertLess(fit['yaw_error'], 1.5)
            self.assertGreaterEqual(fit['iou'], .85)

    def test_cube_size_gate(self):
        self.assertGreaterEqual(self.data['cube16_iou'], .85)
        self.assertTrue(all(self.data['size_gate']))

    def test_destinations_are_distinct(self):
        slots = [tuple(xy) for xy in self.data['destinations'].values()]
        self.assertEqual(len(slots), len(set(slots)))
        self.assertEqual(self.data['unwalled'], ['cube_large', 'screw'])


if __name__ == '__main__':
    unittest.main()
