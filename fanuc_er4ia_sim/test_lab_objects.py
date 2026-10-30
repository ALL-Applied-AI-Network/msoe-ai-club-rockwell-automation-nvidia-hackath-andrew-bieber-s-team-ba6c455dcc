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
from scan_sort_camera import fit_mask, screw_outline, RES, WIDTH, CENTER, mask_centroid, verify_placement

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

def resting_screw(xy, yaw, tilt=6.):
    # Top-down outline of the real 3D screw resting on its head rim (axis ~6 deg tilt).
    mask = np.zeros((RES, RES), np.uint8)
    hr, sr, hh, L = C.SCREW_HEAD_D/2, C.SCREW_SHANK_D/2, C.SCREW_HEAD_H, C.SCREW_LENGTH
    a = np.linspace(0, 2*np.pi, 48, endpoint=False)
    ring = lambda r, z: np.column_stack([r*np.cos(a), r*np.sin(a), np.full_like(a, z)])
    t, w = np.radians(tilt), np.radians(yaw)
    lay = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    tip = np.array([[np.cos(t), 0, -np.sin(t)], [0, 1, 0], [np.sin(t), 0, np.cos(t)]])
    turn = np.array([[np.cos(w), -np.sin(w), 0], [np.sin(w), np.cos(w), 0], [0, 0, 1]])
    for pts in (np.vstack([ring(hr, 0), ring(sr, hh)]), np.vstack([ring(sr, hh), ring(sr, L)])):
        xy2 = (turn@tip@lay@(pts-[0, 0, L/2]).T).T[:, :2]+xy
        cv2.fillConvexPoly(mask, cv2.convexHull(np.round(px(xy2)).astype(np.int32)), 1)
    return mask.astype(bool)

resting = []
for yaw in (7., 21., 84., 98., 126., 161., 168.):  # angles rejected before the fix
    fit = fit_mask(resting_screw((.35, .05), yaw), 'screw')
    resting.append(dict(iou=fit['silhouette_iou'], yaw_error=abs((fit['yaw_deg']-yaw+90) % 180-90),
                        xy_error=float(np.linalg.norm(np.subtract(fit['xy'], (.35, .05))))))
screw_fits += resting

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

def blob(xy, edge, yaw=0.):
    return mask_centroid(square_at(xy, edge, yaw))

def square_at(xy, edge, yaw):
    mask = np.zeros((RES, RES), np.uint8)
    a = np.radians(yaw)
    rot = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    corners = (np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]])*edge/2)@rot.T+xy
    cv2.fillConvexPoly(mask, np.round(px(corners)).astype(int), 1)
    return mask.astype(bool)

blue = dict(bin='blue', place_xy=list(B.drop_xy('blue', 0)))
bx, by = B.REAL_BINS['blue'][0]
vial = next(p for p in items if p['category'] == 'vial')
camera = dict(
    in_bin=verify_placement(blob((bx+.009, by-.009), .010), blue)[0],
    tumbled_in_corner=verify_placement(blob((bx+.011, by+.011), .010, 45.), blue)[0],
    beside_bin=verify_placement(blob((bx+.040, by), .010), blue)[0],
    wrong_bin=verify_placement(blob(B.REAL_BINS['red'][0], .010), blue)[0],
    missing=verify_placement(mask_centroid(np.zeros((RES, RES), bool)), blue)[0],
    vial_on_slot=verify_placement(dict(found=True, xy=[vial['place_xy'][0]+.002, vial['place_xy'][1]]), vial)[0],
    vial_off_slot=verify_placement(dict(found=True, xy=[vial['place_xy'][0], vial['place_xy'][1]+.010]), vial)[0],
    centroid_error=float(np.linalg.norm(np.subtract(blob((.3, .1), .016)['xy'], (.3, .1)))))
import fk
from scan_sort_motion import recovery_steps, finger_boxes
park = S.park_pose()
vial = next(p for p in items if p['category'] == 'vial')
carry = dict(S.plan_any(vial, park)[1])['carry']
frames = recovery_steps(carry[len(carry)//2][0], park, 'vial')
recovery = dict(ends_at_park=bool(np.allclose(frames[-1][0], park)),
                jaws_open=all(j == C.JAW_OPEN for _, j in frames),
                max_step_deg=float(np.degrees(np.abs(np.diff([f[0] for f in frames], axis=0))).max()),
                lowest_mm=float(min((c-np.abs(r)@h)[2]-C.SURFACE_DZ for q, j in frames
                                    for _, (c, r, h) in finger_boxes(q, j))*1000))
print(json.dumps(dict(recovery=recovery, camera=camera,
    screw=[C.SCREW_LENGTH, C.SCREW_HEAD_D, C.SCREW_SHANK_D],
    cube_edges=sorted({p['size'] for p in items if p['category'] == 'cube'}),
    vial=[2*C.VIAL_RADIUS, C.VIAL_LENGTH], cap=[2*C.CAP_RADIUS, C.CAP_HEIGHT],
    jaw={'screw': D.jaw_target('screw'), 'cube16': D.jaw_target('cube', .016)},
    screw_fits=screw_fits,
    cube16_iou=fit_mask(square(.016, 15.), 'cube', .016)['silhouette_iou'],
    size_gate=[rejects(square(.016, 15.), 'cube', .010), rejects(square(.010, 15.), 'cube', .016)],
    counts={k: sum(p['category'] == k for p in items) for k in ('screw', 'cube', 'vial', 'cap')},
    groups=sorted({p['group'] for p in items}),
    destinations={p['prim']: dict(xy=p['place_xy'], bin=p.get('bin'), group=p['group'])
                  for p in items},
    bins=dict(names=sorted(B.REAL_BINS), outer=B.BIN_OUTER, height=B.BIN_HEIGHT,
              pocket=B.BIN_POCKET, depth=B.BIN_DEPTH,
              boxes=sum(1 for g, *_ in B.walls() if g.startswith('bin_')),
              centre_in=B.in_bin([.16, -.36, C.TABLE_TOP_Z+C.SURFACE_DZ+B.BIN_FLOOR+.005], 'yellow'),
              wall_out=B.in_bin([.16+.02, -.36, C.TABLE_TOP_Z+C.SURFACE_DZ+B.BIN_FLOOR+.005], 'yellow'),
              on_rim_out=B.in_bin([.16, -.36, C.TABLE_TOP_Z+C.SURFACE_DZ+B.BIN_HEIGHT+.015], 'yellow'),
              release_z=B.release_tip_height(),
              stacked_in=B.in_bin([.36, -.36, C.TABLE_TOP_Z+C.SURFACE_DZ+.058], 'blue'),
              blue_drops=[list(B.drop_xy('blue', k)) for k in range(3)]))))
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

    def test_recovery_backs_off_to_the_scan_pose(self):
        r = self.data['recovery']
        self.assertTrue(r['ends_at_park'])
        self.assertTrue(r['jaws_open'])
        self.assertLess(r['max_step_deg'], 2.)
        self.assertGreater(r['lowest_mm'], 50.)  # failure mid-carry: stays high

    def test_cad_bin_geometry(self):
        b = self.data['bins']
        self.assertEqual((b['outer'], b['height'], b['pocket'], b['depth']), (.050, .055, .035, .025))
        self.assertEqual(b['boxes'], 5*len(b['names']))  # base + 4 walls each
        self.assertTrue(b['centre_in'])
        self.assertFalse(b['wall_out'])
        self.assertFalse(b['on_rim_out'])
        self.assertGreater(b['release_z'], b['height'])  # fingers stay above the rim
        self.assertTrue(b['stacked_in'])  # 16 mm cube resting on two cubes still counts

    def test_camera_verification_verdicts(self):
        c = self.data['camera']
        self.assertTrue(c['in_bin'])
        self.assertTrue(c['tumbled_in_corner'])
        self.assertFalse(c['beside_bin'])
        self.assertFalse(c['wrong_bin'])
        self.assertFalse(c['missing'])
        self.assertTrue(c['vial_on_slot'])
        self.assertFalse(c['vial_off_slot'])
        self.assertLess(c['centroid_error'], .0005)

    def test_cube_drops_do_not_overlap_and_clear_the_walls(self):
        (a, b, c), half = self.data['bins']['blue_drops'], self.data['bins']['pocket']/2
        centre = (.36, -.36)
        for xy, edge in ((a, .010), (b, .010), (c, .016)):
            for k in (0, 1):
                self.assertLessEqual(abs(xy[k]-centre[k])+edge/2, half-.0015)
        def apart(p, q, e1, e2):
            return any(abs(p[k]-q[k]) >= (e1+e2)/2 for k in (0, 1))
        self.assertTrue(apart(a, b, .010, .010) and apart(a, c, .010, .016) and apart(b, c, .010, .016))

    def test_every_object_that_fits_goes_into_a_bin(self):
        dest = self.data['destinations']
        for prim, d in dest.items():
            with self.subTest(prim=prim):
                if 'Vial' in prim:
                    self.assertIsNone(d['bin'])  # 203 mm vial does not fit a 35 mm pocket
                else:
                    self.assertIsNotNone(d['bin'])
        self.assertEqual(dest['/World/SortCube6']['bin'], 'blue')  # 16 mm blue cube
        caps = [d['bin'] for prim, d in dest.items() if 'Cap' in prim]
        self.assertEqual(len(caps), len(set(caps)))  # one cap per pocket
        vials = [tuple(d['xy']) for prim, d in dest.items() if 'Vial' in prim]
        self.assertEqual(len(vials), len(set(vials)))

    def test_objects_fit_the_pocket(self):
        pocket = self.data['bins']['pocket']
        self.assertLess(self.data['screw'][0], pocket)
        self.assertLess(self.data['cap'][0], pocket)
        self.assertLess(max(self.data['cube_edges'])*2**.5, pocket)


if __name__ == '__main__':
    unittest.main()
