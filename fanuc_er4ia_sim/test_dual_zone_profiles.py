"""Host checks for the original and revised gripper profiles; no Isaac required."""
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
import scene_config as C
import dual_zone as D
from padded_fingers import jaw_for_object
from scan_sort_motion import boxes_overlap
from scan_sort_camera import fit_mask, RES, WIDTH
import scan_sort_bins as B

def jaw(kind, width=None):
    try:
        return D.jaw_target(kind, width)
    except ValueError:
        return None

# Tool X points down, tool Y closes along world Y, tool Z points along world X.
rotation = np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
target = np.array([0.,0.,C.SURFACE_DZ+.003])
flange = target - rotation @ D.tool_offset('vial')
root = np.array([C.TOOL_LENGTH-C.FINGER_LENGTH,0.,0.])
object_box = (np.array([0.,0.,C.SURFACE_DZ+C.VIAL_RADIUS]),np.eye(3),
              np.array([C.VIAL_LENGTH/2,C.VIAL_RADIUS,C.VIAL_RADIUS]))
collisions = []
for sign in (-1,1):
    primary = ('Primary',(C.FINGER_LENGTH,C.FINGER_WIDTH-C.PAD_FACE,C.FINGER_THICK),
               (C.FINGER_LENGTH/2,sign*(C.FINGER_WIDTH+C.PAD_FACE)/2,0.),False)
    for name,size,centre,_ in [primary]+D.extra_boxes(sign):
        if name not in ('Primary','Bridge'):
            continue
        local = root + centre + np.array([0.,sign*D.jaw_target('vial'),0.])
        box = (flange+rotation@local,rotation,np.asarray(size)/2)
        if boxes_overlap(box,object_box):
            collisions.append(name)

mask = np.zeros((RES,RES),np.uint8)
half = np.rint(np.array([C.VIAL_LENGTH,2*C.VIAL_RADIUS])/WIDTH*RES/2).astype(int)
cv2.rectangle(mask,(RES//2-int(half[0]),RES//2-int(half[1])),
              (RES//2+int(half[0]),RES//2+int(half[1])),1,-1)
observed = fit_mask(mask.astype(bool),'vial')
print(json.dumps(dict(profile=C.FINGER_SET,primary_gap=[2*C.PAD_FACE,2*(C.PAD_FACE+C.JAW_OPEN)],
    secondary_gap=[2*D.CAP_FACE,2*(D.CAP_FACE+C.JAW_OPEN)],stroke=C.JAW_STROKE,
    stock_width=C.FINGER_WIDTH,wide_stock=D.CAP_OUTER_WIDTH,
    backing=D.CAP_OUTER_WIDTH-D.CAP_FACE-C.PAD_T,bridge=D.BRIDGE_LENGTH,
    vial=[2*C.VIAL_RADIUS,C.VIAL_LENGTH],offsets={k:D.tool_offset(k).tolist() for k in ('cube','vial','cap')},
    fit={k:jaw(k) for k in ('screw','cube','vial','cap')},cube16=jaw('cube',.016),
    routed={k:jaw_for_object(k) for k in ('screw','cube','vial','cap')} if C.FINGER_SET=='dual_v2' else {},
    higher_cap=jaw_for_object('cap',.008) if C.FINGER_SET=='dual_v2' else None,
    rejects=[jaw('cube',v) is None for v in (.004,.018,float('nan'),float('inf'),-1)],
    invalid_category=jaw('unknown',.010) is None,
    inactive_collisions=collisions,measured_vial=observed,
    vial_bin=B.BINS['vial'][:2],vial_slots=B.SLOTS['vial'],
    stops=[(c.tolist(),s.tolist()) for _,_,c,s,_ in B.vial_stops()])))
'''


class DualZoneProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profiles = {}
        for profile in ('dual', 'dual_v2'):
            result = subprocess.run([sys.executable, '-c', PROBE],
                cwd=Path(__file__).resolve().parent,
                env=dict(os.environ, FINGER_SET=profile),
                text=True, capture_output=True, check=True)
            cls.profiles[profile] = json.loads(result.stdout)

    def test_original_profile_remains_available(self):
        old = self.profiles['dual']
        self.assertEqual(old['primary_gap'], [.008,.020])
        self.assertAlmostEqual(old['secondary_gap'][0], .014)
        self.assertAlmostEqual(old['secondary_gap'][1], .026)
        self.assertEqual(old['vial'], [.017,.140])
        self.assertEqual(old['stock_width'], .012)
        self.assertEqual(old['bridge'], .020)
        self.assertEqual(old['vial_slots'], [[.23,.35],[.23,.43]])
        self.assertIsNone(old['fit']['screw'])

    def test_revision_uses_real_vial_and_unchanged_actuator_stroke(self):
        revised = self.profiles['dual_v2']
        self.assertEqual(revised['vial'], [.02718,.203])
        self.assertEqual(revised['stroke'], .006)
        self.assertAlmostEqual(revised['primary_gap'][0], .0052)
        self.assertAlmostEqual(revised['primary_gap'][1], .0172)
        self.assertAlmostEqual(revised['secondary_gap'][0], .018)
        self.assertAlmostEqual(revised['secondary_gap'][1], .030)

    def test_final_cad_stock_and_bridge_contract(self):
        revised = self.profiles['dual_v2']
        self.assertEqual(revised['stock_width'], .01845)
        self.assertEqual(revised['wide_stock'], .01845)
        self.assertGreaterEqual(revised['backing'], .004)
        self.assertAlmostEqual(revised['bridge'], .0283)

    def test_all_supported_diameters_have_an_in_stroke_jaw_target(self):
        revised = self.profiles['dual_v2']
        for kind, value in revised['fit'].items():
            with self.subTest(kind=kind):
                self.assertIsNotNone(value)
                self.assertGreaterEqual(value, 0.)
                self.assertLessEqual(value, .006)
        self.assertAlmostEqual(revised['fit']['screw'], .0001)
        self.assertAlmostEqual(revised['fit']['vial'], .00429)
        self.assertAlmostEqual(revised['cube16'], .0051)
        self.assertEqual(revised['routed'], revised['fit'])
        self.assertLess(revised['higher_cap'], revised['fit']['cap'])

    def test_invalid_sizes_are_rejected(self):
        self.assertTrue(all(self.profiles['dual_v2']['rejects']))
        self.assertTrue(self.profiles['dual_v2']['invalid_category'])

    def test_virtual_vial_center_has_separate_contact_offset(self):
        offsets = self.profiles['dual_v2']['offsets']
        self.assertAlmostEqual(offsets['cube'][2], 0.)
        self.assertAlmostEqual(offsets['cap'][2], .040)
        self.assertAlmostEqual(offsets['vial'][2], .1215)
        self.assertAlmostEqual(offsets['vial'][2]-offsets['cap'][2], .0815)

    def test_inactive_primary_and_bridge_clear_the_real_vial(self):
        self.assertEqual(self.profiles['dual_v2']['inactive_collisions'], [])

    def test_rendered_mask_size_gate_tracks_the_selected_profile(self):
        for profile, data in self.profiles.items():
            with self.subTest(profile=profile):
                measured = data['measured_vial']
                self.assertAlmostEqual(measured['measured_length_mm'], data['vial'][1]*1000, delta=.5)
                self.assertAlmostEqual(measured['measured_width_mm'], data['vial'][0]*1000, delta=.5)

    def test_vial_fits_revised_bin_and_anti_roll_stops(self):
        revised = self.profiles['dual_v2']
        centre, size = revised['vial_bin']
        radius, length = revised['vial'][0]/2, revised['vial'][1]
        for x,y in revised['vial_slots']:
            self.assertLess(abs(x-centre[0])+length/2, size[0]/2)
            self.assertLess(abs(y-centre[1])+radius, size[1]/2)
        for position, stop_size in revised['stops']:
            slot_y = min(revised['vial_slots'], key=lambda p: abs(p[1]-position[1]))[1]
            inner_clearance = abs(position[1]-slot_y)-stop_size[1]/2-radius
            self.assertAlmostEqual(inner_clearance, .0025)


if __name__ == '__main__':
    unittest.main()
