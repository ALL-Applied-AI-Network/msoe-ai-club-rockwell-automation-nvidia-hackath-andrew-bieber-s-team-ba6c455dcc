"""Pure geometry checks for the physical dual-zone adapter; no hardware claims."""
from contextlib import ExitStack
import unittest
from unittest.mock import patch

import numpy as np

import gripper_profile as G
import sort_config as S


def target(kind="cube", **changes):
    value = dict(kind=kind, short_mm=10., long_mm=10., yaw_base_deg=0.,
                 world_xy=[.35, .1], height_m=.01)
    value.update(changes)
    return value


class GripperProfileTests(unittest.TestCase):
    def setUp(self):
        self.context = ExitStack()
        self.addCleanup(self.context.close)
        for key, value in dict(
            PHYSICAL_GRIPPER_PROFILE="dual_zone",
            TCP_R_FOR_AXIS_X=0.,
            PHYSICAL_VERIFIED_FIT_KINDS=("cube", "screw"),
            DUAL_ZONE_PRIMARY_OFFSET_TCP_M=(0., 0., .01),
            DUAL_ZONE_CAP_OFFSET_TCP_M=(.04, 0., 0.),
            DUAL_ZONE_CUBE_WIDTH_RANGE_MM=(9., 16.),
            DUAL_ZONE_VIAL_DIAMETER_MM=17., DUAL_ZONE_VIAL_LENGTH_MM=140.,
            DUAL_ZONE_CAP_NARROW_END_UP=True,
            DUAL_ZONE_PICK_HEIGHTS_M={"cube": .0025, "vial": .003, "cap": .005},
        ).items():
            self.context.enter_context(patch.object(S, key, value, create=True))

    def test_cap_plug_geometry_matches_taper_not_flange(self):
        result = G.grasp_spec(target("cap", short_mm=31.75))
        expected = 23.622 - (23.622-20.828) * (5.-2.2352)/(15.7-2.2352)
        self.assertAlmostEqual(result["width_mm"], expected)
        self.assertEqual(result["zone"], "cap")
        self.assertEqual(result["contact_offset_m"], [.04, 0., .01])
        self.assertEqual(result["release_height_m"], .005)
        self.assertLess(result["width_mm"]+2, 26)

    def test_cap_orientation_and_flange_height_are_required(self):
        with patch.object(S, "DUAL_ZONE_CAP_NARROW_END_UP", False):
            with self.assertRaisesRegex(ValueError, "narrow-end-up"):
                G.grasp_spec(target("cap", short_mm=31.75))
        with patch.object(S, "DUAL_ZONE_PICK_HEIGHTS_M", {"cap": .002}):
            with self.assertRaisesRegex(ValueError, "above the flange"):
                G.grasp_spec(target("cap", short_mm=31.75))

    def test_known_fit_cube_uses_declared_inventory_not_distorted_width(self):
        spec = G.grasp_spec(target(short_mm=22., long_mm=25.))
        self.assertEqual(spec["width_mm"], 16.)
        with patch.object(S, "PHYSICAL_VERIFIED_FIT_KINDS", ()):
            with self.assertRaisesRegex(ValueError, "opening clearance"):
                G.grasp_spec(target(short_mm=22.))
        with patch.object(S, "DUAL_ZONE_CUBE_WIDTH_RANGE_MM", (7., 16.)):
            with self.assertRaisesRegex(ValueError, "cannot be clamped"):
                G.grasp_spec(target())
        with patch.object(S, "DUAL_ZONE_CUBE_WIDTH_RANGE_MM", (9., 19.)):
            with self.assertRaisesRegex(ValueError, "opening clearance"):
                G.grasp_spec(target())

    def test_nonfinite_camera_width_never_bypasses(self):
        for value in (0., -1., float("inf"), float("nan")):
            with self.subTest(width=value), self.assertRaises(ValueError):
                G.grasp_spec(target(short_mm=value))

    def test_screw_shaft_rejected_even_if_old_profile_says_fits(self):
        with self.assertRaisesRegex(ValueError, "head grasp requires separate target logic"):
            G.grasp_spec(target("screw", short_mm=6., long_mm=30.))

    def test_vial_has_own_height_and_nominal_geometry(self):
        spec = G.grasp_spec(target("vial", short_mm=18., long_mm=145.))
        self.assertEqual(spec["width_mm"], 17.)
        self.assertEqual(spec["object_length_m"], .14)
        self.assertEqual(spec["pick_height_m"], .003)
        self.assertEqual(spec["release_height_m"], .003)
        self.assertEqual(spec["zone"], "primary")

    def test_contact_offset_rotates_and_roundtrips_through_uf0(self):
        spec = G.grasp_spec(target("cap", short_mm=31.75))
        for yaw, expected_xy in ((0, [.31,.1]), (90,[.35,.06]), (180,[.39,.1]), (-90,[.35,.14])):
            with self.subTest(yaw=yaw):
                tcp = G.contact_to_tcp_base([.35,.1], .02, yaw, spec)
                np.testing.assert_allclose(tcp[:2], expected_xy, atol=1e-12)
                self.assertAlmostEqual(tcp[2], .03)
                pose = [tcp[0]*1000,tcp[1]*1000,tcp[2]*1000+S.UF0_Z_MM,-180,0,yaw,1]
                np.testing.assert_allclose(G.tcp_to_contact_base(pose,spec), [.35,.1,.02], atol=1e-12)

    def test_two_rail_envelopes_rotate_with_actual_tcp_pose(self):
        pose = [350.,100.,-200.,-180.,0.,90.,0.]
        rails = G.rail_rectangles(pose)
        np.testing.assert_allclose(rails[0].mean(axis=0), [.35,.1], atol=1e-12)
        np.testing.assert_allclose(rails[1].mean(axis=0), [.35,.14], atol=1e-12)
        np.testing.assert_allclose(np.ptp(rails[0],axis=0), [.036,.006], atol=1e-12)
        widened = G.rail_rectangles(pose, .002)
        np.testing.assert_allclose(np.ptp(widened[0],axis=0), [.040,.010], atol=1e-12)

    def test_obstacles_do_not_require_grasp_eligibility(self):
        with patch.object(S, "DUAL_ZONE_CAP_NARROW_END_UP", False):
            cap = G.object_rectangle(target("cap", short_mm=30., long_mm=30.))
            np.testing.assert_allclose(np.ptp(cap,axis=0), [.03175,.03175], atol=1e-12)
        screw = G.object_rectangle(target("screw", short_mm=6., long_mm=30., yaw_base_deg=90.))
        np.testing.assert_allclose(np.ptp(screw,axis=0), [.006,.030], atol=1e-12)
        oversized = G.object_rectangle(target(short_mm=22., long_mm=25.))
        np.testing.assert_allclose(np.ptp(oversized,axis=0), [.025,.022], atol=1e-12)

    def test_legacy_offsets_and_width_override_remain_original(self):
        with patch.object(S,"PHYSICAL_GRIPPER_PROFILE","legacy"):
            spec = G.grasp_spec(target("screw", short_mm=16., long_mm=30.))
            self.assertEqual(spec["contact_offset_m"], [0.,0.,0.])
            self.assertEqual(spec["pick_height_m"], S.PICK_HEIGHT["screw"])
            self.assertEqual(spec["release_height_m"], S.DROP_HEIGHT)
            np.testing.assert_allclose(G.contact_to_tcp_base([.35,.1],.02,20,spec), [.35,.1,.02])
            with patch.object(S, "PHYSICAL_VERIFIED_FIT_KINDS", ()):
                with self.assertRaisesRegex(ValueError, "configured jaw clearance"):
                    G.grasp_spec(target("screw", short_mm=16.))


class RevisedGripperProfileTests(unittest.TestCase):
    def setUp(self):
        GripperProfileTests.setUp(self)
        for name, value in {
            "PHYSICAL_GRIPPER_PROFILE": "dual_zone_v2",
            "DUAL_ZONE_PRIMARY_OFFSET_TCP_M": (0.,0.,0.),
            "DUAL_ZONE_V2_VIAL_DIAMETER_MM": 27.18,
            "DUAL_ZONE_V2_VIAL_LENGTH_MM": 203.,
            "DUAL_ZONE_V2_VIAL_END_INSET_M": .020,
            "DUAL_ZONE_V2_SCREW_DIAMETER_MM": 6.,
            "DUAL_ZONE_V2_SCREW_LENGTH_MM": 30.,
        }.items():
            self.context.enter_context(patch.object(S, name, value, create=True))

    def test_v2_inventory_fits_declared_gaps_and_uses_correct_rails(self):
        for kind, measured, zone in (("screw",12.,"primary"), ("cube",22.,"primary"),
                                     ("vial",29.,"wide"), ("cap",31.75,"wide")):
            with self.subTest(kind=kind):
                spec = G.grasp_spec(target(kind, short_mm=measured))
                self.assertEqual(spec["profile"], "dual_zone_v2")
                self.assertEqual(spec["zone"], zone)
                gap = G.V2_PRIMARY_GAP_MM if zone == "primary" else G.V2_WIDE_GAP_MM
                clearance = .5 if zone == "primary" else 1.
                self.assertGreaterEqual(spec["width_mm"], gap[0]+.6)
                self.assertLessEqual(spec["width_mm"]+2*clearance, gap[1])
                self.assertEqual(spec["pick_height_m"], spec["release_height_m"])
        screw = G.grasp_spec(target("screw", short_mm=12.))
        self.assertEqual(screw["width_mm"], 6.)
        self.assertEqual(screw["object_length_m"], .03)
        self.assertEqual(screw["pick_height_m"], .0025)
        vial = G.grasp_spec(target("vial", short_mm=27.18))
        self.assertEqual(vial["object_length_m"], .203)
        self.assertEqual(vial["object_height_m"], .02718)

    def test_all_supported_cube_sizes_fit_without_using_camera_override(self):
        with patch.object(S, "PHYSICAL_VERIFIED_FIT_KINDS", ()):
            for width in (9., 10., 11., 16.):
                with self.subTest(width=width):
                    self.assertEqual(G.grasp_spec(target(short_mm=width))["width_mm"], width)
            with self.assertRaisesRegex(ValueError, "opening clearance"):
                G.grasp_spec(target(short_mm=16.3))
            with self.assertRaisesRegex(ValueError, "cannot be clamped"):
                G.grasp_spec(target("screw", short_mm=5.7))

    def test_vial_end_grasp_maps_object_centre_and_release_correctly(self):
        spec = G.grasp_spec(target("vial", short_mm=27.18))
        np.testing.assert_allclose(spec["contact_from_center_tcp_m"], [-.0815,0.,0.], atol=1e-12)
        for xy in ([.4,.05], [.3,-.2]):
            for yaw in (0., 90., -90., 180., 37.):
                with self.subTest(xy=xy,yaw=yaw):
                    contact = G.object_to_contact_base(xy, yaw, spec)
                    tcp = G.contact_to_tcp_base(contact, .003, yaw, spec)
                    pose = [tcp[0]*1000,tcp[1]*1000,tcp[2]*1000+S.UF0_Z_MM,-180.,0.,yaw,1.]
                    np.testing.assert_allclose(G.tcp_to_object_xy(pose,spec),xy,atol=1e-12)
                    self.assertAlmostEqual(np.linalg.norm(np.asarray(xy)-tcp[:2]), .1215)
        contact = G.object_to_contact_base([.4,.05],0.,spec)
        tcp = G.contact_to_tcp_base(contact,.003,0.,spec)
        np.testing.assert_allclose(contact,[.3185,.05],atol=1e-12)
        np.testing.assert_allclose(tcp[:2],[.2785,.05],atol=1e-12)
        vial_end = .4 - .203/2
        self.assertAlmostEqual(vial_end-(tcp[0]+.003), .017)

    def test_end_grasp_follows_measured_offset_sign(self):
        with patch.object(S,"DUAL_ZONE_CAP_OFFSET_TCP_M",(-.04,0.,0.)):
            spec = G.grasp_spec(target("vial",short_mm=27.18))
            contact = G.object_to_contact_base([.4,.05],0.,spec)
            tcp = G.contact_to_tcp_base(contact,.003,0.,spec)
            np.testing.assert_allclose(contact,[.4815,.05],atol=1e-12)
            np.testing.assert_allclose(tcp[:2],[.5215,.05],atol=1e-12)

    def test_v2_footprints_match_final_outer_stock_and_real_vial(self):
        rails = G.rail_rectangles([300.,0.,-200.,-180.,0.,0.,0.])
        for rail in rails:
            np.testing.assert_allclose(np.ptp(rail,axis=0), [.006,.0489], atol=1e-12)
        vial = G.object_rectangle(target("vial",short_mm=17.,long_mm=140.))
        np.testing.assert_allclose(np.ptp(vial,axis=0), [.203,.02718], atol=1e-12)

    def test_v2_invalid_nominal_dimensions_and_nonfinite_width_rejected(self):
        with patch.object(S,"DUAL_ZONE_V2_VIAL_DIAMETER_MM",29.):
            with self.assertRaisesRegex(ValueError,"opening clearance"):
                G.grasp_spec(target("vial",short_mm=27.18))
        with patch.object(S,"DUAL_ZONE_V2_VIAL_END_INSET_M",.11):
            with self.assertRaisesRegex(ValueError,"less than half"):
                G.grasp_spec(target("vial",short_mm=27.18))
        for kind in ("screw","cube","vial","cap"):
            with self.subTest(kind=kind), self.assertRaisesRegex(ValueError,"finite and positive"):
                G.grasp_spec(target(kind,short_mm=float("nan")))

    def test_measured_offset_and_yaw_calibration_must_be_consistent(self):
        with patch.object(S,"TCP_R_FOR_AXIS_X",90.):
            with self.assertRaisesRegex(ValueError,"object long axis"):
                G.grasp_spec(target("vial",short_mm=27.18))
            with patch.object(S,"DUAL_ZONE_CAP_OFFSET_TCP_M",(0.,.04,0.)):
                spec = G.grasp_spec(target("vial",short_mm=27.18))
                contact = G.object_to_contact_base([.4,.05],90.,spec)
                tcp = G.contact_to_tcp_base(contact,.003,90.,spec)
                np.testing.assert_allclose(contact,[.3185,.05],atol=1e-12)
                np.testing.assert_allclose(tcp[:2],[.2785,.05],atol=1e-12)
                pose=[tcp[0]*1000,tcp[1]*1000,tcp[2]*1000+S.UF0_Z_MM,-180.,0.,90.,1.]
                np.testing.assert_allclose(G.tcp_to_object_xy(pose,spec),[.4,.05],atol=1e-12)

    def test_measured_rail_separation_and_calibration_finiteness_checked(self):
        for offset in ((.038,0.,0.),(.042,0.,0.),(.04,0.,.001),(float("nan"),0.,0.)):
            with self.subTest(offset=offset), patch.object(S,"DUAL_ZONE_CAP_OFFSET_TCP_M",offset):
                with self.assertRaises(ValueError):
                    G.grasp_spec(target())
        for yaw in (float("nan"),float("inf")):
            with self.subTest(yaw=yaw), patch.object(S,"TCP_R_FOR_AXIS_X",yaw):
                with self.assertRaisesRegex(ValueError,"finite"):
                    G.grasp_spec(target())
        with patch.object(S,"TCP_R_FOR_AXIS_X",.5):
            G.grasp_spec(target())
        with patch.object(S,"TCP_R_FOR_AXIS_X",1.1):
            with self.assertRaisesRegex(ValueError,"within 1 degree"):
                G.grasp_spec(target())


if __name__ == "__main__":
    unittest.main()
