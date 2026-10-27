"""Synthetic geometric checks; no Isaac physics, SAM inference or robot access."""
from contextlib import ExitStack
import unittest
from unittest.mock import patch

import numpy as np

import sort_config as S
import gripper_profile as G
from dual_clearance import ClearanceError, check_path, polygons_overlap


def item(kind="cube", xy=(.35,0.), yaw=0., length=None, width=None):
    width = {"cube":10., "vial":17., "screw":6., "cap":31.75}[kind] if width is None else width
    length = {"cube":width, "vial":140., "screw":30., "cap":31.75}[kind] if length is None else length
    return dict(kind=kind, label="cube_red" if kind=="cube" else kind,
                world_xy=list(xy), yaw_base_deg=yaw, short_mm=width,
                long_mm=length, height_m={"cube":.010,"vial":.017,"screw":.006,"cap":.0157}[kind])


def pose(x=.35, y=0., height=.0025, r=0., grip=0.):
    return [x*1000,y*1000,(S.SURFACE_Z+height)*1000+S.UF0_Z_MM,-180.,0.,r,grip]


class DualClearanceTests(unittest.TestCase):
    def setUp(self):
        stack=ExitStack()
        self.addCleanup(stack.close)
        settings={
            "PHYSICAL_GRIPPER_PROFILE":"dual_zone",
            "PHYSICAL_VERIFIED_FIT_KINDS":("cube","screw"),
            "DUAL_ZONE_PRIMARY_OFFSET_TCP_M":(0.,0.,0.),
            "DUAL_ZONE_CAP_OFFSET_TCP_M":(.040,0.,0.),
            "DUAL_ZONE_CUBE_WIDTH_RANGE_MM":(9.,16.),
            "DUAL_ZONE_VIAL_DIAMETER_MM":17.,
            "DUAL_ZONE_VIAL_LENGTH_MM":140.,
            "DUAL_ZONE_CAP_NARROW_END_UP":True,
            "DUAL_ZONE_CLEARANCE_MM":2.,
            "DUAL_ZONE_MAX_OBSTACLE_HEIGHT_M":.045,
            "DUAL_ZONE_STATIC_OBSTACLES":(),
        }
        for key,value in settings.items():
            stack.enter_context(patch.object(S,key,value,create=True))

    def test_inactive_rail_hits_neighbour_despite_clear_active_grasp(self):
        target=item()
        neighbour=item(xy=(.39,0.))
        with self.assertRaisesRegex(ClearanceError,"Finger rail"):
            check_path([pose()],target,[target,neighbour])

    def test_opposite_yaw_places_inactive_rail_on_other_side(self):
        target=item()
        check_path([pose(r=180.)],target,[target,item(xy=(.39,0.))])

    def test_high_horizontal_travel_clears_low_objects(self):
        obstacles=[item(xy=(.39,0.))]
        check_path([pose(x=x,height=.18) for x in np.linspace(.30,.45,76)],item(),obstacles,carrying=True)

    def test_long_carried_vial_hits_object_beyond_both_rails(self):
        target=item("vial",xy=(.30,-.10))
        neighbour=item(xy=(.42,0.))
        with self.assertRaisesRegex(ClearanceError,"Carried object"):
            check_path([pose(x=.35,height=.003,grip=1.)],target,[neighbour],carrying=True)

    def test_payload_only_exists_on_closed_gripper_rows(self):
        target=item("vial",xy=(.30,-.10))
        neighbour=item(xy=(.42,0.))
        check_path([pose(x=.35,height=.003,grip=0.)],target,[neighbour],carrying=True)

    def test_selected_source_exclusion_does_not_remove_other_kind(self):
        target=item()
        check_path([pose()],target,[target])
        with self.assertRaises(ClearanceError):
            check_path([pose()],target,[item("screw")])

    def test_ungraspable_cap_is_still_an_obstacle(self):
        with patch.object(S,"DUAL_ZONE_CAP_NARROW_END_UP",False):
            with self.assertRaises(ClearanceError):
                check_path([pose()],item(),[item("cap",xy=(.39,0.))])

    def test_invalid_obstacle_fails_before_high_path_is_accepted(self):
        bad=item(xy=(float("nan"),0.))
        with self.assertRaises(ClearanceError):
            check_path([pose(height=.18)],item(),[bad])

    def test_static_rectangular_stop_blocks_descent(self):
        stop=dict(name="stop",bounds_xy=[[.387,-.01],[.395,.01]],height_m=.010)
        with self.assertRaisesRegex(ClearanceError,"stop"):
            check_path([pose()],item(),[stop])

    def test_low_tilt_and_table_penetration_are_rejected(self):
        tilted=pose();tilted[4]=10.
        with self.assertRaises(ClearanceError):
            check_path([tilted],item(),[item(xy=(.46,0.))])
        with self.assertRaisesRegex(ClearanceError,"table"):
            check_path([pose(height=0.)],item(),[])

    def test_tall_obstacle_exceeds_supported_bridge_model(self):
        tall=dict(name="tall fixture",bounds_xy=[[.45,.1],[.47,.12]],height_m=.08)
        with self.assertRaisesRegex(ClearanceError,"bridge"):
            check_path([pose(height=.18)],item(),[tall])

    def test_sat_detects_edge_clearance_and_separation(self):
        a=np.array([[0,0],[.01,0],[.01,.01],[0,.01]])
        self.assertTrue(polygons_overlap(a,a+[.011,0],.002))
        self.assertFalse(polygons_overlap(a,a+[.013,0],.002))

    def v2_vial(self, xy=(.40,0.), r=0.):
        target=item("vial",xy=xy,yaw=r,length=203.,width=27.18)
        target["height_m"] = .02718
        spec=G.grasp_spec(target)
        contact=G.object_to_contact_base(xy,r,spec)
        xyz=G.contact_to_tcp_base(contact,S.SURFACE_Z+spec["pick_height_m"],r,spec)
        p=[xyz[0]*1000,xyz[1]*1000,xyz[2]*1000+S.UF0_Z_MM,-180.,0.,r,1.]
        return target,spec,p

    def v2_context(self):
        stack=ExitStack()
        for key,value in {
            "PHYSICAL_GRIPPER_PROFILE":"dual_zone_v2",
            "DUAL_ZONE_V2_VIAL_DIAMETER_MM":27.18,
            "DUAL_ZONE_V2_VIAL_LENGTH_MM":203.,
            "DUAL_ZONE_V2_VIAL_END_INSET_M":.020,
        }.items():
            stack.enter_context(patch.object(S,key,value,create=True))
        return stack

    def test_v2_real_vial_far_end_is_screened_at_object_centre(self):
        with self.v2_context():
            target,spec,p=self.v2_vial()
            np.testing.assert_allclose(p[:2],[278.5,0.],atol=1e-8)
            np.testing.assert_allclose(G.tcp_to_object_xy(p,spec),[.40,0.],atol=1e-9)
            with self.assertRaisesRegex(ClearanceError,"Carried object"):
                check_path([p],target,[item(xy=(.49,0.))],carrying=True)

    def test_v2_near_end_grasp_keeps_inactive_primary_past_vial_end(self):
        with self.v2_context():
            target,spec,p=self.v2_vial()
            primary,wide=G.rail_rectangles(p)
            vial=G.object_rectangle(target)
            # Independent geometric expectation: near vial end at 298.5 mm;
            # primary rail's nearest edge at 281.5 mm, leaving 17 mm.
            self.assertAlmostEqual(vial[:,0].min()-primary[:,0].max(),.017)
            # Frozen v2 CAD has a shared outer backplane at +/-18.45 mm
            # closed. Add 6 mm travel on each side: 48.9 mm open envelope.
            self.assertAlmostEqual(np.ptp(primary[:,1]),.0489)
            self.assertAlmostEqual(np.ptp(wide[:,1]),.0489)
            check_path([p],target,[target],carrying=True)

    def test_v2_central_wide_grasp_rejects_inactive_primary_overlap(self):
        with self.v2_context():
            target,spec,_=self.v2_vial()
            spec={**spec,"contact_from_center_tcp_m":[0.,0.,0.]}
            xyz=G.contact_to_tcp_base(target["world_xy"],S.SURFACE_Z+spec["pick_height_m"],0.,spec)
            p=[xyz[0]*1000,xyz[1]*1000,xyz[2]*1000+S.UF0_Z_MM,-180.,0.,0.,1.]
            with patch.object(G,"grasp_spec",return_value=spec):
                with self.assertRaisesRegex(ClearanceError,"Inactive primary"):
                    check_path([p],target,[target],carrying=True)

    def test_v2_rotated_vial_far_end_rotates_with_tcp(self):
        with self.v2_context():
            target,spec,p=self.v2_vial(xy=(.35,0.),r=90.)
            with self.assertRaisesRegex(ClearanceError,"Carried object"):
                check_path([p],target,[item(xy=(.35,.09))],carrying=True)


if __name__=="__main__":
    unittest.main()
