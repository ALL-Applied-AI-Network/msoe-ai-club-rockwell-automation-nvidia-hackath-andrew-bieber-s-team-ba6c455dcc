"""Real calibrated RGB-region checks and synthetic physical-camera detection.

Only the SAM mask producer is replaced. Projection, mask descriptions,
classification, candidate construction and the policy's RGB guard run as-is.
No model weights, Isaac scene labels or physical robot are used.
"""
from contextlib import ExitStack
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from calibration.geometry import project
import gripper_profile as G
from policy import Policy
import sort_config as S
from sort_vision import describe_masks
from test_dual_policy import detection


class DualPixelTests(unittest.TestCase):
    def setUp(self):
        stack=ExitStack()
        self.addCleanup(stack.close)
        directory=stack.enter_context(tempfile.TemporaryDirectory())
        for key,value in {
            "ROOT":Path(directory), "PHYSICAL_GRIPPER_PROFILE":"dual_zone_v2",
            "PHYSICAL_ALLOWED_KINDS":("screw","cube","vial","cap"),
            "PHYSICAL_VERIFIED_FIT_KINDS":("screw","cube"),
            "DUAL_ZONE_PRIMARY_OFFSET_TCP_M":(0.,0.,0.),
            "DUAL_ZONE_CAP_OFFSET_TCP_M":(.040,0.,0.),
            "DUAL_ZONE_CAP_NARROW_END_UP":True,
            "DUAL_ZONE_V2_VIAL_DIAMETER_MM":27.18,
            "DUAL_ZONE_V2_VIAL_LENGTH_MM":203.,
            "DUAL_ZONE_V2_VIAL_END_INSET_M":.020,
            "PHYSICAL_INSPECT_DROP_AREAS":True,
            "TCP_R_FOR_AXIS_X":0.,
        }.items():
            stack.enter_context(patch.object(S,key,value,create=True))
        self.policy=Policy()
        self.policy.mock=False
        self.addCleanup(self.policy.close)
        self.policy.log=lambda *args,**kwargs:None
        self.reference=np.full((720,1280,3),200,np.uint8)
        self.policy.batch_reference=self.reference.copy()
        self.policy.current_rgb=self.reference.copy()
        self.target=detection("cube_red",(.36,0.))
        self.target["grasp_spec"]=G.grasp_spec(self.target)
        spec=self.target["grasp_spec"]
        self.pick=self.policy.object_pose(self.target,self.target["world_xy"],
            S.SURFACE_Z+spec["pick_height_m"],0.,0.,spec)
        self.drop=self.policy.object_pose(self.target,(.34,.20),
            S.SURFACE_Z+spec["release_height_m"],0.,1.,spec)

    def pixels(self,xy,height):
        xy=np.asarray(xy,float)
        world=np.column_stack((xy,np.full(len(xy),S.SURFACE_Z+height)))
        cam=self.policy.cameras["table"]
        return project(cam,world)*np.array([1280,720])/np.asarray(cam["size"])

    def raster(self,polygon,height):
        mask=np.zeros((720,1280),np.uint8)
        cv2.fillConvexPoly(mask,np.rint(self.pixels(polygon,height)).astype(np.int32),1)
        return mask.astype(bool)

    def unchanged(self):
        return self.policy.dual_regions_unchanged(self.target,(self.pick,self.drop))

    def test_same_rgb_preserves_in_frame_pick_and_drop(self):
        self.assertTrue(self.unchanged())

    def test_inactive_source_rail_change_outside_object_bbox_forces_rescan(self):
        wide=G.rail_rectangles(self.pick,margin_m=.004)[1]
        changed=self.raster(wide,.010)
        obj_uv=self.pixels(G.object_rectangle(self.target),self.target["height_m"])
        lo=np.floor(obj_uv.min(axis=0)).astype(int)
        hi=np.ceil(obj_uv.max(axis=0)).astype(int)
        self.assertGreater(np.count_nonzero(changed),50)
        self.assertFalse(changed[lo[1]:hi[1]+1,lo[0]:hi[0]+1].any(),
                         "The disturbance must be outside the object's own image box")
        self.policy.current_rgb[changed]=0
        self.assertFalse(self.unchanged())

    def test_landing_region_change_forces_rescan(self):
        centre=G.tcp_to_object_xy(self.drop,self.target["grasp_spec"])
        changed=self.raster(G.object_rectangle(self.target,xy=centre,margin_m=.004),.010)
        self.assertGreater(np.count_nonzero(changed),50)
        self.policy.current_rgb[changed]=0
        self.assertFalse(self.unchanged())

    def test_change_far_outside_planned_regions_does_not_defer_target(self):
        self.policy.current_rgb[:30,:30]=0
        self.assertTrue(self.unchanged())

    def test_changed_frame_shape_is_rejected(self):
        self.policy.current_rgb=self.reference[:-1].copy()
        self.assertFalse(self.unchanged())

    def test_real_vial_mask_uses_declared_height_and_one_sam_pass(self):
        centre=np.array([.37,0.])
        yaw=20.
        angle=np.deg2rad(yaw)
        rotation=np.array([[np.cos(angle),np.sin(angle)],[-np.sin(angle),np.cos(angle)]])
        corners=np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*[.203/2,.02718/2]
        corners=corners@rotation+centre
        mask=self.raster(corners,.02718)
        rgb=np.full((720,1280,3),235,np.uint8)
        rgb[mask]=(175,175,175)
        records=describe_masks(rgb,[dict(segmentation=mask,predicted_iou=.95,stability_score=.96)])
        # No simulator identities/type labels are supplied to this interface.
        self.assertTrue(all("label" not in record and "kind" not in record for record in records))
        calls=[]

        class FakeSam:
            def detect(self,request):
                calls.append(request)
                return {"detections":records}

            def close(self):
                pass

        self.policy.sam=FakeSam()
        targets=self.policy.detect({"images":{"table":rgb}})
        self.assertEqual(len(calls),1)
        self.assertEqual(len(targets),1)
        self.assertEqual(targets[0]["label"],"vial")
        self.assertAlmostEqual(targets[0]["height_m"],.02718)
        self.assertLess(np.linalg.norm(np.asarray(targets[0]["world_xy"])-centre),.002)
        # Rasterized contour edges can add about one pixel on each side;
        # the tighter 2 mm requirement above applies to the grasp centre.
        self.assertAlmostEqual(targets[0]["long_mm"],203.,delta=3.)
        self.assertAlmostEqual(targets[0]["short_mm"],27.18,delta=3.)
        self.assertEqual(len(self.policy.observed_objects),1)
        self.assertEqual(self.policy.observed_objects[0]["label"],"vial")
        np.testing.assert_allclose(self.policy.observed_objects[0]["world_xy"],targets[0]["world_xy"])


if __name__=="__main__":
    unittest.main()
