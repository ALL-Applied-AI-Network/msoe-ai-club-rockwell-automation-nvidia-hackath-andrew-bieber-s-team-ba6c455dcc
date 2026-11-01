"""Focused host checks. No Isaac runtime, SAM weights or physical robot required."""
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from calibration.geometry import load, project, pixel_to_base
import sort_config as S
from sort_vision import build_candidates, describe_masks, Attempts, ScanQueue, target_unchanged
from policy import Policy, base_to_tcp, interpolate


def fixture(label, xy, long_mm, short_mm, angle, size=(1280,720)):
    import cv2
    cam = load()["table"]
    kind = label.split("_")[0]
    height = {"cube": .010, "screw": .006, "cap": .0157, "vial": .017, "misc": .008}[kind]
    if kind == "cap":
        t = np.linspace(0, 2*np.pi, 80, endpoint=False)
        points = np.column_stack((np.cos(t)*long_mm/2000,np.sin(t)*short_mm/2000))
    else:
        points = np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*[long_mm/2000,short_mm/2000]
    c, s = np.cos(np.radians(angle)), np.sin(np.radians(angle))
    points = points@np.array([[c,s],[-s,c]]) + xy
    xyz = np.column_stack((points,np.full(len(points),S.SURFACE_Z+height)))
    uv = project(cam,xyz)*np.array(size)/cam["size"]
    mask = np.zeros((size[1],size[0]),np.uint8)
    cv2.fillPoly(mask,[np.rint(uv).astype(np.int32)],1)
    rgb = np.full((*mask.shape,3),235,np.uint8)
    rgb[mask.astype(bool)] = {"cube_red":(230,20,20),"cube_blue":(20,50,220),
        "cube_yellow":(230,210,20),"screw":(45,45,45),"cap":(230,210,20),"vial":(190,190,190),
        "misc":(160,30,160)}[label]
    return rgb, [{"segmentation":mask.astype(bool),"predicted_iou":.95,"stability_score":.96}]


def projector(size):
    cam=load()["table"]
    def f(u,v,h):
        return pixel_to_base(cam, np.array([u,v])*cam["size"]/np.array(size), S.SURFACE_Z+h)[:2]
    return f


class SortingTests(unittest.TestCase):
    def setUp(self):
        mode = patch.object(S, "PHYSICAL_SCAN_MODE", "per_pick")
        mode.start()
        self.addCleanup(mode.stop)

    def test_classes_have_separate_drop_areas_within_radius(self):
        drops = {**S.DROP_XY, "misc": S.MISC_DROP_XY}
        self.assertEqual(set(S.BIN_FOR), {"cube_red", "cube_blue", "cube_yellow",
                                        "screw", "cap", "vial", "misc"})
        self.assertEqual(len(set(S.BIN_FOR.values())), 7)
        self.assertEqual(set(S.BIN_FOR.values()), set(drops))
        xmin, xmax, ymin, ymax = S.WORKSPACE
        points = list(drops.values())
        for i, (x, y) in enumerate(points):
            self.assertLess(np.hypot(x, y), S.MAX_RADIUS)
            self.assertFalse(xmin <= x <= xmax and ymin <= y <= ymax)
            for other in points[i+1:]:
                self.assertGreater(np.linalg.norm(np.array([x, y])-other), 2*S.DROP_RADIUS)

    def test_destination_routing_repairs_wrong_zones_and_leaves_correct_ones(self):
        drops = {**S.DROP_XY, "misc": S.MISC_DROP_XY}
        dimensions = {"cube_red": (10, 10, "red"), "cube_blue": (10, 10, "blue"),
                      "cube_yellow": (10, 10, "yellow"), "screw": (30, 6, "unknown"),
                      "cap": (31.75, 31.75, "yellow"), "vial": (140, 17, "unknown"),
                      "misc": (16, 7, "unknown")}
        for label, (length, width, color) in dimensions.items():
            for zone, xy in drops.items():
                with self.subTest(label=label, source_zone=zone):
                    center = np.array(xy)*1000
                    contour = np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*[length/2,width/2]+center
                    detection = {"id": 0, "center_full_px": center.tolist(),
                                 "contour_full_px": contour.tolist(), "color": color,
                                 "predicted_iou": .95, "stability_score": .96, "solidity": 1.0}
                    targets, rejected = build_candidates([detection],
                        lambda u,v,h: np.array([u,v])/1000,
                        drops=drops, inspect_drop_areas=True, allow_misc=True,
                        height_at=lambda u,v: .008)
                    if zone == S.BIN_FOR[label]:
                        self.assertEqual(targets, [])
                        self.assertIn("already in correct destination", rejected[0]["reason"])
                    else:
                        self.assertEqual(len(targets), 1, rejected)
                        self.assertEqual(targets[0]["label"], label)
                        self.assertEqual(targets[0]["source_zone"], zone)
                        self.assertEqual(targets[0]["drop"], S.BIN_FOR[label])

    def test_destination_inspection_does_not_enable_arbitrary_table_pickups(self):
        rgb, masks = fixture("screw", [.15, -.12], 30, 6, 0)
        targets, rejected = build_candidates(describe_masks(rgb, masks), projector((1280,720)),
                                            inspect_drop_areas=True)
        self.assertEqual(targets, [])
        self.assertTrue(any("outside pickup area" in r["reason"] for r in rejected))

    def test_misc_needs_small_shape_and_positive_depth(self):
        rgb, masks = fixture("misc", [.39, 0], 16, 7, 25)
        detections = describe_masks(rgb, masks)
        project_xy = projector((1280,720))
        self.assertEqual(build_candidates(detections, project_xy)[0], [])
        self.assertEqual(build_candidates(detections, project_xy, allow_misc=True)[0], [])
        for height in (0.0, .003, .030, float("nan")):
            self.assertEqual(build_candidates(detections, project_xy, allow_misc=True,
                height_at=lambda u,v: height)[0], [])
        targets, rejected = build_candidates(detections, project_xy, allow_misc=True,
            height_at=lambda u,v: .008)
        self.assertEqual(len(targets), 1, rejected)
        self.assertEqual((targets[0]["kind"], targets[0]["drop"]), ("misc", "misc"))
        np.testing.assert_allclose(targets[0]["world_xy"], [.39, 0], atol=.003)
        for length, width in ((50, 7), (16, 16)):
            rgb, masks = fixture("misc", [.39, 0], length, width, 0)
            self.assertEqual(build_candidates(describe_masks(rgb, masks), project_xy,
                allow_misc=True, height_at=lambda u,v: .008)[0], [])

    def test_misc_cannot_relabel_a_known_object_fragment(self):
        detections = []
        for label, length, width in (("cap", 31.75, 31.75), ("misc", 16, 7)):
            rgb, masks = fixture(label, [.39, 0], length, width, 0)
            d = describe_masks(rgb, masks)[0]
            d["id"] = len(detections)
            detections.append(d)
        targets, rejected = build_candidates(detections, projector((1280,720)),
            allow_misc=True, height_at=lambda u,v: .008)
        self.assertEqual([t["kind"] for t in targets], ["cap"])
        self.assertTrue(any("overlaps" in r["reason"] for r in rejected))

    def test_scan_queue_invalidates_failures_neighbours_and_large_objects(self):
        a = {"kind":"screw", "world_xy":[.30,-.10]}
        near = {"kind":"cube", "world_xy":[.32,-.10]}
        far = {"kind":"cube", "world_xy":[.40,.10]}
        attempts = Attempts()
        queue = ScanQueue(3)
        queue.refresh([a, near, far])
        queue.finish(a, True)
        self.assertEqual(queue.available(attempts), [far])
        queue.finish(far, False)
        self.assertEqual(queue.available(attempts), [])
        queue.refresh([a,far])
        queue.finish({**a,"kind":"cap"}, True)
        self.assertEqual(queue.available(attempts), [])
        queue = ScanQueue(1)
        queue.refresh([a,far]); queue.finish(a,True)
        self.assertEqual(queue.available(attempts), [])

    def test_cached_target_is_invalidated_when_image_changes(self):
        rgb, masks = fixture("screw", [.39, 0], 30, 6, 37)
        target = describe_masks(rgb, masks)[0]
        self.assertTrue(target_unchanged(rgb, rgb.copy(), target))
        changed = rgb.copy()
        changed[masks[0]["segmentation"]] = 235
        self.assertFalse(target_unchanged(rgb, changed, target))

    def test_batch_attempts_all_targets_once_before_retrying(self):
        targets = [{"kind": kind, "world_xy": [.30+i*.03, .10]}
                   for i, kind in enumerate(("screw", "screw", "cube", "cap", "vial"))]
        attempts = Attempts()
        queue = ScanQueue(limit=1, whole_batch=True)
        queue.refresh(targets)
        for target in targets:
            self.assertIs(queue.available(attempts)[0], target)
            attempts.add(target)
            queue.finish(target, False)  # failure must not restart SAM mid-pass
        self.assertEqual(queue.available(attempts), [])
        queue.refresh(targets)  # fresh scan sees that all of them remain
        self.assertEqual(queue.available(attempts), targets)
        for target in targets:
            attempts.add(target)
            queue.finish(target, True)
        queue.refresh(targets)
        self.assertEqual(queue.available(attempts), [])  # no endless failed-pick loop

    def test_batch_changed_target_defers_without_discarding_neighbours(self):
        a = {"kind": "screw", "world_xy": [.30, .10]}
        b = {"kind": "cube", "world_xy": [.32, .10]}
        queue = ScanQueue(whole_batch=True)
        attempts = Attempts()
        queue.refresh([a, b])
        queue.discard(a)
        self.assertEqual(queue.available(attempts), [b])
        self.assertEqual(attempts.count(a), 0)  # no motion was attempted
        queue.finish(b, True)
        self.assertEqual(queue.available(attempts), [])
        queue.refresh([a])
        self.assertEqual(queue.available(attempts), [a])

    def test_watch_rearms_location_only_after_two_absent_full_scans(self):
        target = {"kind": "screw", "world_xy": [.30, .10]}
        attempts = Attempts()
        attempts.add(target); attempts.add(target)
        for _ in range(5):
            attempts.observe([target])
        self.assertEqual(attempts.available([target]), [])
        attempts.observe([])
        self.assertEqual(attempts.available([target]), [])
        attempts.observe([target])  # one missed detection cannot rearm it
        attempts.observe([])
        self.assertEqual(attempts.available([target]), [])
        attempts.observe([])
        self.assertEqual(attempts.available([target]), [target])

    def test_mask_memory_layouts(self):
        rgb, masks = fixture("screw", [.39, 0], 30, 6, 37)
        expected = describe_masks(rgb, masks)
        self.assertEqual(len(expected), 1)
        source = masks[0]["segmentation"]
        layouts = {
            "fortran": np.asfortranarray(source),
            "strided": np.repeat(source, 2, axis=1)[:, ::2],
        }
        for name, mask in layouts.items():
            with self.subTest(layout=name):
                self.assertFalse(mask.flags.c_contiguous)
                actual = describe_masks(rgb, [{**masks[0], "segmentation": mask}])
                self.assertEqual(actual, expected)

    def test_projection_resize_and_uf0(self):
        cam=load()["table"]
        point=np.array([.39,-.12,S.SURFACE_Z])
        uv=project(cam,[point])[0]
        for size in [(640,360),(1280,720),(1920,1080)]:
            small=uv*np.array(size)/cam["size"]
            np.testing.assert_allclose(projector(size)(*small,0),point[:2],atol=1e-8)
        self.assertAlmostEqual(base_to_tcp(.39,-.12,S.SURFACE_Z)[2],-335.1)

    def test_synthetic_masks_all_classes_and_rotated_cube(self):
        for label,length,width,yaw in [("screw",30,6,37),("cube_red",10,10,32),
            ("cube_blue",10,10,-23),("cube_yellow",10,10,0),("cap",31.75,31.75,0),
            ("vial",140,17,10)]:
            with self.subTest(label=label):
                rgb,masks=fixture(label,[.39,0],length,width,yaw)
                detections=describe_masks(rgb,masks)
                targets,rejected=build_candidates(detections,projector((1280,720)))
                self.assertEqual(len(targets),1,rejected)
                self.assertEqual(targets[0]["label"],label)
                self.assertEqual(targets[0]["drop"], S.BIN_FOR[label])
                np.testing.assert_allclose(targets[0]["world_xy"],[.39,0],atol=.003)
                if label.startswith("cube_"):
                    error=(targets[0]["yaw_base_deg"]-yaw+45)%90-45
                    self.assertLess(abs(error),12)  # discretized small masks, not subpixel ground truth

    def test_size_order_drop_exclusion_and_attempt_ids(self):
        detections=[]
        for i,(label,xy,length,width) in enumerate([
            ("vial",[.40,.12],140,17),("cap",[.40,-.10],31.75,31.75),
            ("cube_red",[.32,0],10,10),("screw",[.32,-.15],30,6)]):
            rgb,masks=fixture(label,xy,length,width,0)
            d=describe_masks(rgb,masks)[0];d["id"]=i;detections.append(d)
        targets,_=build_candidates(detections,projector((1280,720)))
        self.assertEqual([t["kind"] for t in targets],["screw","cube","cap","vial"])
        a=Attempts();a.add(targets[0]);a.add({**targets[0],"id":999})
        self.assertEqual([t["kind"] for t in a.available(targets)],["cube","cap","vial"])
        blocked,_=build_candidates(detections,projector((1280,720)),drops={"test":[.32,-.15]})
        self.assertNotIn("screw",[t["kind"] for t in blocked])

    def test_chunk_limits_speed_and_gripper_timing(self):
        a=[180,-300,-35,-180,0,0,0];b=[450,200,-235,-180,0,80,1]
        chunks=interpolate(a,b)
        self.assertTrue(all(1<=len(c)<=100 for c in chunks))
        rows=np.array([a]+[r for c in chunks for r in c])
        self.assertLessEqual(np.linalg.norm(np.diff(rows[:,:3],axis=0),axis=1).max(),S.LINEAR_STEP_MM+1e-9)
        self.assertTrue(np.all(rows[:-1,6]==0));self.assertEqual(rows[-1,6],1)

    def test_preview_preserves_closed_gripper(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(S,"ROOT",Path(tmp)), patch.object(S,"PHYSICAL_EXECUTE",False):
            p=Policy();p.mock=False;p.detect=lambda obs:[]
            obs={"tcp_pose":[300,0,-100,-180,0,0],"joints":[0]*6+[1]}
            self.assertEqual(p.act(obs),obs["tcp_pose"]+[1.0])

    def test_missing_pose_confirmation_stops_motion(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(S,"ROOT",Path(tmp)), patch.object(S,"PHYSICAL_EXECUTE",True), patch.object(S,"PHYSICAL_POSES_CONFIRMED",False):
            p=Policy();p.mock=False
            obs={"tcp_pose":[300,0,-100,-180,0,0],"joints":[0]*7}
            self.assertEqual(p.act(obs),obs["tcp_pose"]+[0.0]);self.assertTrue(p.halted)

    def test_full_state_machine_and_large_object_pause(self):
        for kind,label in [("screw","screw"),("cap","cap")]:
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as tmp, \
                 patch.object(S,"ROOT",Path(tmp)), patch.object(S,"PHYSICAL_EXECUTE",True), \
                 patch.object(S,"PHYSICAL_POSES_CONFIRMED",True), \
                 patch.object(S,"PHYSICAL_ALLOWED_KINDS",("screw","cube","cap","vial")), \
                 patch.object(S,"PHYSICAL_MAX_GRASP_WIDTH_MM",40.0):
                p=Policy();p.mock=False
                target={"kind":kind,"label":label,"world_xy":[.39,0],"yaw_base_deg":30,
                        "drop":S.BIN_FOR[label],"short_mm":6.0 if kind=="screw" else 31.75}
                reads=iter([[target],[],[]]);p.detect=lambda obs:next(reads,[])
                obs={"tcp_pose":[300,0,-100,-180,0,0],"joints":[0]*7}
                held=False;released=False
                for _ in range(400):
                    result=np.asarray(p.act(obs),float).reshape(-1,7)
                    self.assertLessEqual(len(result),100)
                    for row in result:
                        Policy.validate(row)
                        if row[6]>=.5:held=True
                        elif held:released=True
                    obs["tcp_pose"]=result[-1,:6].tolist();obs["joints"][6]=result[-1,6]
                    if p.halted:break
                self.assertTrue(held and released)
                self.assertTrue(p.halted)
                if kind=="cap":self.assertEqual(p.state,"released")

    def test_controller_lag_cannot_advance_phase(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(S,"ROOT",Path(tmp)), patch.object(S,"PHYSICAL_EXECUTE",True), patch.object(S,"PHYSICAL_POSES_CONFIRMED",True):
            p=Policy();p.mock=False
            obs={"tcp_pose":[300,0,-250,-180,0,0],"joints":[0]*7}
            first=np.asarray(p.act(obs)).reshape(-1,7)[-1]
            second=np.array(p.act(obs))
            np.testing.assert_allclose(second,first)
            for _ in range(S.MAX_SETTLE_CALLS):p.act(obs)
            self.assertTrue(p.halted)


if __name__=="__main__":
    unittest.main()
