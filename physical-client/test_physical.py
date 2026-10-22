"""Physical-client regression checks; no network, model weights or real arm."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from calibration.geometry import load, project
from camera_check import check_frame_size, mapping_error
from policy import Policy, base_to_tcp, interpolate
from prepare_submission import write_submission
from mock_cell import validate_tcp, TCP_BOX
from sort_vision import describe_masks
import sort_config as S
from test_sorting import fixture


def target(kind="screw", label="screw", width=6.0):
    return {"kind":kind,"label":label,"world_xy":[.39,0.0],"yaw_base_deg":0.0,
            "drop":S.BIN_FOR[label],"short_mm":width}


class PhysicalTests(unittest.TestCase):
    def test_controller_box_is_enforced_before_queuing_or_selecting(self):
        self.assertEqual(tuple(TCP_BOX),S.PHYSICAL_TCP_BOX_MM)
        for xyz in ((149.9,0,-100),(300,450.1,-100),(300,0,-334.05)):
            pose=[*xyz,-180,0,0,0]
            with self.subTest(xyz=xyz):
                with self.assertRaises(ValueError): Policy.validate(pose)
                with self.assertRaises(ValueError): validate_tcp([pose],.1)
        outside=target('cube','cube_red',10);outside['world_xy']=[.12,.25]
        good=target()
        eligible,disabled=Policy.motion_targets([outside,good])
        self.assertEqual(eligible,[good])
        self.assertIn('action box',disabled[0]['reason'])

    def test_source_verification_moves_off_pickup_before_detection(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(S,"ROOT",Path(tmp)), \
             patch.object(S,"PHYSICAL_EXECUTE",True), patch.object(S,"PHYSICAL_POSES_CONFIRMED",True):
            p=Policy(); p.mock=False; p.state="carry"; p.selected=target()
            reads=[]
            p.detect=lambda obs: reads.append(list(obs['tcp_pose'])) or [target()]
            obs={'tcp_pose':base_to_tcp(.39,0,S.SURFACE_Z+S.TRAVEL_HEIGHT,37)[:6],
                 'joints':[0]*6+[1]}
            for _ in range(100):
                rows=np.asarray(p.act(obs),float).reshape(-1,7)
                # No release during the scan/verification move.
                if not reads:
                    self.assertTrue(np.all(rows[:,6]==1))
                    np.testing.assert_allclose(rows[:,5],37)
                obs['tcp_pose']=rows[-1,:6].tolist();obs['joints'][6]=rows[-1,6]
                if reads: break
            self.assertEqual(len(reads),1)
            np.testing.assert_allclose(reads[0][:3],
                base_to_tcp(*S.SCAN_XY,S.SURFACE_Z+S.SCAN_HEIGHT)[:3])
            # Failed-grasp return crosses at clearance, then descends vertically.
            recovery=np.vstack([rows,*list(p.queue)])
            for a,b in zip(recovery,recovery[1:]):
                if np.linalg.norm(b[:2]-a[:2])>1e-8:
                    self.assertGreaterEqual(min(a[2],b[2]),
                        base_to_tcp(0,0,S.SURFACE_Z+S.TRAVEL_HEIGHT)[2]-1e-8)

    def test_missed_pick_then_successful_retry_emits_controller_valid_rows(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(S,"ROOT",Path(tmp)), \
             patch.object(S,"PHYSICAL_EXECUTE",True),patch.object(S,"PHYSICAL_POSES_CONFIRMED",True):
            p=Policy();p.mock=False
            scans=iter([[target()],[target()],[target()],[],[]])
            p.detect=lambda obs: next(scans,[])
            events=[];p.log=lambda event,**data:events.append((event,data))
            obs={'tcp_pose':[300,0,-100,-180,0,0],'joints':[0]*7}
            action_count=0
            for _ in range(600):
                rows=np.asarray(p.act(obs),float).reshape(-1,7)
                validate_tcp(rows,.1)
                action_count+=len(rows)
                obs['tcp_pose']=rows[-1,:6].tolist();obs['joints'][6]=rows[-1,6]
                if p.halted:break
            self.assertTrue(p.halted)
            self.assertEqual(p.picks,2)
            self.assertEqual(sum(e=='source_still_detected' for e,_ in events),1)
            self.assertEqual(sum(e=='release_commanded' for e,_ in events),1)
            self.assertGreater(action_count,100)

    def test_invalid_later_pose_does_not_publish_partial_plan(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(S,"ROOT",Path(tmp)):
            p=Policy(); start=base_to_tcp(.3,0,S.SURFACE_Z+.2)
            with self.assertRaises(ValueError):
                p.enqueue(start,[base_to_tcp(.35,0,S.SURFACE_Z+.2),
                                 base_to_tcp(.8,0,S.SURFACE_Z+.2)],"carry")
            self.assertFalse(p.queue)
            self.assertEqual(p.state,"start")

    def test_settling_requires_consecutive_matches_after_lag(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(S,"ROOT",Path(tmp)):
            p=Policy(); end=base_to_tcp(.3,0,S.SURFACE_Z+.2)
            p.pending=end; obs={'tcp_pose':end[:6],'joints':[0]*7}
            for _ in range(2): self.assertEqual(p.motion(obs),end)
            lag={'tcp_pose':[end[0]-10,*end[1:6]],'joints':[0]*7}
            self.assertEqual(p.motion(lag),end)
            for _ in range(2): self.assertEqual(p.motion(obs),end)
            self.assertIsNone(p.motion(obs))

    def test_pose_bounds_and_wrapped_interpolation(self):
        for pose in ([float('nan')]*7,base_to_tcp(.55,0,S.SURFACE_Z+.2),
                     base_to_tcp(.3,0,S.SURFACE_Z),base_to_tcp(.3,0,S.SURFACE_Z+.451)):
            with self.subTest(pose=pose),self.assertRaises(ValueError): Policy.validate(pose)
        a=base_to_tcp(.3,0,S.SURFACE_Z+.2,179)
        b=base_to_tcp(.3,0,S.SURFACE_Z+.2,-179,1)
        rows=np.vstack(interpolate(a,b))
        self.assertLessEqual(np.max(np.abs(np.diff(np.r_[a[5],rows[:,5]]))),2.0)
        self.assertEqual(rows[-1,6],1)

    def test_published_camera_orientation_places_base_above_pickup(self):
        cam=load()["table"]
        uv=project(cam,[[0,0,S.SURFACE_Z],[.35,0,S.SURFACE_Z]])
        # Independent rounded coordinates from the organizer's calibration.
        np.testing.assert_allclose(uv,[[981.72,423.92],[979.61,713.27]],atol=.01)
        self.assertLess(uv[0,1],uv[1,1])

    def test_measured_landmarks_expose_an_accidental_180_degree_flip(self):
        cam=load()["table"]
        marks=[{"pixel":[979.61,713.27],"base_mm":[350,0]}]
        self.assertLess(mapping_error(marks,(1920,1080),cam)[0]["error_mm"],.02)
        flipped=[{"pixel":[1919-979.61,1079-713.27],"base_mm":[350,0]}]
        self.assertGreater(mapping_error(flipped,(1920,1080),cam)[0]["error_mm"],100)

    def test_nonmatching_image_aspect_is_rejected(self):
        cam=load()["table"]
        for size in ((640,360),(1280,720),(1920,1080)):
            check_frame_size(size,cam)
        with self.assertRaises(ValueError):
            check_frame_size((640,480),cam)

    def test_raw_real_view_detection_preserves_xy_at_both_client_resolutions(self):
        for size in ((640,360),(1280,720)):
            with self.subTest(size=size),tempfile.TemporaryDirectory() as tmp,patch.object(S,"ROOT",Path(tmp)):
                rgb,masks=fixture("screw",[.39,-.12],30,6,37,size=size)
                detections=describe_masks(rgb,masks)
                class FakeSam:
                    def detect(self,request):
                        return {"detections":detections}
                    def close(self):
                        pass
                p=Policy();p.sam=FakeSam()
                try:
                    found=p.detect({"images":{"table":rgb}})
                    self.assertEqual(len(found),1)
                    np.testing.assert_allclose(found[0]["world_xy"],[.39,-.12],atol=.004)
                    saved=json.loads((p.output/'cycle_000/targets.json').read_text())
                    self.assertEqual(len(saved["motion_eligible"]),1)
                    self.assertTrue((p.output/'cycle_000/camera_mapping.png').is_file())
                finally:
                    p.close()

    def test_physical_detection_can_correct_red_cube_in_screw_destination(self):
        rgb,masks=fixture("cube_red",S.DROP_XY['screws'],10,10,0)
        detections=describe_masks(rgb,masks)
        class FakeSam:
            def detect(self,request):
                return {"detections":detections}
            def close(self):
                pass
        with tempfile.TemporaryDirectory() as tmp,patch.object(S,"ROOT",Path(tmp)):
            p=Policy();p.sam=FakeSam()
            try:
                found=p.detect({"images":{"table":rgb}})
                self.assertEqual(len(found),1)
                self.assertEqual((found[0]['source_zone'],found[0]['drop']),('screws','red'))
            finally:
                p.close()

    def test_ungraspable_width_and_disabled_types_are_excluded_from_motion(self):
        small=target("cube","cube_red",10)
        wide=target("cube","cube_red",16)
        cap=target("cap","cap",31.75)
        missing=target();missing.pop('short_mm')
        eligible,disabled=Policy.motion_targets([small,wide,cap,missing])
        self.assertEqual(eligible,[small])
        self.assertEqual(len(disabled),3)

    def test_disabled_target_holds_without_attempting_a_grasp(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(S,"ROOT",Path(tmp)), \
             patch.object(S,"PHYSICAL_EXECUTE",True),patch.object(S,"PHYSICAL_POSES_CONFIRMED",True):
            p=Policy();p.mock=False;p.state='detect';p.detect=lambda obs:[target('cap','cap',31.75)]
            obs={'tcp_pose':[300,0,-100,-180,0,0],'joints':[0]*7}
            self.assertEqual(p.act(obs),obs['tcp_pose']+[0.0])
            self.assertTrue(p.halted)
            self.assertEqual(p.picks,0)

    def test_failed_source_is_rescanned_and_stops_after_two_attempts(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(S,"ROOT",Path(tmp)), \
             patch.object(S,"PHYSICAL_EXECUTE",True),patch.object(S,"PHYSICAL_POSES_CONFIRMED",True):
            p=Policy();p.mock=False;p.detect=lambda obs:[target()]
            events=[];p.log=lambda event,**data:events.append((event,data))
            obs={'tcp_pose':[300,0,-100,-180,0,0],'joints':[0]*7}
            for _ in range(600):
                rows=np.asarray(p.act(obs),float).reshape(-1,7)
                obs['tcp_pose']=rows[-1,:6].tolist();obs['joints'][6]=rows[-1,6]
                if p.halted:break
            self.assertTrue(p.halted)
            self.assertEqual(p.picks,2)
            self.assertEqual(sum(e=='source_still_detected' for e,_ in events),2)
            self.assertFalse(any(e=='release_commanded' for e,_ in events))

    def test_success_path_uses_low_table_release_and_preserves_unverified_logs(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(S,"ROOT",Path(tmp)), \
             patch.object(S,"PHYSICAL_EXECUTE",True),patch.object(S,"PHYSICAL_POSES_CONFIRMED",True):
            p=Policy();p.mock=False;read=iter([[target()],[],[]]);p.detect=lambda obs:next(read,[])
            events=[];p.log=lambda event,**data:events.append((event,data))
            obs={'tcp_pose':[300,0,-100,-180,0,0],'joints':[0]*7}
            previous=0;releases=[]
            for _ in range(400):
                rows=np.asarray(p.act(obs),float).reshape(-1,7)
                for row in rows:
                    if previous>=.5 and row[6]<.5:releases.append(row)
                    previous=row[6]
                obs['tcp_pose']=rows[-1,:6].tolist();obs['joints'][6]=rows[-1,6]
                if p.halted:break
            self.assertTrue(p.halted)
            self.assertEqual(len(releases),1)
            expected=base_to_tcp(*S.DROP_XY['screws'],S.SURFACE_Z+.010)
            np.testing.assert_allclose(releases[0][:3],expected[:3])
            records=[data for event,data in events if event=='release_commanded']
            self.assertEqual(len(records),1)
            self.assertFalse(records[0]['release_verified'])

    def test_watch_holds_until_deadline_without_repeating_sam(self):
        with tempfile.TemporaryDirectory() as tmp,patch.object(S,"ROOT",Path(tmp)), \
             patch.object(S,"PHYSICAL_EXECUTE",True),patch.object(S,"PHYSICAL_POSES_CONFIRMED",True), \
             patch.object(S,"PHYSICAL_WATCH",True),patch('policy.time.monotonic',return_value=100) as clock:
            p=Policy();p.mock=False;p.state='detect'
            reads=[];p.detect=lambda obs:reads.append(1) or []
            obs={'tcp_pose':[300,0,-100,-180,0,0],'joints':[0]*7}
            hold=obs['tcp_pose']+[0.0]
            self.assertEqual(p.act(obs),hold)
            self.assertEqual(p.state,'idle')
            clock.return_value=159
            self.assertEqual(p.act(obs),hold)
            self.assertEqual(len(reads),1)
            clock.return_value=160
            p.act(obs)
            self.assertEqual(p.state,'detect')
            self.assertEqual(len(reads),1)  # scan-pose motion settles before next detection
            self.assertFalse(p.halted)

    def test_submission_has_real_identity_and_matching_placement_count(self):
        with tempfile.TemporaryDirectory() as tmp:
            sub,trials=write_submission('Andrew Bieber, test@localhost',directory=tmp)
            self.assertEqual(sub['rollouts'],5)
            self.assertEqual(sub['rollouts'],len(trials))
            self.assertEqual(sub['objects'],'different')
            self.assertTrue((Path(tmp)/'placement.yaml').is_file())
            self.assertEqual(trials[0]['objects']['screw'],[390,0,0])
            with self.assertRaises(ValueError):
                write_submission('YOUR_EMAIL@example.com',directory=tmp)


if __name__=='__main__':
    unittest.main()
