"""Physical dual-zone state-machine checks; synthetic RGB, no robot/SAM access.

The controller rows and real clearance implementation are exercised. The RGB
guard is stubbed here to isolate planning; its image tests live separately.
"""
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

import gripper_profile as G
import sort_config as S
from dual_clearance import ClearanceError
from mock_cell import validate_tcp
from policy import Policy


def detection(label, xy, yaw=0.):
    kind = "cube" if label.startswith("cube_") else label
    length, width, height = {
        "screw": (30., 6., .006), "cube": (16., 16., .016),
        "cap": (31.75, 31.75, .0157), "vial": (203., 27.18, .02718),
    }[kind]
    return dict(kind=kind, label=label, world_xy=list(xy), yaw_base_deg=yaw,
                long_mm=length, short_mm=width, height_m=height, drop=S.BIN_FOR[label])


class DualPolicyTests(unittest.TestCase):
    def setUp(self):
        self.context = ExitStack()
        self.addCleanup(self.context.close)
        directory = self.context.enter_context(tempfile.TemporaryDirectory())
        for name, value in {
            "ROOT": Path(directory), "PHYSICAL_GRIPPER_PROFILE": "dual_zone_v2",
            "PHYSICAL_EXECUTE": True, "PHYSICAL_POSES_CONFIRMED": True,
            "PHYSICAL_ALLOWED_KINDS": ("screw", "cube", "vial", "cap"),
            "PHYSICAL_VERIFIED_FIT_KINDS": ("screw", "cube"),
            "PHYSICAL_SCAN_MODE": "batch", "PHYSICAL_WATCH": False,
            "PHYSICAL_BATCH_MAX_IDLE_PASSES": 2, "MAX_ATTEMPTS": 2,
            "MAX_PICKS": 20, "PAUSE_AFTER_LARGE_OBJECT": False,
            "DUAL_ZONE_MOUNT_CONFIRMED": True, "DUAL_ZONE_OBJECTS_CONFIRMED": True,
            "DUAL_ZONE_LAYOUT_CONFIRMED": True, "DUAL_ZONE_CAP_NARROW_END_UP": True,
            "DUAL_ZONE_PRIMARY_OFFSET_TCP_M": (0., 0., 0.),
            "DUAL_ZONE_CAP_OFFSET_TCP_M": (.040, 0., 0.),
            "DUAL_ZONE_STATIC_OBSTACLES": [], "DUAL_ZONE_CLEARANCE_MM": 2.,
        }.items():
            self.context.enter_context(patch.object(S, name, value, create=True))
        self.context.enter_context(patch("policy.target_unchanged", return_value=True))
        self.context.enter_context(patch.object(Policy, "dual_regions_unchanged", return_value=True))
        self.policy = Policy()
        self.policy.mock = False
        self.addCleanup(self.policy.close)
        self.events, self.scans, self.rows = [], [], []
        self.policy.log = lambda event, **data: self.events.append((event, deepcopy(data)))
        self.obs = {"tcp_pose": [300., 0., -100., -180., 0., 0.],
                    "joints": [0.] * 7,
                    "images": {"table": np.zeros((720, 1280, 3), np.uint8)}}

    def feed_scans(self, passes):
        scheduled = iter(passes)

        def detect(obs):
            objects = deepcopy(next(scheduled, []))
            self.scans.append((self.policy.picks, deepcopy(objects)))
            self.policy.observed_objects = deepcopy(objects)
            return objects

        self.policy.detect = detect

    def step(self):
        rows = np.asarray(self.policy.act(self.obs), float).reshape(-1, 7)
        validate_tcp(rows, .1)
        self.rows.extend(rows.tolist())
        self.obs["tcp_pose"] = rows[-1, :6].tolist()
        self.obs["joints"][6] = float(rows[-1, 6])

    def finish(self, limit=4000):
        for _ in range(limit):
            self.step()
            if self.policy.halted:
                break
        self.assertTrue(self.policy.halted, "Physical policy did not terminate")

    def attempts(self):
        return [data for event, data in self.events if event == "attempt"]

    def test_three_kinds_sort_in_one_batch_with_one_final_scan(self):
        objects = [detection("screw", (.30, -.10)),
                   detection("cube_red", (.40, .10)),
                   detection("cap", (.40, -.20))]
        self.feed_scans([objects, []])
        self.finish()
        self.assertEqual([a["label"] for a in self.attempts()], ["screw", "cube_red", "cap"])
        self.assertEqual([picks for picks, _ in self.scans], [0, 3])
        releases = [data for event, data in self.events if event == "release_commanded"]
        self.assertEqual(len(releases), 3)
        self.assertTrue(all(a["pickup_verified"] is False for a in self.attempts()))
        self.assertTrue(all(a["release_verified"] is False for a in releases))
        self.assertTrue(set(np.asarray(self.rows)[:, 6]).issubset({0., 1.}))

    def test_vial_contact_offset_preserves_source_and_release_object_centres(self):
        vial = detection("vial", (.38, 0.))
        original = deepcopy(vial)
        self.feed_scans([[vial], []])
        self.finish()
        self.assertEqual(self.policy.picks, 1, self.events)
        self.assertEqual([picks for picks, _ in self.scans], [0, 1])
        self.assertEqual(vial, original)
        selected = self.policy.selected
        self.assertEqual(selected["world_xy"], original["world_xy"])
        spec = selected["grasp_spec"]
        closes, opens = [], []
        for previous, row in zip(self.rows, self.rows[1:]):
            if previous[6] == 0 and row[6] == 1:
                closes.append(row)
            if previous[6] == 1 and row[6] == 0:
                opens.append(row)
        self.assertEqual((len(closes), len(opens)), (1, 1))
        for pose, expected in ((closes[0], vial["world_xy"]), (opens[0], selected["drop_xy"])):
            np.testing.assert_allclose(G.tcp_to_object_xy(pose, spec), expected, atol=1e-9)
            contact = G.tcp_to_contact_base(pose, spec)
            self.assertAlmostEqual(np.linalg.norm(contact[:2]-expected), .0815)
            self.assertAlmostEqual(np.linalg.norm(np.array(pose[:2])/1000-expected), .1215)
            self.assertAlmostEqual(contact[2], S.SURFACE_Z+.003)
        self.assertEqual(self.policy.placed_envelopes[0]["world_xy"], selected["drop_xy"])

    def test_unconfirmed_mount_holds_without_scanning_or_queuing(self):
        self.feed_scans([[detection("screw", (.30, -.10))]])
        before = list(self.obs["tcp_pose"])
        with patch.object(S, "DUAL_ZONE_MOUNT_CONFIRMED", False):
            self.step()
        self.assertTrue(self.policy.halted)
        self.assertEqual(self.obs["tcp_pose"], before)
        self.assertEqual(self.scans, [])
        self.assertEqual(self.policy.picks, 0)
        self.assertFalse(self.policy.queue)

    def test_original_dual_rejects_shaft_revised_dual_accepts_it(self):
        screw = detection("screw", (.30, -.10))
        with patch.object(S, "PHYSICAL_GRIPPER_PROFILE", "dual_zone"):
            eligible, disabled = self.policy.motion_targets([screw])
            self.assertEqual(eligible, [])
            self.assertIn("shaft grasp unsupported", disabled[0]["reason"])
        eligible, disabled = self.policy.motion_targets([screw])
        self.assertEqual(eligible, [screw])
        self.assertEqual(disabled, [])

    def test_vial_eligibility_considers_valid_opposite_approach(self):
        vial = detection("vial", (.22, 0.))
        spec = G.grasp_spec(vial)
        # Canonical yaw places the primary TCP below the controller's minimum
        # X. The opposite grasp puts it safely at341.5mm instead of98.5mm.
        with self.assertRaises(ValueError):
            Policy.validate(Policy.object_pose(vial, vial["world_xy"],
                S.SURFACE_Z+spec["pick_height_m"], 0., 0., spec))
        Policy.validate(Policy.object_pose(vial, vial["world_xy"],
            S.SURFACE_Z+spec["pick_height_m"], -180., 0., spec))
        eligible, disabled = self.policy.motion_targets([vial])
        self.assertEqual(eligible, [vial])
        self.assertEqual(disabled, [])

    def test_invalid_delivery_never_queues_pickup_or_consumes_attempt(self):
        cube = detection("cube_red", (.40, .10))
        self.policy.observed_objects = [deepcopy(cube)]
        with patch.dict(S.DROP_XY, {"red": (2., 0.)}):
            with self.assertRaises(ClearanceError):
                self.policy.begin_pick(Policy.hold(self.obs), cube)
        self.assertEqual(self.policy.picks, 0)
        self.assertEqual(self.policy.attempts.count(cube), 0)
        self.assertFalse(self.policy.queue)
        self.assertIsNone(self.policy.selected)

    def test_source_recovery_screens_possible_payload_until_open_command(self):
        vial = detection("vial", (.38, 0.))
        spec = G.grasp_spec(vial)
        self.policy.selected = {**vial, "grasp_spec": spec}
        self.policy.observed_objects = [deepcopy(vial)]
        above = Policy.object_pose(vial, vial["world_xy"],
            S.SURFACE_Z+S.TRAVEL_HEIGHT, 0., 1., spec)
        low = Policy.object_pose(vial, vial["world_xy"],
            S.SURFACE_Z+spec["pick_height_m"], 0., 1., spec)
        opened = [*low[:6], 0.]
        # This obstacle lies under the far vial end, well clear of either
        # physical rail. A recovery descent still may be carrying the vial.
        obstacle = dict(name="far-end stop", bounds_xy=[[.455, -.005], [.465, .005]], height_m=.015)
        with patch.object(S, "DUAL_ZONE_STATIC_OBSTACLES", [obstacle]):
            self.policy.dual_check(above, [low, opened], vial, carrying=False)
            with self.assertRaisesRegex(ClearanceError, "Carried object near far-end stop"):
                self.policy.enqueue(above, [low, opened], "rescan")
        self.assertFalse(self.policy.queue)

    def test_obstructed_target_defers_without_attempt_then_clear_target_runs(self):
        blocked = detection("screw", (.30, -.10))
        clear = detection("cube_blue", (.41, .14))
        # Both possible inactive-rail sides are obstructed; the central screw
        # itself is excluded as the intended source, while these walls are not.
        walls = [dict(name="left stop", bounds_xy=[[.25, -.14], [.265, -.06]], height_m=.025),
                 dict(name="right stop", bounds_xy=[[.335, -.14], [.35, -.06]], height_m=.025)]
        self.feed_scans([[blocked, clear], []])
        with patch.object(S, "DUAL_ZONE_STATIC_OBSTACLES", walls):
            self.finish()
        self.assertEqual([a["label"] for a in self.attempts()], ["cube_blue"])
        self.assertEqual(self.policy.attempts.count(blocked), 0)
        deferred = [data for event, data in self.events if event == "cached_target_deferred"]
        self.assertTrue(any(d["label"] == "screw" and "Finger rail" in d["reason"] for d in deferred))
        self.assertEqual([picks for picks, _ in self.scans], [0, 1])


if __name__ == "__main__":
    unittest.main()
