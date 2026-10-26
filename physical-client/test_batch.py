"""Physical batch regressions with synthetic observations; no SAM or robot access."""
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

import sort_config as S
from mock_cell import validate_tcp
from policy import Policy, base_to_tcp
from sort_vision import describe_masks, target_unchanged
from test_sorting import fixture


def target(label="cube_red", xy=(.30, -.10), width=16.0):
    kind = "cube" if label.startswith("cube_") else label
    return {"kind": kind, "label": label, "world_xy": list(xy),
            "yaw_base_deg": 20.0, "drop": S.BIN_FOR[label], "short_mm": width}


class BatchTests(unittest.TestCase):
    def setUp(self):
        self.context = ExitStack()
        self.addCleanup(self.context.close)
        root = self.context.enter_context(tempfile.TemporaryDirectory())
        for name, value in {
            "ROOT": Path(root), "PHYSICAL_EXECUTE": True,
            "PHYSICAL_POSES_CONFIRMED": True, "PHYSICAL_SCAN_MODE": "batch",
            "PHYSICAL_BATCH_MAX_IDLE_PASSES": 2, "PHYSICAL_WATCH": False,
            "PHYSICAL_ALLOWED_KINDS": ("screw", "cube"),
            "PHYSICAL_VERIFIED_FIT_KINDS": ("screw", "cube"),
            "MAX_ATTEMPTS": 2, "MAX_PICKS": 20,
        }.items():
            self.context.enter_context(patch.object(S, name, value, create=True))
        self.unchanged = self.context.enter_context(
            patch("policy.target_unchanged", return_value=True))
        self.p = Policy()
        self.p.mock = False
        self.addCleanup(self.p.close)
        self.events = []
        self.p.log = lambda event, **data: self.events.append((event, deepcopy(data)))
        self.obs = {"tcp_pose": [300, 0, -100, -180, 0, 0],
                    "joints": [0] * 7,
                    "images": {"table": np.zeros((720, 1280, 3), np.uint8)}}
        self.scans = []
        self.rows = []

    def feed_scans(self, passes):
        schedule = iter(passes)

        def detect(obs):
            objects = deepcopy(next(schedule, []))
            self.scans.append({"picks": self.p.picks,
                               "pose": list(obs["tcp_pose"]),
                               "objects": objects})
            self.p.observed_objects = deepcopy(objects)
            return objects

        self.p.detect = detect

    def step(self):
        rows = np.asarray(self.p.act(self.obs), float).reshape(-1, 7)
        validate_tcp(rows, .1)
        self.rows.extend(rows.tolist())
        self.obs["tcp_pose"] = rows[-1, :6].tolist()
        self.obs["joints"][6] = float(rows[-1, 6])

    def finish(self, limit=4000):
        for _ in range(limit):
            self.step()
            if self.p.halted:
                break
        self.assertTrue(self.p.halted, "Batch failed to terminate within action budget")

    def attempts(self):
        return [data for event, data in self.events if event == "attempt"]

    def test_three_objects_use_initial_and_final_scan_only(self):
        objects = [target("screw", (.30, -.10), 6),
                   target("cube_blue", (.39, .03)),
                   target("cube_yellow", (.40, .18))]
        self.feed_scans([objects, []])
        self.finish()
        self.assertEqual(self.p.picks, 3)
        self.assertEqual([scan["picks"] for scan in self.scans], [0, 3])
        self.assertEqual([t["label"] for t in self.attempts()],
                         ["screw", "cube_blue", "cube_yellow"])
        self.assertEqual(sum(event == "release_commanded" for event, _ in self.events), 3)
        for scan in self.scans:
            np.testing.assert_allclose(scan["pose"],
                base_to_tcp(*S.SCAN_XY, S.SURFACE_Z + S.SCAN_HEIGHT)[:6])
        clearance = base_to_tcp(0, 0, S.SURFACE_Z + S.TRAVEL_HEIGHT)[2]
        for a, b in zip(self.rows, self.rows[1:]):
            if np.linalg.norm(np.asarray(b[:2]) - a[:2]) > 1e-8:
                self.assertGreaterEqual(min(a[2], b[2]), clearance - 1e-8)

    def test_missed_first_object_waits_until_other_targets_are_attempted(self):
        a = target("screw", (.30, -.10), 6)
        b = target("cube_blue", (.39, .03))
        c = target("cube_yellow", (.40, .18))
        self.feed_scans([[a, b, c], [a], []])
        self.finish()
        self.assertEqual([t["label"] for t in self.attempts()],
                         ["screw", "cube_blue", "cube_yellow", "screw"])
        self.assertEqual([scan["picks"] for scan in self.scans], [0, 3, 4])
        for event, data in self.events:
            if event == "attempt":
                self.assertIs(data["pickup_verified"], False)
            elif event == "release_commanded":
                self.assertIs(data["release_verified"], False)

    def test_persistent_object_stops_after_two_attempts_and_final_scan(self):
        a = target("screw", (.30, -.10), 6)
        self.feed_scans([[a], [a], [a], [a]])
        self.finish()
        self.assertEqual(self.p.picks, 2)
        self.assertEqual([scan["picks"] for scan in self.scans], [0, 1, 2])
        self.assertEqual(len(self.attempts()), 2)

    def test_total_pick_limit_still_performs_final_batch_scan(self):
        with patch.object(S, "MAX_PICKS", 1):
            a = target("screw", (.30, -.10), 6)
            b = target("cube_blue", (.39, .03))
            self.feed_scans([[a, b], [b]])
            self.finish()
        self.assertEqual(self.p.picks, 1)
        self.assertEqual([scan["picks"] for scan in self.scans], [0, 1])

    def test_changed_cached_target_is_deferred_without_consuming_attempt(self):
        a = target("screw", (.30, -.10), 6)
        b = target("cube_blue", (.39, .03))
        self.unchanged.side_effect = lambda reference, current, t: t["label"] != "screw"
        self.feed_scans([[a, b], []])
        self.finish()
        self.assertEqual([t["label"] for t in self.attempts()], ["cube_blue"])
        self.assertEqual(self.p.attempts.count(a), 0)
        self.assertTrue(any(event == "cached_target_deferred" for event, _ in self.events))
        self.assertEqual([scan["picks"] for scan in self.scans], [0, 1])

    def test_repeatedly_changed_targets_have_bounded_scan_passes(self):
        a = target("screw", (.30, -.10), 6)
        self.unchanged.return_value = False
        self.feed_scans([[a]] * 10)
        self.finish(limit=1000)
        self.assertEqual(self.p.picks, 0)
        self.assertEqual(self.attempts(), [])
        self.assertLessEqual(len(self.scans), S.PHYSICAL_BATCH_MAX_IDLE_PASSES + 1)
        self.assertGreaterEqual(len(self.scans), 2)

    def test_occupied_destination_centre_is_not_selected_for_release(self):
        a = target()
        centre = np.asarray(S.DROP_XY["red"])
        self.p.observed_objects = [{"world_xy": centre.tolist()}]
        xy = np.asarray(self.p.next_drop_xy(a))
        self.assertGreaterEqual(np.linalg.norm(xy - centre), .035)
        self.assertLessEqual(np.linalg.norm(xy - centre), S.DROP_RADIUS)

    def test_repeated_releases_use_distinct_landing_spots(self):
        objects = [target(xy=xy) for xy in ((.30, -.10), (.39, .03), (.40, .18))]
        self.feed_scans([objects, []])
        self.finish()
        releases = []
        drop_z = base_to_tcp(0, 0, S.SURFACE_Z + S.DROP_HEIGHT)[2]
        for a, b in zip(self.rows, self.rows[1:]):
            if a[6] >= .5 and b[6] < .5 and abs(b[2] - drop_z) < 1e-6:
                releases.append(np.asarray(b[:2]) / 1000)
        self.assertEqual(len(releases), 3)
        for i, a in enumerate(releases):
            self.assertLessEqual(np.linalg.norm(a - S.DROP_XY["red"]), S.DROP_RADIUS)
            for b in releases[i + 1:]:
                self.assertGreaterEqual(np.linalg.norm(a - b), .035 - 1e-9)

    def test_reset_discards_previous_pass_state(self):
        a = target()
        self.feed_scans([[a], []])
        self.finish()
        self.p.reset()
        self.assertEqual(self.p.picks, 0)
        self.assertEqual(self.p.batch_queue.targets, [])
        self.assertIsNone(self.p.batch_reference)
        self.assertEqual(self.p.observed_objects, [])
        self.assertEqual(self.p.drop_reservations, {})
        self.assertEqual(self.p.batch_number, 0)
        self.assertFalse(self.p.halted)

    def test_known_fit_width_override_preserves_other_eligibility_checks(self):
        good = [target(label, width=30) for label in
                ("cube_red", "cube_blue", "cube_yellow", "screw")]
        bad = [target(width=width) for width in (0, -1, float("nan"), float("inf"))]
        bad.append(target("cap", width=31.75))
        bad.append(target(xy=(.12, .25)))
        bad.append(target(xy=(float("nan"), 0)))
        eligible, disabled = Policy.motion_targets(good + bad)
        self.assertEqual([id(t) for t in eligible], [id(t) for t in good])
        self.assertEqual(len(disabled), len(bad))
        with patch.object(S, "PHYSICAL_VERIFIED_FIT_KINDS", ()):
            eligible, disabled = Policy.motion_targets(good)
        self.assertEqual(eligible, [])
        self.assertEqual(len(disabled), len(good))

    def test_real_pixel_checks_keep_unchanged_objects_after_previous_pick(self):
        specifications = [("cube_red", (.30, -.10)),
                          ("cube_blue", (.39, .03)),
                          ("cube_yellow", (.40, .18))]
        rgb = np.full((720, 1280, 3), 235, np.uint8)
        masks = {}
        for label, xy in specifications:
            image, records = fixture(label, xy, 10, 10, 20)
            record = records[0]
            masks[label] = record
            rgb[record["segmentation"]] = image[record["segmentation"]]
        self.obs["images"]["table"] = rgb
        active = {label for label, _ in specifications}
        scans = []
        outer = self

        class FakeSam:
            def detect(self, request):
                scans.append({"picks": outer.p.picks, "active": set(active)})
                records = [masks[label] for label, _ in specifications if label in active]
                return {"detections": describe_masks(outer.obs["images"]["table"], records)}

            def close(self):
                pass

        self.p.sam = FakeSam()
        self.unchanged.side_effect = target_unchanged
        original_step = self.step
        releases_seen = 0

        def step_and_remove_picked_pixels():
            nonlocal releases_seen
            original_step()
            release_count = sum(event == "release_commanded" for event, _ in self.events)
            if release_count > releases_seen:
                label = self.p.selected["label"]
                active.remove(label)
                self.obs["images"]["table"][masks[label]["segmentation"]] = 235
                releases_seen = release_count

        self.step = step_and_remove_picked_pixels
        self.finish()
        self.assertEqual([scan["picks"] for scan in scans], [0, 3])
        self.assertEqual(self.p.picks, 3)
        self.assertEqual(active, set())
        self.assertEqual({attempt["label"] for attempt in self.attempts()},
                         {label for label, _ in specifications})
        self.assertFalse(any(event == "cached_target_deferred" for event, _ in self.events))
        self.assertEqual(self.unchanged.call_count, 3)

    def test_real_detection_retains_correct_destination_object_for_occupancy(self):
        rgb, masks = fixture("cube_red", S.DROP_XY["red"], 10, 10, 0)
        detections = describe_masks(rgb, masks)
        requests = []

        class FakeSam:
            def detect(self, request):
                requests.append(request)
                return {"detections": detections}

            def close(self):
                pass

        self.p.sam = FakeSam()
        self.obs["images"]["table"] = rgb
        found = self.p.detect(self.obs)
        self.assertEqual(found, [])
        self.assertEqual(len(requests), 1)
        self.assertEqual(len(self.p.observed_objects), 1)
        occupied = self.p.observed_objects[0]
        self.assertEqual(occupied["label"], "cube_red")
        self.assertEqual(occupied["source_zone"], "red")
        np.testing.assert_allclose(occupied["world_xy"], S.DROP_XY["red"], atol=.004)
        landing = np.asarray(self.p.next_drop_xy(target()))
        self.assertGreaterEqual(np.linalg.norm(landing - occupied["world_xy"]), .035)
        self.assertLessEqual(np.linalg.norm(landing - S.DROP_XY["red"]), S.DROP_RADIUS)


if __name__ == "__main__":
    unittest.main()
