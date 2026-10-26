"""Watch-mode batch recovery checks; no SAM, network or physical robot access."""
from contextlib import ExitStack
from copy import deepcopy
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

import sort_config as S
from mock_cell import validate_tcp
from policy import Policy


def target(label, xy):
    return {"kind": "cube", "label": label, "world_xy": list(xy),
            "yaw_base_deg": 0.0, "drop": S.BIN_FOR[label], "short_mm": 10.0}


class BatchWatchTests(unittest.TestCase):
    def check_new_target_after_idle(self, empty_scans):
        with ExitStack() as ctx:
            tmp = ctx.enter_context(tempfile.TemporaryDirectory())
            settings = {
                "ROOT": Path(tmp), "PHYSICAL_EXECUTE": True,
                "PHYSICAL_POSES_CONFIRMED": True, "PHYSICAL_SCAN_MODE": "batch",
                "PHYSICAL_WATCH": True, "PHYSICAL_BATCH_MAX_IDLE_PASSES": 2,
                "PHYSICAL_ALLOWED_KINDS": ("screw", "cube"),
                "PHYSICAL_VERIFIED_FIT_KINDS": ("screw", "cube"),
                "MAX_ATTEMPTS": 2, "MAX_PICKS": 20,
            }
            for name, value in settings.items():
                ctx.enter_context(patch.object(S, name, value))
            clock = ctx.enter_context(patch("policy.time.monotonic", return_value=100.0))
            # Only old A changes. New B must get its own image check and attempt.
            ctx.enter_context(patch("policy.target_unchanged",
                side_effect=lambda reference, current, t: t["label"] != "cube_red"))
            a = target("cube_red", (.30, -.10))
            b = target("cube_blue", (.40, .10))
            passes = [[a]] + [[] for _ in range(empty_scans)] + [[b], []]
            schedule = iter(passes)
            scans, events = [], []
            p = Policy()
            p.mock = False
            ctx.callback(p.close)
            p.log = lambda event, **data: events.append((event, deepcopy(data)))

            def detect(obs):
                objects = deepcopy(next(schedule, []))
                scans.append((p.picks, objects))
                p.observed_objects = deepcopy(objects)
                return objects

            p.detect = detect
            obs = {"tcp_pose": [300, 0, -100, -180, 0, 0], "joints": [0] * 7,
                   "images": {"table": np.zeros((720, 1280, 3), np.uint8)}}
            for _ in range(2000):
                if p.state == "idle":
                    clock.return_value = p.idle_until + .01
                rows = np.asarray(p.act(obs), float).reshape(-1, 7)
                validate_tcp(rows, .1)
                obs["tcp_pose"] = rows[-1, :6].tolist()
                obs["joints"][6] = float(rows[-1, 6])
                if p.halted or (len(scans) == len(passes) and p.state == "idle"):
                    break
            self.assertFalse(p.halted, events)
            self.assertEqual(len(scans), len(passes))
            self.assertEqual([t["label"] for e, t in events if e == "attempt"],
                             ["cube_blue"])
            self.assertEqual(p.picks, 1)
            self.assertEqual(scans[-1][0], 1)  # final scan followed B's attempt
            self.assertEqual(p.batch_idle_passes, 0)
            self.assertFalse(p.batch_active)
            self.assertEqual(p.state, "idle")

    def test_new_target_after_deferred_batch_and_empty_scan_is_attempted(self):
        self.check_new_target_after_idle(empty_scans=1)

    def test_repeated_empty_watch_scans_do_not_count_as_stale_batches(self):
        self.check_new_target_after_idle(empty_scans=3)


if __name__ == "__main__":
    unittest.main()
