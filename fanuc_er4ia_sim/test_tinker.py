"""Tinker app checks that need no display, Isaac or GPU."""
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest

import tinker as A


def row(seed, status="pass", placed=13, recoveries=0):
    return dict(seed=seed, status=status, returncode=0, wall_s=790.0, placed=placed, expected=13,
                complete=status == "pass", failed_objects=[] if status == "pass" else ["/World/SortVial0"],
                recoveries=[{"reason": "dropped"}] * recoveries, camera_sim_agreement={"agree": 13, "checks": 13},
                sim_final_audit_ok=True, sort_seconds=600, performance=None, layout_attempt=1, physics=None,
                error=None, objects=[])


class CommandTests(unittest.TestCase):
    def test_sort_command_matches_benchmark_trial(self):
        cmd = A.sort_command("isaac", Path("r"), seed=1007, headless=True)
        self.assertEqual(cmd[:3], ["isaac", "-u", str(A.HERE / "demo_scan_sort_spaced.py")])
        for part in (["--headless", "--exit-on-finish"], ["--layout-seed", "1007"], ["--physics-seed", "1007"]):
            i = cmd.index(part[0])
            self.assertEqual(cmd[i:i + len(part)], part)

    def test_sort_command_defaults_and_options(self):
        cmd = A.sort_command("isaac", Path("r"))
        self.assertIn("--viewport-fps", cmd)
        for flag in ("--layout-seed", "--physics-seed", "--only", "--careful", "--headless"):
            self.assertNotIn(flag, cmd)
        cmd = A.sort_command("isaac", Path("r"), seed=5, physics=False, only="vial", careful=True, force_drop=True)
        self.assertNotIn("--physics-seed", cmd)
        self.assertEqual(cmd[cmd.index("--only") + 1], "vial")
        self.assertIn("--careful", cmd)
        self.assertIn("--force-drop", cmd)

    def test_bench_command(self):
        cmd = A.bench_command(Path("out"), trials=3, start_seed=2000, gui=True, retry_crashed=True, isaac_python="i")
        self.assertEqual(cmd[0], sys.executable)
        self.assertEqual(cmd[cmd.index("--trials") + 1], "3")
        self.assertEqual(cmd[cmd.index("--start-seed") + 1], "2000")
        self.assertIn("--gui", cmd)
        self.assertIn("--retry-crashed", cmd)


class ParseTests(unittest.TestCase):
    def test_progress_lines(self):
        self.assertEqual(A.parse_line("[scan] OBJECT 3/13: cube_red 1"), ("object", ("3", "13", "cube_red")))
        self.assertEqual(A.parse_line("[verify] Camera: /World/SortCube0 in red bin -> PLACED")[1][1], "PLACED")
        self.assertEqual(A.parse_line("[verify] Camera: /World/SortCap0 off target -> NOT PLACED")[1][1], "NOT PLACED")
        self.assertEqual(A.parse_line("[sort] PASS: 13/13 objects placed")[0], "result")
        self.assertEqual(A.parse_line("[bench] trial 1/50 seed 1000: PASS 13/13 placed")[1], ("1", "50", "1000", "PASS"))
        self.assertIsNone(A.parse_line("[layout] Random layout: seed 1000"))

    def test_colour_tags(self):
        self.assertEqual(A.line_tag("[bench] trial 1/50 seed 1000: PASS 13/13 placed"), "good")
        self.assertEqual(A.line_tag("[bench] trial 2/50 seed 1001: CRASH None/None placed"), "bad")
        self.assertEqual(A.line_tag("[sort] FAIL: 12/13"), "bad")
        self.assertEqual(A.line_tag("[recover] /World/SortVial0: dropped"), "warn")
        self.assertIsNone(A.line_tag("[layout] Separated layout"))


class FolderTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.out = Path(self.tmp.name) / "benchmark"
        self.out.mkdir()

    def tearDown(self):
        self.tmp.cleanup()

    def test_results_skip_partial_line_and_stats(self):
        (self.out / "results.jsonl").write_text(
            json.dumps(row(1000)) + "\n" + json.dumps(row(1001, "fail", 12, 2)) + "\n" + '{"seed": 10')
        rows = A.load_results(self.out)
        self.assertEqual([r["seed"] for r in rows], [1000, 1001])
        text = A.stats_line(rows)
        self.assertIn("1/2 full sorts passed", text)
        self.assertIn("objects 25/26", text)
        self.assertIn("2 recoveries", text)
        self.assertEqual(A.stats_line([]), "No finished trials yet.")

    def test_activity_from_newest_trial_log(self):
        self.assertIsNone(A.benchmark_activity(self.out))
        log = self.out / "trial_01000" / "console.log"
        log.parent.mkdir()
        log.write_text("x")
        old = time.time() - 600
        os.utime(log, (old, old))
        self.assertGreater(A.benchmark_activity(self.out), 590)
        self.assertLess(A.benchmark_activity(self.out, now=old + 5), 10)

    def test_log_path_inside_or_beside(self):
        self.assertEqual(A.bench_log_path(self.out), self.out / "benchmark.log")
        beside = self.out.with_suffix(".log")
        beside.write_text("[bench] code: x")
        self.assertEqual(A.bench_log_path(self.out), beside)
        (self.out / "benchmark.log").write_text("")
        self.assertEqual(A.bench_log_path(self.out), self.out / "benchmark.log")

    def test_tail_reads_only_new_lines(self):
        path = self.out / "x.log"
        tail = A.Tail(path)
        self.assertEqual(tail.read(), [])
        path.write_text("a\nb\n")
        self.assertEqual(tail.read(), ["a", "b"])
        with open(path, "a") as f:
            f.write("c\n")
        self.assertEqual(tail.read(), ["c"])

    def test_job_writes_command_and_output_to_log(self):
        log = self.out / "job.log"
        job = A.Job([sys.executable, "-c", "print('hello from job')"], log)
        job.proc.wait(timeout=30)
        text = log.read_text()
        self.assertTrue(text.startswith("$ "))
        self.assertIn("hello from job", text)
        self.assertFalse(job.running())


class PhysicalLabTests(unittest.TestCase):
    def test_commands_match_readme(self):
        self.assertEqual(A.physical_command("preflight", uv="uv"), ["uv", "run", "preflight.py", "--sam", "--require-cuda"])
        self.assertEqual(A.physical_command("mock", trials=5, uv="uv"), ["uv", "run", "run.py", "--mock", "--trials", "5"])
        self.assertEqual(A.physical_command("submit", uv="uv"), ["uv", "run", "run.py", "--submit"])
        self.assertEqual(A.physical_command("slot", uv="uv"), ["uv", "run", "run.py"])
        self.assertEqual(A.physical_command("prepare", contact=" Andrew Bieber, a@b.c ", uv="uv")[-2:],
                         ["--contact", "Andrew Bieber, a@b.c"])
        with self.assertRaises(ValueError):
            A.physical_command("prepare", contact="  ", uv="uv")
        for step, (args, _) in A.PHYSICAL_STEPS.items():
            self.assertTrue((A.PHYSICAL / args[0]).is_file(), f"{step}: {args[0]} missing in physical-client")

    def test_env_adds_uv_and_sam(self):
        env = A.physical_env({"PATH": "/usr/bin"})
        self.assertTrue(env["PATH"].endswith("/usr/bin"))
        self.assertIn("physical-uv", env["PATH"])
        self.assertIn("sam2-check", env["SORT_SAM_PYTHON"])
        self.assertEqual(A.physical_env({"PATH": "", "SORT_SAM_PYTHON": "x"})["SORT_SAM_PYTHON"], "x")

    def test_flags_are_read_not_written(self):
        with tempfile.TemporaryDirectory() as d:
            cfg = Path(d) / "sort_config.py"
            cfg.write_text("PHYSICAL_EXECUTE = True  # note\nPHYSICAL_POSES_CONFIRMED = False\n")
            before = cfg.read_text()
            self.assertEqual(A.physical_flags(cfg), {"PHYSICAL_EXECUTE": True, "PHYSICAL_POSES_CONFIRMED": False})
            self.assertEqual(cfg.read_text(), before)
            self.assertEqual(A.physical_flags(Path(d) / "missing.py"), {})
        self.assertEqual(set(A.physical_flags()), {"PHYSICAL_EXECUTE", "PHYSICAL_POSES_CONFIRMED"})

    def test_request_id_hides_secret(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / ".request.json"
            path.write_text(json.dumps({"request_id": "q3ac71f", "secret": "s3cret", "url": "u"}))
            self.assertEqual(A.request_id(path), "q3ac71f")
            self.assertIsNone(A.request_id(Path(d) / "none.json"))

    def test_cell_lines(self):
        self.assertEqual(A.parse_cell_line("request q3ac71f: pending"), ("request", ("q3ac71f", "pending")))
        self.assertEqual(A.parse_cell_line("INFO request q3ac71f approved: run r12")[0], "approved")
        self.assertEqual(A.parse_cell_line("INFO request q3 is waiting for the operator's approval ...")[0], "waiting")
        self.assertEqual(A.parse_cell_line("INFO trial 7 (layout 2) running"), ("trial", ("7", "2")))
        self.assertEqual(A.parse_cell_line("INFO trial 7 ended"), ("trial_end", ("7",)))
        self.assertEqual(A.parse_cell_line("INFO all trials finished")[0], "done")
        self.assertEqual(A.parse_cell_line("✗ Not ready: 2 actions were rejected")[0], "verdict")


if __name__ == "__main__":
    unittest.main()
