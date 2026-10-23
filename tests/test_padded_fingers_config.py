"""Host-only regression tests for padded-gripper configuration."""

import json
import os
from pathlib import Path
import subprocess
import sys
import unittest


SIM_DIR = Path(__file__).resolve().parents[1] / "fanuc_er4ia_sim"


def probe(**environment):
    env = os.environ.copy()
    for name in ("FINGER_SET", "PAD_FACE_MM", "JAW_STROKE_MM"):
        env.pop(name, None)
    env.update(environment)
    code = """
import json
import scene_config as C
import padded_fingers as P
print(json.dumps({
    "finger_set": C.FINGER_SET,
    "stroke": C.JAW_STROKE,
    "pad_face": C.PAD_FACE,
    "enabled": P.enabled(),
    "closed_gap": P.closed_gap(),
    "open_gap": P.open_gap(),
}))
"""
    return subprocess.run(
        [sys.executable, "-c", code],
        cwd=SIM_DIR,
        env=env,
        text=True,
        capture_output=True,
    )


class PaddedFingerConfigurationTests(unittest.TestCase):
    def test_default_is_installed_smc_scallop_configuration(self):
        result = probe()
        self.assertEqual(result.returncode, 0, result.stderr)
        values = json.loads(result.stdout)
        self.assertEqual(values["finger_set"], "scallop")
        self.assertFalse(values["enabled"])
        self.assertAlmostEqual(values["stroke"], 0.006)

    def test_supported_presets_have_consistent_geometry(self):
        expected = {
            "padded": (0.004, 0.006, 0.008, 0.020),
            "cap": (0.007, 0.006, 0.014, 0.026),
            "universal": (0.0045, 0.0085, 0.009, 0.026),
        }
        for preset, (face, stroke, closed, opened) in expected.items():
            with self.subTest(preset=preset):
                result = probe(FINGER_SET=preset)
                self.assertEqual(result.returncode, 0, result.stderr)
                values = json.loads(result.stdout)
                self.assertTrue(values["enabled"])
                self.assertAlmostEqual(values["pad_face"], face)
                self.assertAlmostEqual(values["stroke"], stroke)
                self.assertAlmostEqual(values["closed_gap"], closed)
                self.assertAlmostEqual(values["open_gap"], opened)

    def test_unknown_preset_fails_during_host_only_import(self):
        result = probe(FINGER_SET="typo")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("invalid FINGER_SET", result.stderr)

    def test_invalid_numeric_override_fails_during_host_only_import(self):
        result = probe(FINGER_SET="padded", PAD_FACE_MM="not-a-number")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("must be numeric millimetres", result.stderr)


if __name__ == "__main__":
    unittest.main()
