"""Host-only regression tests for padded-gripper configuration."""

import json
import os
from pathlib import Path
import subprocess
import sys
import unittest


SIM_DIR = Path(__file__).resolve().parents[1] / "fanuc_er4ia_sim"
DIMENSIONS = Path(__file__).resolve().parents[1] / "hardware" / "dual_zone_v2" / "dimensions.json"


def probe(**environment):
    env = os.environ.copy()
    for name in ("FINGER_SET", "PAD_FACE_MM", "JAW_STROKE_MM"):
        env.pop(name, None)
    env.update(environment)
    code = """
import json
import scene_config as C
import padded_fingers as P
import dual_zone as D
print(json.dumps({
    "finger_set": C.FINGER_SET,
    "stroke": C.JAW_STROKE,
    "pad_face": C.PAD_FACE,
    "enabled": P.enabled(),
    "closed_gap": P.closed_gap(),
    "open_gap": P.open_gap(),
    "primary_rail_length": D.PRIMARY_RAIL_LENGTH,
    "primary_rail_span": D.PRIMARY_RAIL_SPAN,
    "primary_pad_profile": D.PRIMARY_PAD_PROFILE,
    "wide_rail_length": D.WIDE_RAIL_LENGTH,
    "extra_box_names": [entry[0] for entry in D.extra_boxes(1)],
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

    def test_dual_v2_screw_contact_matches_candidate_cad(self):
        result = probe(FINGER_SET="dual_v2")
        self.assertEqual(result.returncode, 0, result.stderr)
        values = json.loads(result.stdout)
        dimensions = json.loads(DIMENSIONS.read_text())

        self.assertAlmostEqual(values["stroke"], 0.006)
        self.assertAlmostEqual(values["closed_gap"], 0.0052)
        self.assertAlmostEqual(values["open_gap"], 0.0172)
        self.assertAlmostEqual(values["primary_rail_length"] * 1000,
                               dimensions["primary_rail_length_x"])
        self.assertEqual(values["primary_rail_span"], [-0.021, 0.003])
        self.assertAlmostEqual(values["primary_rail_span"][1], 0.003)
        self.assertLess(values["primary_rail_span"][0], -0.003)
        self.assertEqual(values["primary_pad_profile"], "flat_compliant")
        self.assertEqual(dimensions["primary_v_groove_depth"], 0.0)
        self.assertAlmostEqual(values["wide_rail_length"], 0.050)
        self.assertIn("PrimaryExtension", values["extra_box_names"])
        self.assertIn("PrimaryPadExtension", values["extra_box_names"])

    def test_original_dual_profile_keeps_its_six_mm_primary_contact(self):
        result = probe(FINGER_SET="dual")
        self.assertEqual(result.returncode, 0, result.stderr)
        values = json.loads(result.stdout)
        self.assertAlmostEqual(values["primary_rail_length"], 0.006)
        self.assertEqual(values["primary_rail_span"], [-0.003, 0.003])
        self.assertNotIn("PrimaryExtension", values["extra_box_names"])


if __name__ == "__main__":
    unittest.main()

