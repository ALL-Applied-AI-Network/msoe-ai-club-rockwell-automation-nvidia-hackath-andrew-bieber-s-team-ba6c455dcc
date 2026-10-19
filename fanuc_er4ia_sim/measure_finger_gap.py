#!/usr/bin/env python3
"""Measure the mounted gripper's finger gap in Isaac Sim, open and closed.

    python measure_finger_gap.py --finger-set padded      # expect ~8 closed / 20 open
    python measure_finger_gap.py --finger-set scallop     # expect ~4 closed / 16 open

Gap is computed from the simulated jaw body positions, so it reflects the joint
limits and drives, not just the CAD numbers.
"""

import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument("--finger-set", default=os.environ.get("FINGER_SET", "padded"))
parser.add_argument("--headless", action="store_true")
args = parser.parse_args()
os.environ["FINGER_SET"] = args.finger_set  # must be set before scene_config is imported

import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": args.headless})

import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.types import ArticulationAction

import cell
import scene_config as C

world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 30)
stage = omni.usd.get_context().get_stage()
cell.build(stage, with_objects=False)
world.reset()

arm = SingleArticulation(prim_path=C.ROBOT_PRIM, name="gap_probe")
arm.initialize()
names = list(arm.dof_names)
joints = {name: index for index, name in enumerate(names)}
jaws = {tag: SingleXFormPrim(path) for tag, path in cell.JAW_PRIMS.items()}


def settle(jaw):
    for _ in range(180):
        full = arm.get_joint_positions().copy()
        for name, value in cell.jaw_targets(jaw).items():
            full[joints[name]] = value
        arm.apply_action(ArticulationAction(joint_positions=full))
        world.step(render=not args.headless)
    left = np.asarray(jaws["L"].get_world_pose()[0], dtype=float)
    right = np.asarray(jaws["R"].get_world_pose()[0], dtype=float)
    # Jaw origins sit FINGER_WIDTH/2 outside each root face, so:
    root_gap = float(np.linalg.norm(left - right)) - C.FINGER_WIDTH
    tip_gap = root_gap + 2.0 * C.FINGER_INNER_TIP  # pad faces (padded) / hook tips (scallop)
    return root_gap, tip_gap


print(f"[gap] finger set: {C.FINGER_SET}")
for label, jaw, expect in (
    ("closed", C.JAW_CLOSED, 2.0 * C.FINGER_INNER_TIP),
    ("open", C.JAW_OPEN, 2.0 * (C.JAW_OPEN + C.FINGER_INNER_TIP)),
):
    root, tip = settle(jaw)
    flag = "OK" if abs(tip - expect) < 0.0005 else "MISMATCH"
    print(
        f"[gap] {label:6s} root faces {root * 1000:6.2f} mm | "
        f"tip/pad gap {tip * 1000:6.2f} mm (expected {expect * 1000:.2f}) {flag}"
    )

app.close()