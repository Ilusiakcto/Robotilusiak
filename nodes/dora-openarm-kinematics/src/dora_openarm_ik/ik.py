# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Dora node: mink-based differential IK solver for OpenArm.

Accepts end-effector pose targets and solves joint angles via mink's QP-based
differential IK. Both arms share one mink.Configuration and one QP solve per
step.

Pose convention (inputs and outputs):  float32[7] = [px, py, pz, qw, qx, qy, qz]
Inputs:
  target_right – float32[7]  right EE target pose
  target_left  – float32[7]  left  EE target pose
  position     – float32[16] current joint state right[8]+left[8] (optional sync)

Outputs:
  position_right – float32[8] solved right arm joint angles
  position_left  – float32[8] solved left arm joint angles
  status         – ["ready"] on startup
"""

from __future__ import annotations

import argparse
import time

import dora
import numpy as np
import pyarrow as pa

from openarm_control import (
    Kinematics,
    register_common_args,
    register_ik_args,
    ik_params_from_args,
    setup_from_args,
)


def _map_trigger_to_gripper_v2(trigger: float, side: str) -> float:
    """V2 gripper: trigger 0.0~1.0 → gripper angle (radians, hinge joint)"""
    if side == "right":
        return (-1.57 / 2.0) * (1.0 - trigger)  # 0→-0.785, 1→0
    else:
        return (1.57 / 2.0) * (1.0 - trigger)   # 0→0.785, 1→0


def _map_trigger_to_gripper_v1(trigger: float, side: str) -> float:
    """V1 gripper: trigger 0.0~1.0 → gripper position (meters, slide joint)
    
    V1 finger joints are slide joints with range [0, 0.044] meters.
    trigger=0 (released) → fully open (0.044m)
    trigger=1 (pressed)  → fully closed (0.0m)
    """
    return 0.044 * (1.0 - trigger)  # 0→0.044, 1→0


def _run(args: argparse.Namespace) -> None:
    setup = setup_from_args(args)
    
    # Select gripper mapping based on robot version
    if args.gripper_type == "v1":
        gripper_fn = _map_trigger_to_gripper_v1
        trigger_threshold = 0.3  # V1: trigger must be >30% pressed to activate
        print("[ik] Using V1 gripper mapping (slide joint, 0-0.044m)")
        print("[ik] V1: Arms will stay still until trigger pressed >30%")
    else:
        gripper_fn = _map_trigger_to_gripper_v2
        trigger_threshold = 0.0  # V2: no threshold (backward compatible)
        print("[ik] Using V2 gripper mapping (hinge joint, radians)")
    
    kin = Kinematics(setup, ik_params_from_args(args))

    node = dora.Node()
    node.send_output("status", pa.array(["ready"]))
    
    # Track trigger states - arms don't move until trigger is pressed
    trigger_active = {"right": False, "left": False}
    trigger_values = {"right": 0.0, "left": 0.0}

    for event in node:
        if event["type"] != "INPUT":
            continue

        eid = event["id"]
        values = np.array(event["value"], dtype=np.float32)

        if eid == "position":
            if values.shape == (16,):
                kin.sync(values)
            continue

        if eid == "target_right" and "right" in kin.setup.sides:
            if values.shape != (7,):
                print(f"Warning: expected target_right[7], got {values.shape}. Skipping.")
                continue
            kin.set_target("right", values)

        elif eid == "target_left" and "left" in kin.setup.sides:
            if values.shape != (7,):
                print(f"Warning: expected target_left[7], got {values.shape}. Skipping.")
                continue
            kin.set_target("left", values)

        elif eid == "trigger_right":
            tval = float(values[0])
            trigger_values["right"] = tval
            trigger_active["right"] = tval > trigger_threshold
            kin.set_gripper("right", gripper_fn(tval, "right"))
            continue

        elif eid == "trigger_left":
            tval = float(values[0])
            trigger_values["left"] = tval
            trigger_active["left"] = tval > trigger_threshold
            kin.set_gripper("left", gripper_fn(tval, "left"))
            continue

        else:
            continue

        if not kin.ready():
            continue

        result = kin.solve()
        if result is None:
            continue

        ts = {"timestamp": time.time_ns()}
        
        # Only send positions for arms with active triggers (V1 safety)
        if trigger_active["right"]:
            node.send_output("position_right", pa.array(result[:8],  type=pa.float32()), ts)
        if trigger_active["left"]:
            node.send_output("position_left",  pa.array(result[8:16], type=pa.float32()), ts)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Mink IK dora node – OpenArm end-effector pose → joint angles"
    )
    register_common_args(parser)
    register_ik_args(parser)
    parser.add_argument("--gripper-type", choices=["v1", "v2"], default="v2",
                        help="Gripper type: v1 (slide joint, 0-0.044m) or v2 (hinge joint, radians)")
    args = parser.parse_args()
    _run(args)


if __name__ == "__main__":
    main()
