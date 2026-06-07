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


# V1 zero position (from LeRobot calibration with homing_offset=0)
# All joints at 0 rad = calibrated zero position (arms straight down)
# Joint limits are ±90° (±1.5708 rad) from this zero
_V1_ZERO_POSITION = {
    "right": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # All at zero
    "left":  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # All at zero
}

# Max position change per step (rad) - prevents sudden jumps that fault motors
# Lower = smoother but slower tracking, Higher = faster but can fault
# At 50Hz, 0.1 rad/step = 5 rad/s max velocity
_MAX_DELTA_PER_STEP = np.array([
    0.15,  # J1 - shoulder
    0.15,  # J2 - shoulder  
    0.15,  # J3 - elbow
    0.08,  # J4 - elbow (most sensitive, prone to faulting)
    0.2,   # J5 - wrist
    0.2,   # J6 - wrist
    0.2,   # J7 - wrist
    0.01,  # J8 - gripper (meters for V1)
], dtype=np.float32)


def _smooth_position(new_pos: np.ndarray, prev_pos: np.ndarray) -> np.ndarray:
    """Rate-limit position changes to prevent motor faults."""
    if prev_pos is None:
        return new_pos
    
    delta = new_pos - prev_pos
    # Clamp each joint's delta to max allowed
    clamped_delta = np.clip(delta, -_MAX_DELTA_PER_STEP, _MAX_DELTA_PER_STEP)
    return prev_pos + clamped_delta


def _run(args: argparse.Namespace) -> None:
    setup = setup_from_args(args)
    
    # Select gripper mapping based on robot version
    if args.gripper_type == "v1":
        gripper_fn = _map_trigger_to_gripper_v1
        trigger_threshold = 0.3  # V1: trigger must be >30% pressed to activate
        print("[ik] Using V1 gripper mapping (slide joint, 0-0.044m)")
        print("[ik] V1: Arms will stay still until trigger pressed >30%")
        
        # Initialize MuJoCo to zero position (matches LeRobot calibration)
        # Robot should be at zero position before starting teleoperation
        import mujoco
        qpos = setup.data.qpos
        
        # Find joint indices and set to zero position
        for side, zero_pos in _V1_ZERO_POSITION.items():
            for i in range(8):  # 7 arm joints + 1 gripper
                joint_name = f"openarm_{side}_joint{i+1}" if i < 7 else f"openarm_{side}_finger_joint"
                try:
                    jnt_id = mujoco.mj_name2id(setup.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                    if jnt_id >= 0:
                        qpos_adr = setup.model.jnt_qposadr[jnt_id]
                        qpos[qpos_adr] = zero_pos[i]
                except:
                    pass
        
        mujoco.mj_forward(setup.model, setup.data)
        print(f"[ik] V1 initialized to zero position (all joints at 0, per LeRobot calibration)")
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
    first_activation = {"right": True, "left": True}  # Track first trigger press
    
    # Track previous positions for smoothing (prevents sudden jumps)
    prev_pos = {"right": None, "left": None}

    for event in node:
        if event["type"] != "INPUT":
            continue

        eid = event["id"]
        values = np.array(event["value"], dtype=np.float32)

        if eid == "position":
            # Sync IK internal state to robot's actual position
            # This is critical for smooth operation
            if values.shape == (16,):
                kin.sync(values)
                print(f"[ik] Synced to robot position: R-J4={values[3]:.3f}, L-J4={values[11]:.3f}")
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
            pos_right = result[:8].copy()
            if args.gripper_type == "v1":
                pos_right = _smooth_position(pos_right, prev_pos["right"])
                prev_pos["right"] = pos_right.copy()
            node.send_output("position_right", pa.array(pos_right, type=pa.float32()), ts)
        if trigger_active["left"]:
            pos_left = result[8:16].copy()
            if args.gripper_type == "v1":
                pos_left = _smooth_position(pos_left, prev_pos["left"])
                prev_pos["left"] = pos_left.copy()
            node.send_output("position_left", pa.array(pos_left, type=pa.float32()), ts)


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
