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


# V1 zero position in MODEL/URDF space (NOT motor space!)
# Physical arms-down = motor space zeros, but IK operates in model space.
# Model-space arms-down = −joint_offsets (from config files).
# These values ensure IK's initial belief matches physical arms-down position.
_V1_ZERO_POSITION = {
    "right": [0.0, 0.506145, -1.570796, 1.745329, 0.0, -0.331612, 1.570796, 0.0],
    "left":  [0.0, -0.506145, 1.570796, 1.745329, 0.0, 0.331612, -1.570796, 0.0],
}

# Max position change per step (rad) - prevents sudden jumps that fault motors
# Lower = smoother but slower tracking, Higher = faster but can fault
# At 50Hz, 0.1 rad/step = 5 rad/s max velocity
_MAX_DELTA_PER_STEP = np.array([
    0.08,  # J1 - shoulder
    0.08,  # J2 - shoulder  
    0.08,  # J3 - elbow
    0.04,  # J4 - elbow (most sensitive, prone to faulting)
    0.10,  # J5 - wrist
    0.10,  # J6 - wrist
    0.10,  # J7 - wrist
    0.005, # J8 - gripper (meters for V1)
], dtype=np.float32)

# VERY conservative limits for ramp-up period after activation
_RAMPUP_DELTA_PER_STEP = np.array([
    0.02,  # J1
    0.02,  # J2
    0.02,  # J3
    0.01,  # J4 - extra conservative
    0.03,  # J5
    0.03,  # J6
    0.03,  # J7
    0.002, # J8
], dtype=np.float32)

# Number of frames for ramp-up period after activation
_RAMPUP_FRAMES = 25  # ~0.5 seconds at 50Hz


def _smooth_position_safe(target_pos: np.ndarray, actual_pos: np.ndarray, prev_cmd: np.ndarray, frames_since_activation: int) -> np.ndarray:
    """Safe smoothing: prefer ACTUAL position, fallback to prev_cmd if no feedback.
    
    Uses conservative limits during ramp-up period after trigger activation.
    """
    # Use conservative limits during ramp-up, normal limits after
    if frames_since_activation < _RAMPUP_FRAMES:
        max_delta = _RAMPUP_DELTA_PER_STEP
    else:
        max_delta = _MAX_DELTA_PER_STEP
    
    # Prefer actual position, fallback to prev_cmd
    if actual_pos is not None:
        base_pos = actual_pos
    elif prev_cmd is not None:
        base_pos = prev_cmd
    else:
        # First command ever - just return target (will be smoothed next frame)
        return target_pos
    
    delta = target_pos - base_pos
    clamped_delta = np.clip(delta, -max_delta, max_delta)
    return base_pos + clamped_delta


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
    
    # Track actual robot position from state feedback
    actual_robot_pos = {"right": None, "left": None}
    
    # Track previous commanded position (fallback if no state feedback)
    prev_cmd = {"right": None, "left": None}
    
    # Track frames since activation for ramp-up period
    frames_since_activation = {"right": 0, "left": 0}
    
    # Track if we've synced IK to actual robot position on first activation
    synced_on_activation = {"right": False, "left": False}
    
    # Track if we've logged first state reception (diagnostic)
    first_state_logged = {"right": False, "left": False}
    
    # === DIAGNOSTIC: Print expected positions at startup ===
    print("\n" + "="*70)
    print("[ik] DIAGNOSTIC: IK Node Starting - Expected Calibration Values")
    print("="*70)
    if args.gripper_type == "v1":
        print(f"[ik] _V1_ZERO_POSITION (what IK expects when arms are down):")
        print(f"[ik]   RIGHT: {_V1_ZERO_POSITION['right'][:4]}...")
        print(f"[ik]   LEFT:  {_V1_ZERO_POSITION['left'][:4]}...")
        print(f"[ik]")
        print(f"[ik] If robot motor positions DON'T match these when arms are down,")
        print(f"[ik] update _V1_ZERO_POSITION to match actual motor readings.")
    print("="*70 + "\n")

    for event in node:
        if event["type"] != "INPUT":
            continue

        eid = event["id"]
        raw_value = event["value"]

        if eid == "position":
            # Legacy sync input (combined 16-joint state)
            values = np.array(raw_value, dtype=np.float32)
            if values.shape == (16,):
                kin.sync(values)
                actual_robot_pos["right"] = values[:8].copy()
                actual_robot_pos["left"] = values[8:16].copy()
                print(f"[ik] Synced to robot position: R-J4={values[3]:.3f}, L-J4={values[11]:.3f}")
            continue

        # Handle state feedback from followers (closed-loop control)
        # State is a PyArrow StructArray with fields: qpos, qvel, qtorque
        if eid == "state_right":
            try:
                # PyArrow StructArray: extract qpos field
                if hasattr(raw_value, 'field'):
                    qpos_arr = raw_value.field('qpos')
                    qpos = np.array(qpos_arr, dtype=np.float32)
                elif isinstance(raw_value, pa.StructArray):
                    qpos_arr = raw_value.field('qpos')
                    qpos = np.array(qpos_arr, dtype=np.float32)
                else:
                    qpos = None
                if qpos is not None and len(qpos) == 8:
                    old_pos = actual_robot_pos["right"]
                    actual_robot_pos["right"] = qpos.copy()
                    
                    # === DIAGNOSTIC: First state reception comparison ===
                    if not first_state_logged["right"] and args.gripper_type == "v1":
                        first_state_logged["right"] = True
                        expected = np.array(_V1_ZERO_POSITION["right"], dtype=np.float32)
                        delta = qpos - expected
                        max_delta = np.max(np.abs(delta))
                        print("\n" + "="*70)
                        print("[ik] DIAGNOSTIC: FIRST STATE RECEIVED - RIGHT ARM")
                        print("="*70)
                        print(f"[ik] ACTUAL motor pos:   [{qpos[0]:+.4f}, {qpos[1]:+.4f}, {qpos[2]:+.4f}, {qpos[3]:+.4f}, ...]")
                        print(f"[ik] EXPECTED (IK init): [{expected[0]:+.4f}, {expected[1]:+.4f}, {expected[2]:+.4f}, {expected[3]:+.4f}, ...]")
                        print(f"[ik] DELTA:              [{delta[0]:+.4f}, {delta[1]:+.4f}, {delta[2]:+.4f}, {delta[3]:+.4f}, ...]")
                        print(f"[ik] MAX DELTA: {max_delta:.4f} rad ({np.rad2deg(max_delta):.1f} deg)")
                        if max_delta > 0.3:
                            print(f"[ik] *** WARNING: Large mismatch! This will cause motor faults! ***")
                            print(f"[ik] *** Update _V1_ZERO_POSITION in ik.py to match ACTUAL values ***")
                        else:
                            print(f"[ik] OK: Positions match within tolerance.")
                        print("="*70 + "\n")
                    elif old_pos is None or np.max(np.abs(qpos - old_pos)) > 0.01:
                        print(f"[ik] STATE_RIGHT updated: J1-4=[{qpos[0]:.3f}, {qpos[1]:.3f}, {qpos[2]:.3f}, {qpos[3]:.3f}]")
            except Exception as e:
                print(f"[ik] Warning: Could not parse state_right: {e}")
            continue

        if eid == "state_left":
            try:
                # PyArrow StructArray: extract qpos field
                if hasattr(raw_value, 'field'):
                    qpos_arr = raw_value.field('qpos')
                    qpos = np.array(qpos_arr, dtype=np.float32)
                elif isinstance(raw_value, pa.StructArray):
                    qpos_arr = raw_value.field('qpos')
                    qpos = np.array(qpos_arr, dtype=np.float32)
                else:
                    qpos = None
                if qpos is not None and len(qpos) == 8:
                    old_pos = actual_robot_pos["left"]
                    actual_robot_pos["left"] = qpos.copy()
                    
                    # === DIAGNOSTIC: First state reception comparison ===
                    if not first_state_logged["left"] and args.gripper_type == "v1":
                        first_state_logged["left"] = True
                        expected = np.array(_V1_ZERO_POSITION["left"], dtype=np.float32)
                        delta = qpos - expected
                        max_delta = np.max(np.abs(delta))
                        print("\n" + "="*70)
                        print("[ik] DIAGNOSTIC: FIRST STATE RECEIVED - LEFT ARM")
                        print("="*70)
                        print(f"[ik] ACTUAL motor pos:   [{qpos[0]:+.4f}, {qpos[1]:+.4f}, {qpos[2]:+.4f}, {qpos[3]:+.4f}, ...]")
                        print(f"[ik] EXPECTED (IK init): [{expected[0]:+.4f}, {expected[1]:+.4f}, {expected[2]:+.4f}, {expected[3]:+.4f}, ...]")
                        print(f"[ik] DELTA:              [{delta[0]:+.4f}, {delta[1]:+.4f}, {delta[2]:+.4f}, {delta[3]:+.4f}, ...]")
                        print(f"[ik] MAX DELTA: {max_delta:.4f} rad ({np.rad2deg(max_delta):.1f} deg)")
                        if max_delta > 0.3:
                            print(f"[ik] *** WARNING: Large mismatch! This will cause motor faults! ***")
                            print(f"[ik] *** Update _V1_ZERO_POSITION in ik.py to match ACTUAL values ***")
                        else:
                            print(f"[ik] OK: Positions match within tolerance.")
                        print("="*70 + "\n")
                    elif old_pos is None or np.max(np.abs(qpos - old_pos)) > 0.01:
                        print(f"[ik] STATE_LEFT updated: J1-4=[{qpos[0]:.3f}, {qpos[1]:.3f}, {qpos[2]:.3f}, {qpos[3]:.3f}]")
            except Exception as e:
                print(f"[ik] Warning: Could not parse state_left: {e}")
            continue
        
        # For other events, convert to numpy array
        values = np.array(raw_value, dtype=np.float32)

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
            was_active = trigger_active["right"]
            trigger_values["right"] = tval
            trigger_active["right"] = tval > trigger_threshold
            kin.set_gripper("right", gripper_fn(tval, "right"))
            
            # CRITICAL: On first activation, sync IK to actual robot position
            if trigger_active["right"] and not was_active:
                if actual_robot_pos["right"] is not None and not synced_on_activation["right"]:
                    # Build full state array for sync (right + left)
                    right_pos = actual_robot_pos["right"]
                    left_pos = actual_robot_pos["left"] if actual_robot_pos["left"] is not None else np.zeros(8, dtype=np.float32)
                    full_state = np.concatenate([right_pos, left_pos])
                    print(f"[ik] RIGHT SYNC: actual_pos={right_pos[:4]}")
                    kin.sync(full_state)
                    # Reset frame counter for ramp-up period
                    frames_since_activation["right"] = 0
                    synced_on_activation["right"] = True
                    print(f"[ik] RIGHT ARM ACTIVATED: Synced IK to actual robot. J1-4=[{right_pos[0]:.3f}, {right_pos[1]:.3f}, {right_pos[2]:.3f}, {right_pos[3]:.3f}]")
                else:
                    print(f"[ik] RIGHT ARM ACTIVATED: NO SYNC! actual_pos={'None' if actual_robot_pos['right'] is None else 'exists'}, already_synced={synced_on_activation['right']}")
            elif not trigger_active["right"] and was_active:
                # Reset sync flag when trigger released
                synced_on_activation["right"] = False
                frames_since_activation["right"] = 0
                print(f"[ik] RIGHT ARM DEACTIVATED")
            continue

        elif eid == "trigger_left":
            tval = float(values[0])
            was_active = trigger_active["left"]
            trigger_values["left"] = tval
            trigger_active["left"] = tval > trigger_threshold
            kin.set_gripper("left", gripper_fn(tval, "left"))
            
            # CRITICAL: On first activation, sync IK to actual robot position
            if trigger_active["left"] and not was_active:
                if actual_robot_pos["left"] is not None and not synced_on_activation["left"]:
                    # Build full state array for sync (right + left)
                    right_pos = actual_robot_pos["right"] if actual_robot_pos["right"] is not None else np.zeros(8, dtype=np.float32)
                    left_pos = actual_robot_pos["left"]
                    full_state = np.concatenate([right_pos, left_pos])
                    print(f"[ik] LEFT SYNC: actual_pos={left_pos[:4]}")
                    kin.sync(full_state)
                    # Reset frame counter for ramp-up period
                    frames_since_activation["left"] = 0
                    synced_on_activation["left"] = True
                    print(f"[ik] LEFT ARM ACTIVATED: Synced IK to actual robot. J1-4=[{left_pos[0]:.3f}, {left_pos[1]:.3f}, {left_pos[2]:.3f}, {left_pos[3]:.3f}]")
                else:
                    print(f"[ik] LEFT ARM ACTIVATED: NO SYNC! actual_pos={'None' if actual_robot_pos['left'] is None else 'exists'}, already_synced={synced_on_activation['left']}")
            elif not trigger_active["left"] and was_active:
                # Reset sync flag when trigger released
                synced_on_activation["left"] = False
                frames_since_activation["left"] = 0
                print(f"[ik] LEFT ARM DEACTIVATED")
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
            # DEBUG: Log command vs actual position
            if actual_robot_pos["right"] is not None:
                delta = pos_right - actual_robot_pos["right"]
                max_delta = np.max(np.abs(delta))
                if max_delta > 0.1:  # Log if delta > 0.1 rad
                    print(f"[ik] RIGHT CMD delta > 0.1rad! max={max_delta:.3f} cmd={pos_right[:4]} actual={actual_robot_pos['right'][:4]}")
            # V2 APPROACH: No smoothing - let IK damping and motor controllers handle it
            # if args.gripper_type == "v1":
            #     # Safe smoothing with ramp-up, prefers actual pos, falls back to prev_cmd
            #     pos_right = _smooth_position_safe(pos_right, actual_robot_pos["right"], prev_cmd["right"], frames_since_activation["right"])
            #     prev_cmd["right"] = pos_right.copy()
            #     frames_since_activation["right"] += 1
            node.send_output("position_right", pa.array(pos_right, type=pa.float32()), ts)
        if trigger_active["left"]:
            pos_left = result[8:16].copy()
            # DEBUG: Log command vs actual position
            if actual_robot_pos["left"] is not None:
                delta = pos_left - actual_robot_pos["left"]
                max_delta = np.max(np.abs(delta))
                if max_delta > 0.1:  # Log if delta > 0.1 rad
                    print(f"[ik] LEFT CMD delta > 0.1rad! max={max_delta:.3f} cmd={pos_left[:4]} actual={actual_robot_pos['left'][:4]}")
            # V2 APPROACH: No smoothing - let IK damping and motor controllers handle it
            # if args.gripper_type == "v1":
            #     # Safe smoothing with ramp-up, prefers actual pos, falls back to prev_cmd
            #     pos_left = _smooth_position_safe(pos_left, actual_robot_pos["left"], prev_cmd["left"], frames_since_activation["left"])
            #     prev_cmd["left"] = pos_left.copy()
            #     frames_since_activation["left"] += 1
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
