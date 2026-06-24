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

"""Dora node: Pyroki-based IK solver for OpenArm.

Uses Pyroki (JAX-based) cost-based optimization for IK solving.
This approach naturally avoids joint overloads through limit_cost penalties.

Pose convention (inputs and outputs):  float32[7] = [px, py, pz, qw, qx, qy, qz]
Inputs:
  target_right – float32[7]  right EE target pose
  target_left  – float32[7]  left  EE target pose
  state_right  – StructArray with qpos field (actual robot position)
  state_left   – StructArray with qpos field (actual robot position)
  trigger_right – float32[1] trigger value for right arm
  trigger_left  – float32[1] trigger value for left arm

Outputs:
  position_right – float32[8] solved right arm joint angles
  position_left  – float32[8] solved left arm joint angles
  status         – ["ready"] on startup
"""

from __future__ import annotations

import argparse
import time
import os

import dora
import numpy as np
import pyarrow as pa
import subprocess
import tempfile
import shutil

import jax
import jax.numpy as jnp
import jaxlie
import jaxls
import pyroki as pk


# Joint offsets: IK model space -> motor space
# These convert IK joint positions to what the motors expect
LEFT_JOINT_OFFSETS = np.array([0.0, 0.506145, -1.570796, -1.745329, 0.0, -0.331612, 1.570796, 0.0], dtype=np.float32)
RIGHT_JOINT_OFFSETS = np.array([0.0, -0.506145, 1.570796, -1.745329, 0.0, 0.331612, -1.570796, 0.0], dtype=np.float32)

# V1 zero position in MODEL/URDF space (NOT motor space!)
_V1_ZERO_POSITION = {
    "right": [0.0, 0.506145, -1.570796, 1.745329, 0.0, -0.331612, 1.570796, 0.0],
    "left":  [0.0, -0.506145, 1.570796, 1.745329, 0.0, 0.331612, -1.570796, 0.0],
}

# Max position change per step (rad) - prevents sudden jumps
_MAX_DELTA_PER_STEP = np.array([
    0.08,  # J1 - shoulder
    0.08,  # J2 - shoulder  
    0.08,  # J3 - elbow
    0.04,  # J4 - elbow (most sensitive)
    0.10,  # J5 - wrist
    0.10,  # J6 - wrist
    0.10,  # J7 - wrist
    0.005, # J8 - gripper (meters for V1)
], dtype=np.float32)


def _map_trigger_to_gripper_v1(trigger: float, side: str) -> float:
    """V1 gripper: trigger 0.0~1.0 → gripper position (meters, slide joint)"""
    return 0.044 * (1.0 - trigger)  # 0→0.044, 1→0


def _smooth_position(target_pos: np.ndarray, base_pos: np.ndarray) -> np.ndarray:
    """Apply per-joint delta limiting for smooth movement."""
    if base_pos is None:
        return target_pos
    delta = target_pos - base_pos
    clamped_delta = np.clip(delta, -_MAX_DELTA_PER_STEP, _MAX_DELTA_PER_STEP)
    return base_pos + clamped_delta


def _get_urdf_path() -> str:
    """Fetch OpenArm URDF from git repo and process xacro."""
    cache_dir = os.path.expanduser("~/.cache/openarm_urdf")
    urdf_cache = os.path.join(cache_dir, "openarm_v10_bimanual.urdf")
    
    # Return cached URDF if exists
    if os.path.exists(urdf_cache):
        print(f"[pyroki-ik] Using cached URDF: {urdf_cache}")
        return urdf_cache
    
    print("[pyroki-ik] Downloading openarm_description repo...")
    os.makedirs(cache_dir, exist_ok=True)
    
    # Clone repo to temp dir
    repo_url = "https://github.com/enactic/openarm_description.git"
    with tempfile.TemporaryDirectory() as tmp_dir:
        repo_dir = os.path.join(tmp_dir, "openarm_description")
        subprocess.run(["git", "clone", "--depth", "1", repo_url, repo_dir], check=True)
        
        # Process xacro
        xacro_path = os.path.join(repo_dir, "assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro")
        print(f"[pyroki-ik] Processing xacro: {xacro_path}")
        
        result = subprocess.run(
            ["xacro", xacro_path, "bimanual:=true", "hand:=true", "ros2_control:=false"],
            capture_output=True, text=True, check=True,
            cwd=repo_dir
        )
        
        # Write URDF to cache
        with open(urdf_cache, "w") as f:
            f.write(result.stdout)
        
        print(f"[pyroki-ik] URDF cached at: {urdf_cache}")
    
    return urdf_cache


class OpenArmPyrokiIK:
    """Pyroki-based IK solver for OpenArm bimanual robot."""
    
    def __init__(self, urdf_path: str = None):
        if urdf_path is None:
            print("[pyroki-ik] Fetching URDF from openarm_description repo...")
            urdf_path = _get_urdf_path()
        print(f"[pyroki-ik] Loading URDF from: {urdf_path}")
        
        # Load robot model
        self.robot = pk.Robot.from_urdf(urdf_path)
        
        # Find end-effector link indices
        self.L_ee_link_idx = self.robot.links.names.index("openarm_left_hand")
        self.R_ee_link_idx = self.robot.links.names.index("openarm_right_hand")
        
        print(f"[pyroki-ik] Left EE link index: {self.L_ee_link_idx}")
        print(f"[pyroki-ik] Right EE link index: {self.R_ee_link_idx}")
        print(f"[pyroki-ik] Actuated joints: {len(self.robot.joints.actuated_names)}")
        
        # Current joint configuration (IK space)
        self.q_current = self._get_default_config()
        
        # Target poses (SE3)
        self.target_L: jaxlie.SE3 | None = None
        self.target_R: jaxlie.SE3 | None = None
        
        # JIT compile the solve function
        self._jit_solve = jax.jit(self._solve_internal)
        
        # Warmup JIT
        self._warmup()
    
    def _get_default_config(self) -> jnp.ndarray:
        """Get default joint configuration."""
        joint_names = list(self.robot.joints.actuated_names)
        
        default_pose = {
            "openarm_left_joint1": 0.0,
            "openarm_left_joint2": -0.8,
            "openarm_left_joint3": 0.0,
            "openarm_left_joint4": 0.3,  # Start extended so elbow can bend
            "openarm_left_joint5": 0.0,
            "openarm_left_joint6": 0.0,
            "openarm_left_joint7": 0.0,
            "openarm_right_joint1": 0.0,
            "openarm_right_joint2": 0.8,
            "openarm_right_joint3": 0.0,
            "openarm_right_joint4": 0.3,  # Start extended so elbow can bend
            "openarm_right_joint5": 0.0,
            "openarm_right_joint6": 0.0,
            "openarm_right_joint7": 0.0,
            "openarm_left_finger_joint1": 0.0,
            "openarm_left_finger_joint2": 0.0,
            "openarm_right_finger_joint1": 0.0,
            "openarm_right_finger_joint2": 0.0,
        }
        
        config_list = []
        for name in joint_names:
            config_list.append(default_pose.get(name, 0.0))
        
        return jnp.array(config_list)
    
    def _build_costs(
        self,
        target_L: jaxlie.SE3 | None,
        target_R: jaxlie.SE3 | None,
        q_current: jnp.ndarray,
    ) -> list:
        """Build cost functions for the IK problem."""
        costs = []
        JointVar = self.robot.joint_var_cls
        
        # Rest cost: penalize moving away from current position (prevents drift)
        if q_current is not None:
            costs.append(
                pk.costs.rest_cost(
                    JointVar(0),
                    rest_pose=q_current,
                    weight=3.0,  # Balanced: allows movement but prevents drift
                )
            )
        
        # Manipulability cost
        costs.append(
            pk.costs.manipulability_cost(
                self.robot,
                JointVar(0),
                jnp.array([self.L_ee_link_idx, self.R_ee_link_idx], dtype=jnp.int32),
                weight=0.01,
            )
        )
        
        # Pose cost for left arm
        if target_L is not None:
            costs.append(
                pk.costs.pose_cost_analytic_jac(
                    self.robot,
                    JointVar(0),
                    target_L,
                    jnp.array(self.L_ee_link_idx, dtype=jnp.int32),
                    pos_weight=100.0,  # Strong target tracking
                    ori_weight=20.0,
                )
            )
        
        # Pose cost for right arm
        if target_R is not None:
            costs.append(
                pk.costs.pose_cost_analytic_jac(
                    self.robot,
                    JointVar(0),
                    target_R,
                    jnp.array(self.R_ee_link_idx, dtype=jnp.int32),
                    pos_weight=100.0,  # Strong target tracking
                    ori_weight=20.0,
                )
            )
        
        # Joint limit cost (KEY: prevents overloads by penalizing near-limit positions)
        costs.append(pk.costs.limit_cost(self.robot, JointVar(0), weight=20.0))
        
        # Self-collision cost
        costs.append(pk.costs.self_collision_cost(self.robot, JointVar(0), weight=10.0))
        
        return costs
    
    def _solve_internal(
        self,
        target_L: jaxlie.SE3 | None,
        target_R: jaxlie.SE3 | None,
        q_current: jnp.ndarray,
    ) -> jnp.ndarray:
        """Internal solve function (JIT-compiled)."""
        costs = self._build_costs(target_L, target_R, q_current)
        
        var_joints = self.robot.joint_var_cls(jnp.array([0]))
        initial_vals = jaxls.VarValues.make(
            [var_joints.with_value(q_current[jnp.newaxis, :])]
        )
        
        problem = jaxls.LeastSquaresProblem(costs, [var_joints])
        
        solution = problem.analyze().solve(
            initial_vals=initial_vals,
            verbose=False,
            linear_solver="dense_cholesky",
            termination=jaxls.TerminationConfig(max_iterations=50),
        )
        
        return solution[var_joints][0]
    
    def _warmup(self):
        """Warmup JIT compilation."""
        print("[pyroki-ik] Warming up JIT...")
        dummy_pose = jaxlie.SE3.identity()
        _ = self._jit_solve(dummy_pose, dummy_pose, self.q_current)
        print("[pyroki-ik] JIT warmup complete")
    
    def set_target(self, side: str, pose: np.ndarray):
        """Set target pose for an arm. pose = [px, py, pz, qw, qx, qy, qz]"""
        position = jnp.array(pose[:3])
        quaternion = jnp.array([pose[3], pose[4], pose[5], pose[6]])  # wxyz
        rotation = jaxlie.SO3(wxyz=quaternion)
        se3_pose = jaxlie.SE3.from_rotation_and_translation(rotation, position)
        
        if side == "left":
            self.target_L = se3_pose
        else:
            self.target_R = se3_pose
    
    def sync(self, joint_positions: np.ndarray, side: str):
        """Sync IK state to actual robot position."""
        # Convert from motor space to IK space
        if side == "right":
            # First 7 joints (skip gripper for now)
            ik_pos = joint_positions[:7] + RIGHT_JOINT_OFFSETS[:7]
        else:
            ik_pos = joint_positions[:7] + LEFT_JOINT_OFFSETS[:7]
        
        # Update relevant joints in q_current
        joint_names = list(self.robot.joints.actuated_names)
        for i, name in enumerate(joint_names):
            if side == "right" and name.startswith("openarm_right_joint"):
                joint_num = int(name[-1]) - 1  # joint1 -> index 0
                if joint_num < 7:
                    self.q_current = self.q_current.at[i].set(ik_pos[joint_num])
            elif side == "left" and name.startswith("openarm_left_joint"):
                joint_num = int(name[-1]) - 1
                if joint_num < 7:
                    self.q_current = self.q_current.at[i].set(ik_pos[joint_num])
    
    def solve(self) -> np.ndarray | None:
        """Solve IK and return joint positions for both arms."""
        if self.target_L is None and self.target_R is None:
            return None
        
        # Solve IK
        q_solved = self._jit_solve(self.target_L, self.target_R, self.q_current)
        
        # Update current state
        self.q_current = q_solved
        
        # Extract joint positions for each arm and convert to motor space
        joint_names = list(self.robot.joints.actuated_names)
        
        right_ik = np.zeros(8, dtype=np.float32)
        left_ik = np.zeros(8, dtype=np.float32)
        
        for i, name in enumerate(joint_names):
            if name.startswith("openarm_right_joint"):
                joint_num = int(name[-1]) - 1
                if joint_num < 7:
                    right_ik[joint_num] = float(q_solved[i])
            elif name.startswith("openarm_left_joint"):
                joint_num = int(name[-1]) - 1
                if joint_num < 7:
                    left_ik[joint_num] = float(q_solved[i])
            elif name == "openarm_right_finger_joint1":
                right_ik[7] = float(q_solved[i])
            elif name == "openarm_left_finger_joint1":
                left_ik[7] = float(q_solved[i])
        
        # Convert IK space to motor space
        right_motor = right_ik - RIGHT_JOINT_OFFSETS
        left_motor = left_ik - LEFT_JOINT_OFFSETS
        
        # Combine into single array [right[8], left[8]]
        return np.concatenate([right_motor, left_motor])
    
    def set_gripper(self, side: str, value: float):
        """Set gripper position."""
        joint_names = list(self.robot.joints.actuated_names)
        target_name = f"openarm_{side}_finger_joint1"
        
        for i, name in enumerate(joint_names):
            if name == target_name:
                self.q_current = self.q_current.at[i].set(value)
                break


def _run(args: argparse.Namespace) -> None:
    # Initialize Pyroki IK (auto-fetches URDF if not provided)
    urdf_path = getattr(args, 'urdf', None)
    if urdf_path and not os.path.exists(urdf_path):
        print(f"[pyroki-ik] Warning: URDF not found at {urdf_path}, will auto-fetch")
        urdf_path = None
    
    ik = OpenArmPyrokiIK(urdf_path)
    
    node = dora.Node()
    node.send_output("status", pa.array(["ready"]))
    
    # Track trigger states
    trigger_active = {"right": False, "left": False}
    trigger_threshold = 0.3
    
    # Track actual robot position from state feedback
    actual_robot_pos = {"right": None, "left": None}
    
    # Track previous commanded position
    prev_cmd = {"right": None, "left": None}
    
    # Track sync state
    synced_on_activation = {"right": False, "left": False}
    
    print("[pyroki-ik] Ready for inputs")
    
    for event in node:
        if event["type"] != "INPUT":
            continue
        
        eid = event["id"]
        raw_value = event["value"]
        
        # Handle state feedback
        if eid == "state_right":
            try:
                if hasattr(raw_value, 'field'):
                    qpos_arr = raw_value.field('qpos')
                    qpos = np.array(qpos_arr, dtype=np.float32)
                    if len(qpos) == 8:
                        actual_robot_pos["right"] = qpos.copy()
            except Exception as e:
                print(f"[pyroki-ik] Warning: Could not parse state_right: {e}")
            continue
        
        if eid == "state_left":
            try:
                if hasattr(raw_value, 'field'):
                    qpos_arr = raw_value.field('qpos')
                    qpos = np.array(qpos_arr, dtype=np.float32)
                    if len(qpos) == 8:
                        actual_robot_pos["left"] = qpos.copy()
            except Exception as e:
                print(f"[pyroki-ik] Warning: Could not parse state_left: {e}")
            continue
        
        values = np.array(raw_value, dtype=np.float32)
        
        # Handle targets
        if eid == "target_right":
            if values.shape == (7,):
                ik.set_target("right", values)
        
        elif eid == "target_left":
            if values.shape == (7,):
                ik.set_target("left", values)
        
        # Handle triggers
        elif eid == "trigger_right":
            tval = float(values[0])
            was_active = trigger_active["right"]
            trigger_active["right"] = tval > trigger_threshold
            ik.set_gripper("right", _map_trigger_to_gripper_v1(tval, "right"))
            
            # Sync on activation
            if trigger_active["right"] and not was_active:
                if actual_robot_pos["right"] is not None and not synced_on_activation["right"]:
                    ik.sync(actual_robot_pos["right"], "right")
                    synced_on_activation["right"] = True
                    print(f"[pyroki-ik] RIGHT ARM ACTIVATED: Synced to robot")
            elif not trigger_active["right"] and was_active:
                synced_on_activation["right"] = False
                print(f"[pyroki-ik] RIGHT ARM DEACTIVATED")
            continue
        
        elif eid == "trigger_left":
            tval = float(values[0])
            was_active = trigger_active["left"]
            trigger_active["left"] = tval > trigger_threshold
            ik.set_gripper("left", _map_trigger_to_gripper_v1(tval, "left"))
            
            # Sync on activation
            if trigger_active["left"] and not was_active:
                if actual_robot_pos["left"] is not None and not synced_on_activation["left"]:
                    ik.sync(actual_robot_pos["left"], "left")
                    synced_on_activation["left"] = True
                    print(f"[pyroki-ik] LEFT ARM ACTIVATED: Synced to robot")
            elif not trigger_active["left"] and was_active:
                synced_on_activation["left"] = False
                print(f"[pyroki-ik] LEFT ARM DEACTIVATED")
            continue
        
        else:
            continue
        
        # Solve IK
        result = ik.solve()
        if result is None:
            continue
        
        ts = {"timestamp": time.time_ns()}
        
        # Send positions for active arms with rate limiting
        if trigger_active["right"]:
            pos_right = result[:8].copy()
            if actual_robot_pos["right"] is not None:
                pos_right = _smooth_position(pos_right, actual_robot_pos["right"])
            prev_cmd["right"] = pos_right.copy()
            node.send_output("position_right", pa.array(pos_right, type=pa.float32()), ts)
        
        if trigger_active["left"]:
            pos_left = result[8:16].copy()
            if actual_robot_pos["left"] is not None:
                pos_left = _smooth_position(pos_left, actual_robot_pos["left"])
            prev_cmd["left"] = pos_left.copy()
            node.send_output("position_left", pa.array(pos_left, type=pa.float32()), ts)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Pyroki IK dora node – OpenArm end-effector pose → joint angles"
    )
    parser.add_argument("--urdf", required=False, default=None,
                        help="Path to OpenArm URDF file (auto-fetched if not provided)")
    args = parser.parse_args()
    _run(args)


if __name__ == "__main__":
    main()
