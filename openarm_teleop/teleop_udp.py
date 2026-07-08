#!/usr/bin/env python3
"""
OpenArm VR Teleop via UDP (No Adamo dependency)
================================================

This module provides VR teleoperation for the OpenArm robot using UDP input
from the Meta Quest APK. It uses the same IK logic as the original teleop.py
but replaces the Adamo network with direct UDP communication.

Usage:
    python -m openarm_teleop.teleop_udp --right-can can0 --left-can can1

The Quest APK sends JSON packets over UDP with the following structure:
- lc / rc: left/right controller poses (x, y, z, qx, qy, qz, qw)
- rf: reference/head pose
- lt / rt: left/right trigger (0.0-1.0)
- lg / rg: left/right grip (0.0-1.0)
- v / vl / vr: validity flags
"""

from __future__ import annotations

import argparse
import json
import os
import select
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

# Add script directory to path for direct execution
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

import numpy as np
from scipy.spatial.transform import Rotation

from kinematics import OpenArmKinematics
from can_motor import (
    OpenArmCAN,
    pack_mit,
    GRIPPER_MOTOR_TYPE,
    GRIPPER_SEND_ID,
)

# ═══════════════════════════════════════════════════════════════════════════════
# Constants
# ═══════════════════════════════════════════════════════════════════════════════

DEFAULT_UDP_HOST = "0.0.0.0"
DEFAULT_UDP_PORT = 5006

GRIPPER_OPEN = -1.0
GRIPPER_CLOSED = 0.0

# ── Exact Adamo constants ──────────────────────────────────────────────

# Bent-elbow home configs (j4 = elbow bend forward, j2 = 0 keeps arm at side)
HOME_JOINTS_RIGHT = np.array([0, 0, 0, 1.2, 0, 0, 0])
HOME_JOINTS_LEFT = np.array([0, 0, 0, 1.2, 0, 0, 0])

# Joint limits from URDF [lower, upper] for each of 7 joints
JOINT_LIMITS_RIGHT = np.array([
    [-1.396263, 3.490659],   # j1: base yaw
    [-0.174533, 3.316125],   # j2: shoulder pitch
    [-1.570796, 1.570796],   # j3: upper arm rotation
    [0.0, 2.443461],         # j4: elbow (bends one way only)
    [-1.570796, 1.570796],   # j5: forearm rotation
    [-0.785398, 0.785398],   # j6: wrist pitch
    [-1.570796, 1.570796],   # j7: wrist roll
])
JOINT_LIMITS_LEFT = np.array([
    [-3.490659, 1.396263],
    [-3.316125, 0.174533],
    [-1.570796, 1.570796],
    [0.0, 2.443461],
    [-1.570796, 1.570796],
    [-0.785398, 0.785398],
    [-1.570796, 1.570796],
])

# ── MID TUNING: halfway between normal and pong ──
DLS_LAMBDA_MAX = 0.035     # Mid damping (normal 0.05, pong 0.02)
DLS_SIGMA_THRESH = 0.04    # Mid singularity threshold (normal 0.05, pong 0.03)
NULL_SPACE_GAIN = 0.35     # Mid pull toward home (normal 0.5, pong 0.2)
IK_SUB_ITERS = 3           # Sub-iterations per control cycle
ORIENT_WEIGHT = 0.225      # Mid orientation priority (normal 0.3, pong 0.15)
JOINT_LIMIT_K = 10.0       # Joint-limit avoidance gain (unchanged)
MANIP_GAIN = 0.0           # Disabled (unchanged)


# ═══════════════════════════════════════════════════════════════════════════════
# UDP VR Receiver (replaces Adamo VRInput)
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class VRControllerState:
    """State of a single VR controller."""
    position: Optional[np.ndarray] = None      # [x, y, z] in meters
    orientation: Optional[np.ndarray] = None   # [qw, qx, qy, qz] quaternion
    trigger: float = 0.0                        # 0.0-1.0
    grip: float = 0.0                           # 0.0-1.0
    updated: bool = False                       # True if new data this frame


class UdpVRInput:
    """
    UDP-based VR input receiver for Meta Quest.
    
    Replaces Adamo's pub/sub with direct UDP socket listening.
    JSON format matches the Quest APK output.
    """

    def __init__(self, host: str = DEFAULT_UDP_HOST, port: int = DEFAULT_UDP_PORT):
        self._host = host
        self._port = port
        self._lock = threading.Lock()
        self._latest: Optional[dict] = None
        self._prev_msg: Optional[dict] = None
        self._running = True
        self._reset_requested = False
        
        # Start background receiver thread
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def _recv_loop(self) -> None:
        """Background thread that receives UDP packets."""
        while self._running:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    sock.bind((self._host, self._port))
                    sock.settimeout(1.0)
                    print(f"[VR] Listening on UDP {self._host}:{self._port}")

                    while self._running:
                        try:
                            data, _ = sock.recvfrom(4096)
                            msg = self._parse_packet(data)
                            
                            # Drain queued packets, keep only freshest
                            while select.select([sock], [], [], 0.0)[0]:
                                data, _ = sock.recvfrom(4096)
                                parsed = self._parse_packet(data)
                                if parsed is not None:
                                    msg = parsed

                            if msg is not None:
                                with self._lock:
                                    self._latest = msg
                                    # Check for reset button (X button)
                                    if msg.get("x", False):
                                        self._reset_requested = True

                        except TimeoutError:
                            continue
                        except Exception as e:
                            print(f"[VR] Recv error: {e}")

            except OSError as e:
                print(f"[VR] Socket error: {e}")
                if self._running:
                    time.sleep(1.0)

    def _parse_packet(self, data: bytes) -> Optional[dict]:
        """Parse a UDP packet as JSON."""
        try:
            line = data.decode("utf-8", errors="replace").strip()
            if not line:
                return None
            return json.loads(line)
        except json.JSONDecodeError:
            return None

    def _parse_controller(self, c: dict) -> tuple[np.ndarray, np.ndarray]:
        """
        Parse controller pose from JSON dict.
        
        Converts Unity left-handed coords to right-handed:
        - Position: z → -z
        - Quaternion: qx → -qx, qy → -qy
        
        Returns (position, quaternion) where quaternion is [qw, qx, qy, qz].
        """
        pos = np.array([c["x"], c["y"], -c["z"]], dtype=np.float64)
        quat = np.array([c["qw"], -c["qx"], -c["qy"], c["qz"]], dtype=np.float64)
        return pos, quat

    def get_state(self) -> tuple[VRControllerState, VRControllerState]:
        """
        Get current state of both controllers.
        
        Returns (left_state, right_state).
        """
        with self._lock:
            msg = self._latest
            prev = self._prev_msg
            self._prev_msg = msg

        left = VRControllerState()
        right = VRControllerState()

        if msg is None:
            return left, right

        # Check if data actually updated
        left.updated = (msg != prev)
        right.updated = (msg != prev)

        # Parse left controller
        if "lc" in msg and msg["lc"] is not None:
            lc = msg["lc"]
            left.position, left.orientation = self._parse_controller(lc)
        left.trigger = float(msg.get("lt", 0.0))
        left.grip = float(msg.get("lg", 0.0))

        # Parse right controller
        if "rc" in msg and msg["rc"] is not None:
            rc = msg["rc"]
            right.position, right.orientation = self._parse_controller(rc)
        right.trigger = float(msg.get("rt", 0.0))
        right.grip = float(msg.get("rg", 0.0))

        return left, right

    def get_head_pose(self) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get head/reference pose for calibration.
        
        Returns (position, quaternion) or (None, None) if not available.
        """
        with self._lock:
            msg = self._latest

        if msg is None or "rf" not in msg or msg["rf"] is None:
            return None, None

        rf = msg["rf"]
        pos, quat = self._parse_controller(rf)
        return pos, quat

    def consume_reset(self) -> bool:
        """Check if reset was requested (X button pressed)."""
        with self._lock:
            if self._reset_requested:
                self._reset_requested = False
                return True
            return False

    def close(self) -> None:
        """Stop the receiver thread."""
        self._running = False
        self._thread.join(timeout=2.0)


# ═══════════════════════════════════════════════════════════════════════════════
# One Euro Filter (smoothing) — exact copy from Adamo teleop.py
# ═══════════════════════════════════════════════════════════════════════════════

def _smoothing_factor(t_e, cutoff):
    r = 2 * np.pi * cutoff * t_e
    return r / (r + 1)


class OneEuroFilter:
    """
    One Euro Filter for smoothing 3D positions.
    Exact implementation from Adamo teleop.py.
    """

    def __init__(self, x0: np.ndarray, min_cutoff=2.25, beta=0.006, d_cutoff=1.0):
        self.min_cutoff = np.full(x0.shape, min_cutoff)
        self.beta = np.full(x0.shape, beta)
        self.d_cutoff = np.full(x0.shape, d_cutoff)
        self.x_prev = x0.astype(np.float64)
        self.dx_prev = np.zeros(x0.shape, dtype=np.float64)
        self.t_prev = None

    def __call__(self, x: np.ndarray, t: float) -> np.ndarray:
        if self.t_prev is None:
            self.t_prev = t
            self.x_prev = x.astype(np.float64)
            return x
        t_e = t - self.t_prev
        if t_e <= 0:
            return self.x_prev
        t_e = np.full(x.shape, t_e)
        a_d = _smoothing_factor(t_e, self.d_cutoff)
        dx = (x - self.x_prev) / t_e
        dx_hat = a_d * dx + (1 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * np.abs(dx_hat)
        a = _smoothing_factor(t_e, cutoff)
        x_hat = a * x + (1 - a) * self.x_prev
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t
        return x_hat


# ═══════════════════════════════════════════════════════════════════════════════
# IK Helper Functions — exact copy from Adamo teleop.py
# ═══════════════════════════════════════════════════════════════════════════════

def _quat_to_yaw(q):
    """Extract yaw (rotation around WebXR Y axis) from quaternion [w, x, y, z]."""
    w, x, y, z = q
    forward_x = -2.0 * (x * z + w * y)
    forward_z = 2.0 * (x * x + y * y) - 1.0
    return np.arctan2(forward_x, -forward_z)


def _to_calibrated_space(p_controller, calib_yaw):
    """Transform controller position from local-floor to calibrated space."""
    c, s = np.cos(calib_yaw), np.sin(calib_yaw)
    R_inv = np.array([[ c, 0,  s],
                      [ 0, 1,  0],
                      [-s, 0,  c]])
    return R_inv @ p_controller


def _quat_to_rotation_matrix(q):
    """Convert quaternion [w, x, y, z] to 3x3 rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])


def _skew(v):
    """Skew-symmetric matrix for cross product: skew(a) @ b = a x b."""
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def _ee_jacobian(J_full, ee_pos):
    """Convert spatial Jacobian to body Jacobian at EE (6xN)."""
    J_pos = J_full[:3, :] - _skew(ee_pos) @ J_full[3:, :]
    J_ang = J_full[3:, :]
    return np.vstack([J_pos, J_ang])


def _orientation_error(R_desired, R_current):
    """Compute orientation error as a 3-vector (axis-angle-like)."""
    R_err = R_desired @ R_current.T
    return 0.5 * np.array([
        R_err[2, 1] - R_err[1, 2],
        R_err[0, 2] - R_err[2, 0],
        R_err[1, 0] - R_err[0, 1],
    ])


def _vr_orientation_to_robot(vr_quat, calib_yaw):
    """Transform VR controller orientation to robot frame."""
    R_ctrl = _quat_to_rotation_matrix(vr_quat)
    c, s = np.cos(calib_yaw), np.sin(calib_yaw)
    R_calib_inv = np.array([[ c, 0,  s],
                             [ 0, 1,  0],
                             [-s, 0,  c]])
    R_hfs = R_calib_inv @ R_ctrl
    R_webxr_to_robot = np.array([[ 0, 0, -1],
                                  [-1, 0,  0],
                                  [ 0, 1,  0]], dtype=np.float64)
    return R_webxr_to_robot @ R_hfs


def _joint_limit_weights(q, joint_limits, k=JOINT_LIMIT_K):
    """Compute per-joint weights that increase near joint limits."""
    q_mid = 0.5 * (joint_limits[:, 0] + joint_limits[:, 1])
    q_half_range = 0.5 * (joint_limits[:, 1] - joint_limits[:, 0])
    q_half_range = np.maximum(q_half_range, 1e-6)
    normalized = (q - q_mid) / q_half_range
    normalized = np.clip(normalized, -1.0, 1.0)
    return 1.0 + k * normalized**4


# ═══════════════════════════════════════════════════════════════════════════════
# IK Controller — exact copy from Adamo teleop.py
# ═══════════════════════════════════════════════════════════════════════════════

class ArmIKController:
    """Damped least-squares IK controller with orientation tracking.
    Exact implementation from Adamo teleop.py.
    """

    def __init__(
        self,
        kinematics: OpenArmKinematics,
        side: str = "left",
        position_scale: float = 1.0,
        max_joint_velocity: float = 2.0,
        mirror: bool = False,
    ):
        self.kin = kinematics
        self.side = side
        self.position_scale = position_scale
        self.max_joint_velocity = max_joint_velocity
        self.mirror = mirror
        self.vr_origin: Optional[np.ndarray] = None
        self.home_joints = HOME_JOINTS_LEFT if side == "left" else HOME_JOINTS_RIGHT
        self.joint_limits = JOINT_LIMITS_LEFT if side == "left" else JOINT_LIMITS_RIGHT
        self._home_ee_pos: Optional[np.ndarray] = None
        self._home_ee_rot: Optional[np.ndarray] = None
        self._vr_orient_ref: Optional[np.ndarray] = None
        self._calib_yaw: Optional[float] = None
        self._current_joints: Optional[np.ndarray] = None
        self._debug_count = 0

    def calibrate(self, joint_pos: np.ndarray, vr_position: np.ndarray,
                  vr_orientation: Optional[np.ndarray] = None,
                  calib_yaw: Optional[float] = None):
        self._current_joints = joint_pos.copy()
        self.vr_origin = vr_position.copy()
        self._calib_yaw = calib_yaw
        ee_pose = self.kin.forward_kinematics(joint_pos)
        self._home_ee_pos = ee_pose[:3, 3].copy()
        self._home_ee_rot = ee_pose[:3, :3].copy()
        if vr_orientation is not None and calib_yaw is not None:
            self._vr_orient_ref = _vr_orientation_to_robot(vr_orientation, calib_yaw)
        else:
            self._vr_orient_ref = None
        self._debug_count = 0
        print(f"  [{self.side}] VR origin: {np.round(vr_position, 4)}")
        print(f"  [{self.side}] Calibrated joints: {np.round(joint_pos, 3)}")
        print(f"  [{self.side}] Calibrated EE pos: ({self._home_ee_pos[0]:.4f}, "
              f"{self._home_ee_pos[1]:.4f}, {self._home_ee_pos[2]:.4f})")
        print(f"  [{self.side}] Orientation tracking: "
              f"{'ON' if self._vr_orient_ref is not None else 'OFF'}")

    def compute(self, vr_position: np.ndarray, dt: float,
                vr_orientation: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        if self._home_ee_pos is None or vr_position is None:
            return None

        # -- Target pose --
        vr_delta = (vr_position - self.vr_origin) * self.position_scale
        # WebXR -> Robot: X=forward(-Z), Y=left(-X), Z=up(+Y)
        # Mirror flips left/right (Y axis in robot frame)
        mirror_sign = -1.0 if self.mirror else 1.0
        robot_delta = np.array([-vr_delta[2], mirror_sign * -vr_delta[0], vr_delta[1]])
        target_pos = self._home_ee_pos + robot_delta

        has_orient = (vr_orientation is not None and self._calib_yaw is not None
                      and self._vr_orient_ref is not None)
        R_target = None
        if has_orient:
            R_vr_now = _vr_orientation_to_robot(vr_orientation, self._calib_yaw)
            R_delta_world = R_vr_now @ self._vr_orient_ref.T
            R_target = R_delta_world @ self._home_ee_rot

        # -- Sub-iterations: multiple IK solves per control cycle --
        sub_dt = dt / IK_SUB_ITERS
        for _ in range(IK_SUB_ITERS):
            current_ee = self.kin.forward_kinematics(self._current_joints)
            current_pos = current_ee[:3, 3]
            current_rot = current_ee[:3, :3]

            J_full = self.kin.get_jacobian(self._current_joints)
            pos_error = target_pos - current_pos

            if has_orient:
                orient_error = _orientation_error(R_target, current_rot)
                error = np.concatenate([pos_error, ORIENT_WEIGHT * orient_error])
                J = _ee_jacobian(J_full, current_pos)
                J[3:, :] *= ORIENT_WEIGHT
                task_dim = 6
            else:
                error = pos_error
                J = _ee_jacobian(J_full, current_pos)[:3, :]
                task_dim = 3

            # Adaptive damping
            svs = np.linalg.svd(J, compute_uv=False)
            sigma_min = svs[-1] if len(svs) > 0 else 0.0
            if sigma_min < DLS_SIGMA_THRESH:
                lam = DLS_LAMBDA_MAX * (1.0 - (sigma_min / DLS_SIGMA_THRESH)**2)
            else:
                lam = 0.0

            # Joint-limit weighting
            w = _joint_limit_weights(self._current_joints, self.joint_limits)
            w_inv = 1.0 / w

            # Weighted DLS
            JWinv = J * w_inv[np.newaxis, :]
            JWinvJT = JWinv @ J.T
            reg = lam**2 * np.eye(task_dim) if lam > 0 else 1e-10 * np.eye(task_dim)
            A_inv = np.linalg.solve(JWinvJT + reg, np.eye(task_dim))
            J_pinv = (J.T @ A_inv) * w_inv[:, np.newaxis]
            dq_task = J_pinv @ error

            # Error-gated null-space
            err_norm = np.linalg.norm(pos_error)
            ns_scale = np.exp(-50.0 * err_norm**2)
            null_proj = np.eye(7) - J_pinv @ J
            dq_null_home = ns_scale * NULL_SPACE_GAIN * (self.home_joints - self._current_joints)

            # Manipulability gradient (disabled by default)
            dq_null_manip = np.zeros(7)

            dq = dq_task + null_proj @ (dq_null_home + MANIP_GAIN * dq_null_manip)

            # Velocity limit per sub-iteration
            max_step = self.max_joint_velocity * sub_dt
            max_abs = np.max(np.abs(dq))
            if max_abs > max_step:
                dq *= max_step / max_abs

            self._current_joints = self._current_joints + dq

            # Hard clamp
            self._current_joints = np.clip(
                self._current_joints,
                self.joint_limits[:, 0],
                self.joint_limits[:, 1],
            )

        # -- Debug logging --
        self._debug_count += 1
        if self._debug_count <= 10 or self._debug_count % 60 == 0:
            cond = svs[0] / svs[-1] if svs[-1] > 1e-10 else float('inf')
            print(f"  [{self.side}] #{self._debug_count}: "
                  f"err_pos={np.round(pos_error, 4)} "
                  f"sigma_min={sigma_min:.4f} lam={lam:.4f} "
                  f"ns={ns_scale:.3f} cond={cond:.1f} "
                  f"joints={np.round(self._current_joints, 3)}",
                  flush=True)

        return self._current_joints.copy()

    def reset(self, joint_pos: np.ndarray, vr_position: np.ndarray,
              vr_orientation: Optional[np.ndarray] = None,
              calib_yaw: Optional[float] = None):
        self.calibrate(joint_pos, vr_position, vr_orientation, calib_yaw)


# ═══════════════════════════════════════════════════════════════════════════════
# Keyboard Input (optional reset via 'R' key)
# ═══════════════════════════════════════════════════════════════════════════════

class KeyboardInput:
    """Simple keyboard input for reset functionality."""

    def __init__(self):
        self._reset = False
        self._running = True
        try:
            import sys
            if sys.stdin.isatty():
                self._thread = threading.Thread(target=self._loop, daemon=True)
                self._thread.start()
            else:
                self._thread = None
        except Exception:
            self._thread = None

    def _loop(self) -> None:
        try:
            import sys
            import tty
            import termios
            
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setcbreak(fd)
                while self._running:
                    if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                        ch = sys.stdin.read(1)
                        if ch.lower() == 'r':
                            self._reset = True
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except Exception:
            pass

    def consume_reset(self) -> bool:
        if self._reset:
            self._reset = False
            return True
        return False

    def close(self) -> None:
        self._running = False


# ═══════════════════════════════════════════════════════════════════════════════
# Helper Functions
# ═══════════════════════════════════════════════════════════════════════════════

def _quat_to_yaw(quat: np.ndarray) -> float:
    """Extract yaw from quaternion [qw, qx, qy, qz]."""
    r = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]])
    euler = r.as_euler('zyx')
    return euler[0]


def _to_calibrated_space(pos: np.ndarray, calib_yaw: float) -> np.ndarray:
    """Rotate position into calibrated coordinate frame."""
    c, s = np.cos(-calib_yaw), np.sin(-calib_yaw)
    R = np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1],
    ])
    return R @ pos


def _latest_joint_positions_for_reset(
    arm: Optional[OpenArmCAN],
    fallback: Optional[np.ndarray],
    side: str,
) -> Optional[np.ndarray]:
    """Get current joint positions from arm or use fallback (exact Adamo logic)."""
    if arm is None:
        return None

    try:
        for state in arm.states[:7]:
            state.valid = False
        arm.refresh()
        n_valid = sum(1 for state in arm.states[:7] if state.valid)
        if n_valid == 7:
            return arm.get_positions()
        if fallback is not None:
            print(f"  [{side}] Reset using last commanded joints "
                  f"({n_valid}/7 current motor states valid)", flush=True)
            return fallback.copy()
        print(f"  [{side}] Reset cannot read joints "
              f"({n_valid}/7 current motor states valid)", flush=True)
        return None
    except Exception as exc:
        if fallback is not None:
            print(f"  [{side}] Reset joint refresh failed ({exc}); "
                  "using last commanded joints", flush=True)
            return fallback.copy()
        print(f"  [{side}] Reset joint refresh failed: {exc}", flush=True)
        return None


# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    parser = argparse.ArgumentParser(
        description="OpenArm VR Teleop via UDP (No Adamo)"
    )
    parser.add_argument("--right-can", type=str, default=None,
                        help="CAN interface for right arm (e.g., can0)")
    parser.add_argument("--left-can", type=str, default=None,
                        help="CAN interface for left arm (e.g., can1)")
    parser.add_argument("--udp-host", type=str, default=DEFAULT_UDP_HOST,
                        help="UDP host to listen on")
    parser.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT,
                        help="UDP port to listen on")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="Control loop rate in Hz")
    parser.add_argument("--motor-kp", type=float, default=35.0,
                        help="Motor position gain (0-500)")
    parser.add_argument("--motor-kd", type=float, default=1.0,
                        help="Motor damping gain (0-5)")
    parser.add_argument("--position-scale", type=float, default=1.0,
                        help="Scale factor for VR position")
    parser.add_argument("--max-joint-velocity", type=float, default=2.0,
                        help="Max joint velocity in rad/s")
    parser.add_argument("--mirror", action="store_true",
                        help="Mirror left/right controls (for viewing robot from front)")
    args = parser.parse_args()

    if args.right_can is None and args.left_can is None:
        print("Error: Must specify at least one of --right-can or --left-can")
        return

    use_right = args.right_can is not None
    use_left = args.left_can is not None
    dt = 1.0 / args.rate

    print(f"[Teleop] Starting UDP VR teleop")
    print(f"  Right arm: {args.right_can}")
    print(f"  Left arm: {args.left_can}")
    print(f"  UDP: {args.udp_host}:{args.udp_port}")
    print(f"  Rate: {args.rate} Hz")

    # Initialize kinematics
    # URDF path relative to this script
    urdf_path = os.path.join(_SCRIPT_DIR, "urdf", "openarm_bimanual_abs.urdf")
    
    right_kin = OpenArmKinematics(urdf_path, arm_prefix="openarm_right_") if use_right else None
    left_kin = OpenArmKinematics(urdf_path, arm_prefix="openarm_left_") if use_left else None

    # Initialize arm controllers
    right_arm: Optional[OpenArmCAN] = None
    left_arm: Optional[OpenArmCAN] = None

    if use_right:
        right_arm = OpenArmCAN(args.right_can)
        right_arm.enable_all()
        print("[Right] Arm enabled")

    if use_left:
        left_arm = OpenArmCAN(args.left_can)
        left_arm.enable_all()
        print("[Left] Arm enabled")

    # Initialize VR input
    vr = UdpVRInput(args.udp_host, args.udp_port)
    keyboard = KeyboardInput()

    # Initial joint positions
    right_joints = _latest_joint_positions_for_reset(right_arm, None, "right") if use_right else None
    left_joints = _latest_joint_positions_for_reset(left_arm, None, "left") if use_left else None

    # IK controllers (created on calibration)
    right_ik: Optional[ArmIKController] = None
    left_ik: Optional[ArmIKController] = None
    right_euro: Optional[OneEuroFilter] = None
    left_euro: Optional[OneEuroFilter] = None

    if use_right:
        right_ik = ArmIKController(
            right_kin, side="right",
            position_scale=args.position_scale,
            max_joint_velocity=args.max_joint_velocity,
            mirror=args.mirror,
        )
    if use_left:
        left_ik = ArmIKController(
            left_kin, side="left",
            position_scale=args.position_scale,
            max_joint_velocity=args.max_joint_velocity,
            mirror=args.mirror,
        )

    calibrated = False
    calib_yaw: Optional[float] = None
    start_time = time.monotonic()
    last_debug = 0.0

    def rebuild_ik_solver(
        reason: str,
        left_vr: VRControllerState,
        right_vr: VRControllerState,
        head_quat: Optional[np.ndarray],
    ) -> bool:
        nonlocal right_ik, left_ik
        nonlocal right_joints, left_joints
        nonlocal right_euro, left_euro
        nonlocal calibrated, calib_yaw

        print(f"Reset requested ({reason}) — rebuilding IK solver", flush=True)

        right_euro = None
        left_euro = None
        calibrated = False

        if use_right:
            right_joints = _latest_joint_positions_for_reset(right_arm, right_joints, "right")
            right_ik = ArmIKController(
                right_kin, side="right",
                position_scale=args.position_scale,
                max_joint_velocity=args.max_joint_velocity,
                mirror=args.mirror,
            )
        if use_left:
            left_joints = _latest_joint_positions_for_reset(left_arm, left_joints, "left")
            left_ik = ArmIKController(
                left_kin, side="left",
                position_scale=args.position_scale,
                max_joint_velocity=args.max_joint_velocity,
                mirror=args.mirror,
            )

        got_right = (not use_right or
                     (right_joints is not None and right_vr.position is not None))
        got_left = (not use_left or
                    (left_joints is not None and left_vr.position is not None))
        if head_quat is None or not got_right or not got_left:
            print("  Reset pending: waiting for current head/controller pose", flush=True)
            return False

        calib_yaw = _quat_to_yaw(head_quat)

        if right_ik and right_vr.position is not None:
            right_cs = _to_calibrated_space(right_vr.position, calib_yaw)
            right_euro = OneEuroFilter(right_cs)
            right_ik.calibrate(right_joints, right_cs, right_vr.orientation, calib_yaw)

        if left_ik and left_vr.position is not None:
            left_cs = _to_calibrated_space(left_vr.position, calib_yaw)
            left_euro = OneEuroFilter(left_cs)
            left_ik.calibrate(left_joints, left_cs, left_vr.orientation, calib_yaw)

        calibrated = True
        print(f"  IK solver reset complete (calib yaw={np.degrees(calib_yaw):.1f}°)", flush=True)
        return True

    print("Waiting for VR data...", flush=True)

    try:
        while True:
            loop_start = time.monotonic()
            t = loop_start - start_time

            left_vr, right_vr = vr.get_state()
            head_pos, head_quat = vr.get_head_pose()
            keyboard_reset = keyboard.consume_reset()
            vr_reset = vr.consume_reset()

            if keyboard_reset or vr_reset:
                reason = "keyboard R" if keyboard_reset else "VR X"
                rebuild_ik_solver(reason, left_vr, right_vr, head_quat)

            # Debug: print VR state periodically
            if t - last_debug > 2.0:
                last_debug = t
                r_pos = right_vr.position if right_vr.position is not None else "None"
                l_pos = left_vr.position if left_vr.position is not None else "None"
                print(f"  [VR] t={t:.1f}s  L={l_pos}  R={r_pos}  "
                      f"L.upd={left_vr.updated} R.upd={right_vr.updated}", flush=True)

            if not calibrated:
                got_right = use_right and right_vr.position is not None
                got_left = use_left and left_vr.position is not None
                got_head = head_quat is not None
                need_right = use_right
                need_left = use_left
                if (got_right or not need_right) and (got_left or not need_left) and got_head:
                    calib_yaw = _quat_to_yaw(head_quat)
                    if right_ik and right_vr.position is not None:
                        right_cs = _to_calibrated_space(right_vr.position, calib_yaw)
                        right_euro = OneEuroFilter(right_cs)
                        right_ik.calibrate(right_joints, right_cs, right_vr.orientation, calib_yaw)
                    if left_ik and left_vr.position is not None:
                        left_cs = _to_calibrated_space(left_vr.position, calib_yaw)
                        left_euro = OneEuroFilter(left_cs)
                        left_ik.calibrate(left_joints, left_cs, left_vr.orientation, calib_yaw)
                    calibrated = True
                    print(f"Calibrated — VR teleop active (calib yaw={np.degrees(calib_yaw):.1f}°)", flush=True)
                else:
                    time.sleep(0.01)
                    continue

            # Right arm
            if right_ik and right_vr.updated and right_vr.position is not None:
                right_cs = _to_calibrated_space(right_vr.position, calib_yaw)
                if right_euro is not None:
                    right_cs = right_euro(right_cs, t)
                cmd = right_ik.compute(right_cs, dt, right_vr.orientation)
                if cmd is not None:
                    right_arm.set_joint_positions(
                        cmd, kp=args.motor_kp, kd=args.motor_kd,
                        process_responses=False)
                    right_joints = cmd.copy()
                right_arm._process_responses()

            # Left arm
            if left_ik and left_vr.updated and left_vr.position is not None:
                left_cs = _to_calibrated_space(left_vr.position, calib_yaw)
                if left_euro is not None:
                    left_cs = left_euro(left_cs, t)
                cmd = left_ik.compute(left_cs, dt, left_vr.orientation)
                if cmd is not None:
                    left_arm.set_joint_positions(
                        cmd, kp=args.motor_kp, kd=args.motor_kd,
                        process_responses=False)
                    left_joints = cmd.copy()
                left_arm._process_responses()

            # Grippers (trigger: 0→open, 1→closed)
            if right_arm:
                t_val = np.clip(right_vr.trigger, 0.0, 1.0)
                grip_pos = GRIPPER_OPEN + t_val * (GRIPPER_CLOSED - GRIPPER_OPEN)
                data = pack_mit(GRIPPER_MOTOR_TYPE, kp=args.motor_kp, kd=args.motor_kd, q=grip_pos)
                right_arm.bus.send(GRIPPER_SEND_ID, data)

            if left_arm:
                t_val = np.clip(left_vr.trigger, 0.0, 1.0)
                grip_pos = GRIPPER_OPEN + t_val * (GRIPPER_CLOSED - GRIPPER_OPEN)
                data = pack_mit(GRIPPER_MOTOR_TYPE, kp=args.motor_kp, kd=args.motor_kd, q=grip_pos)
                left_arm.bus.send(GRIPPER_SEND_ID, data)

            # Maintain loop rate
            elapsed = time.monotonic() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nShutting down...", flush=True)
    finally:
        keyboard.close()
        if right_arm:
            right_arm.disable_all()
            right_arm.close()
        if left_arm:
            left_arm.disable_all()
            left_arm.close()
        vr.close()
        print("Done")


if __name__ == "__main__":
    main()
