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

GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 1.5

# Joint limits (radians) — same as original teleop.py
JOINT_LIMITS_LOWER = np.array([-2.8, -1.8, -2.8, -1.8, -2.8, -1.8, -2.8])
JOINT_LIMITS_UPPER = np.array([2.8, 1.8, 2.8, 1.8, 2.8, 1.8, 2.8])

# Homing position (radians)
HOME_POSITION = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


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
# One Euro Filter (smoothing)
# ═══════════════════════════════════════════════════════════════════════════════

class OneEuroFilter:
    """
    One Euro Filter for smoothing 3D positions.
    
    Provides adaptive low-pass filtering that reduces jitter while
    maintaining responsiveness during fast movements.
    """

    def __init__(
        self,
        x0: np.ndarray,
        min_cutoff: float = 1.0,
        beta: float = 0.007,
        d_cutoff: float = 1.0,
    ):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = x0.copy()
        self.dx_prev = np.zeros_like(x0)
        self.t_prev: Optional[float] = None

    def __call__(self, x: np.ndarray, t: float) -> np.ndarray:
        if self.t_prev is None:
            self.t_prev = t
            return x

        dt = t - self.t_prev
        if dt <= 0:
            return self.x_prev

        self.t_prev = t

        # Compute derivative
        dx = (x - self.x_prev) / dt

        # Filter derivative
        alpha_d = self._alpha(self.d_cutoff, dt)
        dx_hat = alpha_d * dx + (1 - alpha_d) * self.dx_prev

        # Adaptive cutoff based on derivative magnitude
        cutoff = self.min_cutoff + self.beta * np.linalg.norm(dx_hat)

        # Filter position
        alpha = self._alpha(cutoff, dt)
        x_hat = alpha * x + (1 - alpha) * self.x_prev

        self.x_prev = x_hat
        self.dx_prev = dx_hat
        return x_hat

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2 * np.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)


# ═══════════════════════════════════════════════════════════════════════════════
# IK Controller (DLS with joint limit avoidance)
# ═══════════════════════════════════════════════════════════════════════════════

class ArmIKController:
    """
    Damped Least Squares IK controller with adaptive damping and joint limits.
    
    This is the same IK approach as the original Adamo teleop.py.
    """

    def __init__(
        self,
        kinematics: OpenArmKinematics,
        side: str = "right",
        position_scale: float = 1.0,
        max_joint_velocity: float = 2.0,
        damping: float = 0.05,
    ):
        self.kin = kinematics
        self.side = side
        self.position_scale = position_scale
        self.max_joint_velocity = max_joint_velocity
        self.damping = damping

        self.q: Optional[np.ndarray] = None
        self.origin_pos: Optional[np.ndarray] = None
        self.origin_rot: Optional[Rotation] = None
        self.calib_yaw: float = 0.0
        self.calibrated = False

    def calibrate(
        self,
        current_joints: np.ndarray,
        vr_position: np.ndarray,
        vr_orientation: np.ndarray,
        calib_yaw: float,
    ) -> None:
        """
        Calibrate IK to current robot state and VR pose.
        
        This captures the VR origin so movements are relative.
        """
        self.q = current_joints.copy()
        self.origin_pos = vr_position.copy()
        self.origin_rot = Rotation.from_quat([
            vr_orientation[1],  # qx
            vr_orientation[2],  # qy
            vr_orientation[3],  # qz
            vr_orientation[0],  # qw
        ])
        self.calib_yaw = calib_yaw
        self.calibrated = True
        print(f"[IK:{self.side}] Calibrated at joints={np.degrees(current_joints)}")

    def compute(
        self,
        vr_position: np.ndarray,
        dt: float,
        vr_orientation: np.ndarray,
    ) -> Optional[np.ndarray]:
        """
        Compute joint command given VR pose.
        
        Returns joint positions or None if not calibrated.
        """
        if not self.calibrated or self.q is None:
            return None

        # Relative position from calibration origin
        delta_pos = (vr_position - self.origin_pos) * self.position_scale

        # Get current end-effector pose
        T_current = self.kin.forward_kinematics(self.q)
        p_current = T_current[:3, 3]

        # Target position
        p_target = p_current + delta_pos
        self.origin_pos = vr_position.copy()  # Update origin for next frame

        # Target orientation from VR
        vr_rot = Rotation.from_quat([
            vr_orientation[1],
            vr_orientation[2],
            vr_orientation[3],
            vr_orientation[0],
        ])
        rel_rot = self.origin_rot.inv() * vr_rot
        self.origin_rot = vr_rot

        R_current = T_current[:3, :3]
        R_target = R_current @ rel_rot.as_matrix()

        # Build target transform
        T_target = np.eye(4)
        T_target[:3, :3] = R_target
        T_target[:3, 3] = p_target

        # Solve IK
        q_new = self._solve_ik(T_target, dt)
        if q_new is not None:
            self.q = q_new

        return self.q.copy()

    def _solve_ik(self, T_target: np.ndarray, dt: float) -> Optional[np.ndarray]:
        """Damped least squares IK solver."""
        q = self.q.copy()

        for _ in range(5):  # Iterations
            T_current = self.kin.forward_kinematics(q)

            # Position error
            p_err = T_target[:3, 3] - T_current[:3, 3]

            # Orientation error (axis-angle)
            R_err = T_target[:3, :3] @ T_current[:3, :3].T
            r = Rotation.from_matrix(R_err)
            rotvec = r.as_rotvec()

            # Task-space error
            err = np.concatenate([p_err, rotvec])
            if np.linalg.norm(err) < 1e-4:
                break

            # Jacobian
            J = self.kin.jacobian(q)

            # Damped least squares
            JJT = J @ J.T
            damping_matrix = (self.damping ** 2) * np.eye(6)
            dq = J.T @ np.linalg.solve(JJT + damping_matrix, err)

            # Joint limit avoidance (gradient)
            margin = 0.1
            grad = np.zeros(7)
            for i in range(7):
                if q[i] < JOINT_LIMITS_LOWER[i] + margin:
                    grad[i] = JOINT_LIMITS_LOWER[i] + margin - q[i]
                elif q[i] > JOINT_LIMITS_UPPER[i] - margin:
                    grad[i] = JOINT_LIMITS_UPPER[i] - margin - q[i]

            # Null-space projection
            J_pinv = J.T @ np.linalg.inv(JJT + damping_matrix)
            null_proj = np.eye(7) - J_pinv @ J
            dq += 0.5 * null_proj @ grad

            # Velocity limiting
            dq_max = self.max_joint_velocity * dt
            dq = np.clip(dq, -dq_max, dq_max)

            q = q + dq

            # Clamp to joint limits
            q = np.clip(q, JOINT_LIMITS_LOWER, JOINT_LIMITS_UPPER)

        return q


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
    """Get current joint positions from arm or use fallback."""
    if arm is None:
        return fallback
    try:
        positions = arm.get_positions()
        if positions is not None:
            print(f"[{side}] Got current joints: {np.degrees(positions)}")
            return positions
    except Exception as e:
        print(f"[{side}] Failed to read joints: {e}")
    return fallback if fallback is not None else HOME_POSITION.copy()


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
    parser.add_argument("--motor-kp", type=float, default=10.0,
                        help="Motor position gain")
    parser.add_argument("--motor-kd", type=float, default=1.0,
                        help="Motor velocity gain")
    parser.add_argument("--position-scale", type=float, default=1.0,
                        help="Scale factor for VR position")
    parser.add_argument("--max-joint-velocity", type=float, default=2.0,
                        help="Max joint velocity in rad/s")
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
        )
    if use_left:
        left_ik = ArmIKController(
            left_kin, side="left",
            position_scale=args.position_scale,
            max_joint_velocity=args.max_joint_velocity,
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
            )
        if use_left:
            left_joints = _latest_joint_positions_for_reset(left_arm, left_joints, "left")
            left_ik = ArmIKController(
                left_kin, side="left",
                position_scale=args.position_scale,
                max_joint_velocity=args.max_joint_velocity,
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
