# OpenArm V1 VR Teleoperation - Technical Documentation

## Overview

This document describes the work done to adapt VR teleoperation from OpenArm V2 to OpenArm V1, including hardware differences, software changes, current issues, and outstanding problems.

---

## 1. Hardware Differences: V1 vs V2

### 1.1 Gripper

| Feature | V1 | V2 |
|---------|----|----|
| **Type** | Linkage-driven parallel gripper | Compact gripper with in-hand camera |
| **Joint Type** | Slide joint (linear) | Hinge joint (rotational) |
| **Range** | 0.0 - 0.044 meters | -1.5708 to +1.5708 radians |
| **Control** | Position in meters | Position in radians |

### 1.2 Wrist Cameras

| Feature | V1 | V2 |
|---------|----|----|
| **Wrist cameras** | None | Built-in |

### 1.3 Motors (Damiao Series)

Both V1 and V2 use Damiao motors with MIT control mode:

| Joint | Motor Type | Notes |
|-------|------------|-------|
| J1-J2 | DM8009 | Shoulder joints |
| J3-J4 | DM4340 | Elbow joints (J4 prone to faults) |
| J5-J8 | DM4310 | Wrist and gripper |

**MIT Control Formula:**
```
torque = Kp × (position_target - position_current) + Kd × (velocity_target - velocity_current)
```

### 1.4 Joint Limits

| Joint | V1 Limits | V2 Limits |
|-------|-----------|-----------|
| J1-J7 | ±90° (±1.5708 rad) | ±90° (±1.5708 rad) |
| J2 | ±100° (±1.745 rad) | ±90° |
| J4 (Elbow) | ±90° | 0° to 140° |
| J8 (Gripper) | 0-0.044m (slide) | ±90° (hinge) |

### 1.5 CAN Interface Mapping

| Arm | CAN Interface |
|-----|---------------|
| Left | can0 |
| Right | can1 |

---

## 2. Software Changes Made

### 2.1 Gripper Mapping Function

Created V1-specific gripper mapping in `nodes/dora-openarm-kinematics/src/dora_openarm_ik/ik.py`:

```python
def _map_trigger_to_gripper_v1(trigger: float) -> float:
    """Map VR trigger (0.0-1.0) to V1 gripper position (0.0-0.044m)."""
    return trigger * 0.044
```

### 2.2 Trigger Gating (Safety Feature)

Arms only move when VR trigger is pressed >30%:

```python
trigger_threshold = 0.3  # V1: trigger must be >30% pressed to activate
trigger_active["right"] = trigger_value > trigger_threshold
```

This prevents unintended robot movement when controllers are idle.

### 2.3 Coordinate Transformation

Updated `_FRAME_ROT_V1` in `nodes/dora-openarm-vr/src/node/dora_openarm_quest_receiver/main.py` for "user behind robot" perspective:

```python
# V1 frame rotation - "Same direction" mapping for user behind robot
_FRAME_ROT_V1 = np.array([
    [  0.,   0.,  -1.],  # robot X (forward) = -VR Z
    [ -1.,   0.,   0.],  # robot Y (left)    = -VR X
    [  0.,   1.,   0.],  # robot Z (up)      = +VR Y
], dtype=np.float64)
```

### 2.4 Joint Configuration Files

Created V1-specific configs in `config/`:
- `openarm_v1_left.yaml`
- `openarm_v1_right.yaml`

Key settings:
```yaml
joint_limits:
  right_arm:
    - [-1.5708, 1.5708]  # J1-J7: ±90°
    - [0.0, 0.044]       # J8: gripper slide joint (meters)

control_gains:
  kps: [70.0, 70.0, 70.0, 15.0, 10.0, 10.0, 10.0, 10.0]  # J4 reduced to 15
  kds: [2.75, 2.5, 2.0, 1.0, 0.7, 0.6, 0.5, 0.2]         # J4 reduced to 1.0
```

### 2.5 IK Solver Parameters

Adjusted IK parameters in `dataflow-vr-real-v1.yaml` to prevent sudden jumps:

| Parameter | Original | Changed To | Reason |
|-----------|----------|------------|--------|
| `--dt` | 0.1 | 0.02 | 5x smaller position steps |
| `--damping` | 0.1 | 0.2 | More conservative solver |

### 2.6 Position Smoothing

Added rate-limiting in IK output to prevent motor faults:

```python
_MAX_DELTA_PER_STEP = np.array([
    0.15,  # J1 - shoulder
    0.15,  # J2 - shoulder  
    0.15,  # J3 - elbow
    0.08,  # J4 - elbow (most sensitive)
    0.2,   # J5-J7 - wrist
    0.01,  # J8 - gripper
])
```

### 2.7 Environment Variables

Moved hardcoded UDP relay config to environment variables:

```bash
UDP_RELAY_LOCAL_HOST=0.0.0.0
UDP_RELAY_LOCAL_PORT=5006
UDP_RELAY_REMOTE_HOST=<Ireland_Tailscale_IP>
UDP_RELAY_REMOTE_PORT=5006
```

---

## 3. What We Achieved

| Feature | Status |
|---------|--------|
| VR pose reception via UDP | ✅ Working |
| Coordinate transformation (LH→RH) | ✅ Working |
| Trigger gating (>30% to activate) | ✅ Working |
| Simultaneous bimanual control | ✅ Working |
| IK solver with V1 joint limits | ✅ Working |
| V1 gripper mapping (slide joint) | ✅ Implemented |
| Zero position initialization | ✅ Working |
| Position smoothing | ✅ Implemented |
| Kenya→Ireland UDP relay | ✅ Working |
| MuJoCo simulation dataflow | ✅ Working |
| Real robot dataflow | ⚠️ Partially working |

---

## 4. Current Issues (Unresolved)

### 4.1 J4 (Elbow) Motor Fault - CRITICAL

**Symptom:** J4 motor LED blinks red after a few seconds of VR teleoperation.

**Damiao Fault Code:** `E` = Overload (motor trying to apply too much torque)

**Observations:**
- J4 **never faults** during manual leader-follower teleoperation
- J4 **always faults** during VR teleoperation
- Fault occurs within seconds of starting VR control

**Root Cause Analysis:**
- Manual teleoperation: Leader arm movements are naturally smooth, position errors are small
- VR teleoperation: IK solver can output positions far from current position
- Large position error × high Kp = excessive torque demand → overload fault

**Attempted Fixes:**
| Fix | Result |
|-----|--------|
| Reduced J4 Kp from 60 to 15 | Still faults |
| Reduced J4 Kd from 2.0 to 1.0 | Still faults |
| Reduced IK dt from 0.1 to 0.02 | Still faults |
| Added position rate-limiting (0.08 rad/step for J4) | Still faults |
| Tightened J4 joint limits | Reverted - limited range of motion |

**Hypothesis:**
The IK solver may be commanding positions that require the elbow to move through configurations that demand high torque, even with smoothing. The fundamental issue may be in how the IK solver plans trajectories.

**Next Steps to Try:**
1. Add torque limiting in motor driver
2. Implement trajectory interpolation instead of direct position commands
3. Investigate IK null-space optimization to avoid high-torque configurations
4. Compare IK output between VR and manual teleoperation to identify differences

### 4.2 Gripper Unresponsiveness

**Symptom:** Gripper does not open/close in response to VR trigger.

**Status:** Partially investigated

**Possible Causes:**
1. Gripper joint limits may still be incorrect
2. Trigger value may not be reaching gripper mapping function
3. Motor driver may not be receiving gripper commands

**Debug Steps Needed:**
1. Add logging to verify trigger values are received
2. Verify gripper position is included in IK output array
3. Check motor driver is sending gripper commands to CAN

---

## 5. Architecture Summary

```
Quest VR (WiFi) 
    ↓ UDP packets (pose, trigger, buttons)
Kenya PC - udp_relay.py
    ↓ Tailscale VPN
Ireland PC - Dora Dataflow
    ├── udp-receiver: Parse VR data, coordinate transform
    ├── ik: Differential IK (MuJoCo) → joint angles
    ├── follower-left: Motor driver (can0)
    └── follower-right: Motor driver (can1)
```

---

## 6. Files Modified

| File | Changes |
|------|---------|
| `nodes/dora-openarm-kinematics/src/dora_openarm_ik/ik.py` | V1 gripper mapping, trigger gating, position smoothing |
| `nodes/dora-openarm-vr/src/node/dora_openarm_quest_receiver/main.py` | V1 coordinate transform |
| `config/openarm_v1_left.yaml` | V1 joint limits, reduced J4 gains |
| `config/openarm_v1_right.yaml` | V1 joint limits, reduced J4 gains |
| `dataflow-vr-real-v1.yaml` | V1 real robot dataflow |
| `dataflow-vr-mujoco-v1.yaml` | V1 simulation dataflow |
| `models/v1/openarm_bimanual.xml` | MuJoCo model with V1 joint limits |
| `udp_relay.py` | Environment variable config |

---

## 7. Recommendations for Next Developer

1. **J4 Fault Investigation:**
   - Add detailed logging of IK output vs actual motor position
   - Compare trajectories between manual and VR teleoperation
   - Consider implementing velocity-based control instead of position-based

2. **Gripper Debugging:**
   - Add logging at each stage: trigger input → gripper mapping → IK output → motor command
   - Verify CAN messages are being sent for gripper motor

3. **General Improvements:**
   - Implement proper trajectory interpolation
   - Add motor fault detection and auto-recovery
   - Consider adding torque feedback for compliance control

---

## 8. Contact

For questions about this documentation or the codebase, refer to the commit history in the melvine-git/openarm_teleoperation repository.

---

*Last updated: June 2026*
