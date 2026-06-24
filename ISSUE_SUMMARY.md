# VR Teleoperation Issue: Left Arm J4 Motor Fault

## Problem Summary

When running VR teleoperation on the **OpenArm V1 bimanual robot**, the **left arm's J4 (elbow) motor faults** (LED blinks red) within seconds of pressing the VR trigger. The right arm appears to work correctly.

**Fault type:** Damiao motor overload fault (code "E")

---

## Architecture Overview

```
Kenya (VR Operator)              Ireland (Robot)
┌──────────────┐                 ┌──────────────────────────────────────┐
│ Quest VR     │                 │ dora-rs Dataflow                     │
│ Headset      │──UDP──┐         │                                      │
└──────────────┘       │         │  udp-receiver → IK → follower-right  │
                       │         │                    → follower-left   │
┌──────────────┐       │         │                                      │
│ udp_relay.py │───────┼─────────│  (state feedback loops back to IK)   │
│ (forwards    │  Tailscale VPN  │                                      │
│  UDP packets)│                 └──────────────────────────────────────┘
└──────────────┘                              │
                                              ▼
                                    ┌──────────────────┐
                                    │ OpenArm V1 Robot │
                                    │ (CAN bus motors) │
                                    └──────────────────┘
```

---

## Suspected Root Cause

**IK initial position mismatch with actual motor positions.**

The IK solver initializes with a hardcoded "zero position" (`_V1_ZERO_POSITION` in `ik.py`), which represents where IK believes the robot arms are when physically hanging down. If this doesn't match the **actual motor encoder positions**, then:

1. When VR trigger is pressed, IK syncs to what it *thinks* is the current position
2. IK computes a target position
3. The commanded position has a large delta from actual motor position
4. Large delta → large torque demand → **motor overload fault**

### Key Code Location

```python
# nodes/dora-openarm-kinematics/src/dora_openarm_ik/ik.py, lines 73-76
_V1_ZERO_POSITION = {
    "right": [0.0, 0.506145, -1.570796, 1.745329, 0.0, -0.331612, 1.570796, 0.0],
    "left":  [0.0, -0.506145, 1.570796, 1.745329, 0.0, 0.331612, -1.570796, 0.0],
}
```

These values assume motors were calibrated with "arms hanging straight down = motor position 0". If the Damiao motors were calibrated at a different pose, these values are wrong.

---

## What We've Verified

| Check | Status |
|-------|--------|
| UDP relay forwarding packets Kenya → Ireland | ✅ Working (13k+ packets) |
| VR data reaching Ireland | ✅ Working |
| CAN interface config (swapped left/right) | ✅ Fixed |
| Right arm movement | ✅ Working |
| Left arm J4 faulting | ❌ **PROBLEM** |

---

## Diagnostic Output Added

I added inline diagnostics to the IK node. When the pipeline runs, it now outputs:

```
======================================================================
[ik] DIAGNOSTIC: FIRST STATE RECEIVED - LEFT ARM
======================================================================
[ik] ACTUAL motor pos:   [+X.XXXX, +X.XXXX, +X.XXXX, +X.XXXX, ...]
[ik] EXPECTED (IK init): [+0.0000, -0.5061, +1.5708, +1.7453, ...]
[ik] DELTA:              [+X.XXXX, +X.XXXX, +X.XXXX, +X.XXXX, ...]
[ik] MAX DELTA: X.XXXX rad (XX.X deg)
[ik] *** WARNING: Large mismatch! This will cause motor faults! ***
======================================================================
```

**If MAX DELTA > 0.3 rad (~17°), motor faults are expected.**

---

## Questions for Senior Review

1. **Motor calibration:** Were the Damiao motors calibrated with arms in "hanging down" position? Or a different pose?

2. **Offset convention:** The config files have `joint_offsets`. Should these be applied by the driver before sending to motors? Currently `dora-openarm/main.py` sends IK output directly to `arm.send_position()` without applying offsets.

3. **Left vs Right difference:** Why does only the left arm fault? Could there be a sign convention difference in the offsets?

4. **CAN interface mapping:** Config files now say `right_arm: can1, left_arm: can0`. Does this match the physical wiring?

---

## Files to Review

| File | Purpose |
|------|---------|
| `nodes/dora-openarm-kinematics/src/dora_openarm_ik/ik.py` | IK solver, `_V1_ZERO_POSITION`, sync logic |
| `nodes/dora-openarm/src/dora_openarm/main.py` | Motor driver, sends positions to robot |
| `config/openarm_v1_left.yaml` | Left arm config, joint_offsets, CAN mapping |
| `config/openarm_v1_right.yaml` | Right arm config |
| `dataflow-vr-real-v1.yaml` | Dora dataflow definition |

---

## To Reproduce

1. On Kenya PC: `python udp_relay.py`
2. On Ireland PC: `git pull melvine main && dora run dataflow-vr-real-v1.yaml --uv`
3. Press VR trigger > 30%
4. Observe left arm J4 motor LED blinks red (fault)

---

## Proposed Fix

Once we get the ACTUAL motor positions from the diagnostic output, update `_V1_ZERO_POSITION` in `ik.py` to match the real motor encoder readings when arms are physically down.
