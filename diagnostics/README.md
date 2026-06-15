# OpenArm V1 Diagnostic Tests

Run these tests **on the Ireland machine** to isolate the VR teleoperation problem.

## Test Order (Divide & Conquer)

Run them in order. Stop at the first failure.

### Test 1: Motor Positions at Rest
```bash
python diagnostics/test1_motor_positions.py
```
**What it checks:** Are motor positions near zero when arms are hanging down?

**If FAIL:** Motors need recalibration (use Damiao Debugging Tool → SaveZero)

---

### Test 2: State Feedback Timing
```bash
python diagnostics/test2_state_feedback.py
```
**What it checks:** Can we read robot state quickly?

**If FAIL:** CAN communication issue

---

### Test 3: IK Command vs Actual Position (CRITICAL)
```bash
python diagnostics/test3_ik_vs_actual.py
```
**What it checks:** Does IK's initial belief match actual motor positions?

**If FAIL:** Update `_V1_ZERO_POSITION` in `ik.py` to match actual motor readings

---

### Test 4: Manual Single-Arm Movement
```bash
python diagnostics/test4_single_arm_manual.py --side right
python diagnostics/test4_single_arm_manual.py --side left
```
**What it checks:** Can we move a joint directly (bypassing VR/IK)?

**If FAIL:** Motor driver or CAN wiring issue

---

## Quick Fix Workflow

1. Run Test 1 → Get actual motor positions when arms are down
2. Run Test 3 → See if IK's belief matches
3. If mismatch: Update `_V1_ZERO_POSITION` in `nodes/dora-openarm-kinematics/src/dora_openarm_ik/ik.py`
4. Restart dataflow and test VR teleoperation

## Key Insight

The J4 (elbow) motor fault happens because:
1. IK thinks the arm is at position A
2. Robot is actually at position B  
3. First command jumps from B to A (large delta)
4. Large delta → large torque → overload fault (blinking red LED)

The fix is to make IK's initial belief match reality.
