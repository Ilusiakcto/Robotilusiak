#!/usr/bin/env python3
"""
TEST 4: Manual single-arm position test (NO VR, NO IK)
======================================================
This bypasses the entire VR/IK pipeline and sends a simple position
command directly to ONE arm. Use this to verify the motor driver works.

SAFETY: Only moves J6 (wrist) by a small amount - lowest risk joint.

Usage:
    python diagnostics/test4_single_arm_manual.py --side right
    python diagnostics/test4_single_arm_manual.py --side left
"""

import argparse
import time
import numpy as np

try:
    import openarm_driver
except ImportError:
    print("ERROR: openarm_driver not installed")
    exit(1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=["right", "left"], default="right")
    args = parser.parse_args()
    
    print("="*60)
    print(f"TEST 4: Manual Single Arm Test ({args.side.upper()})")
    print("="*60)
    print("\nThis test sends a small position command to J6 (wrist roll).")
    print("It bypasses VR and IK completely to verify motor driver works.")
    print("\n⚠️  SAFETY: Keep clear of the robot arm!")
    print("\nPress Enter to continue (Ctrl+C to abort)...")
    input()
    
    config_path = f"config/openarm_v1_{args.side}.yaml"
    config = openarm_driver.Config(config_path)
    arm = openarm_driver.SingleArmDriver(f"{args.side}_arm", config)
    arm.start()
    
    # Get current position
    current = np.array(arm.fetch_position(), dtype=np.float32)
    print(f"\nCurrent position: {current}")
    
    # Create target: same as current, but move J6 by 0.2 rad (~11°)
    target = current.copy()
    target[5] += 0.2  # J6 is index 5 (0-indexed)
    
    print(f"\nTarget position:  {target}")
    print(f"Delta (J6 only):  {target[5] - current[5]:.3f} rad ({np.rad2deg(target[5] - current[5]):.1f}°)")
    
    print("\nSending position command...")
    
    # Ramp to target over 1 second
    steps = 50
    for i in range(steps):
        alpha = (i + 1) / steps
        interp = current + alpha * (target - current)
        arm.send_position(interp)
        time.sleep(0.02)
    
    time.sleep(0.5)
    
    # Read final position
    final = np.array(arm.fetch_position(), dtype=np.float32)
    print(f"\nFinal position:   {final}")
    
    # Move back
    print("\nMoving back to original position...")
    for i in range(steps):
        alpha = (i + 1) / steps
        interp = target + alpha * (current - target)
        arm.send_position(interp)
        time.sleep(0.02)
    
    arm.stop()
    
    print("\n" + "="*60)
    print("DIAGNOSIS")
    print("="*60)
    
    delta = abs(final[5] - target[5])
    if delta < 0.05:
        print("✓ PASS: Motor responded correctly to position command.")
        print("  The driver and CAN communication are working.")
    else:
        print(f"✗ FAIL: Motor did not reach target (delta: {delta:.3f} rad)")
        print("  Check: CAN wiring, motor enable status, motor faults")


if __name__ == "__main__":
    main()
