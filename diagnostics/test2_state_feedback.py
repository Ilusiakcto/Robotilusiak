#!/usr/bin/env python3
"""
TEST 2: Verify state feedback arrives before trigger press
==========================================================
This runs a minimal dataflow to check if the IK receives robot state
BEFORE you press the VR trigger.

Expected: State feedback should arrive within 100ms of startup
If NOT: The circular dependency issue still exists

Usage:
    python diagnostics/test2_state_feedback.py
"""

import time
import numpy as np

try:
    import openarm_driver
except ImportError:
    print("ERROR: openarm_driver not installed")
    exit(1)


def test_state_timing(side: str, config_path: str):
    """Test how quickly state feedback is available."""
    print(f"\n{'='*60}")
    print(f"Testing {side.upper()} arm state feedback timing")
    print(f"{'='*60}")
    
    config = openarm_driver.Config(config_path)
    arm = openarm_driver.SingleArmDriver(f"{side}_arm", config)
    
    start_time = time.time()
    arm.start()
    init_time = time.time() - start_time
    print(f"Arm initialized in {init_time*1000:.1f}ms")
    
    # Fetch state multiple times
    for i in range(5):
        t0 = time.time()
        state = arm.fetch_state(refresh=True)
        dt = (time.time() - t0) * 1000
        
        qpos = state['qpos']
        print(f"  State #{i+1}: fetched in {dt:.1f}ms, J1-4=[{qpos[0]:.3f}, {qpos[1]:.3f}, {qpos[2]:.3f}, {qpos[3]:.3f}]")
        time.sleep(0.05)
    
    arm.stop()
    return True


def main():
    print("="*60)
    print("TEST 2: State Feedback Timing")
    print("="*60)
    print("\nThis test verifies that robot state is available immediately.")
    print("In the real dataflow, IK needs state BEFORE the trigger is pressed.")
    print("\nPress Enter to continue...")
    input()
    
    test_state_timing("right", "config/openarm_v1_right.yaml")
    test_state_timing("left", "config/openarm_v1_left.yaml")
    
    print("\n" + "="*60)
    print("DIAGNOSIS")
    print("="*60)
    print("✓ If state was fetched quickly (<50ms), the driver is working.")
    print("  The dataflow uses a 50ms timer to request state, so feedback")
    print("  should be available within 100ms of dataflow startup.")
    print("\n  IMPORTANT: Wait 1-2 seconds after starting the dataflow")
    print("  before pressing the VR trigger to ensure state is available.")


if __name__ == "__main__":
    main()
