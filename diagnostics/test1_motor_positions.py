#!/usr/bin/env python3
"""
TEST 1: Verify motor positions at rest (arms down)
==================================================
Run this on Ireland with the robot arms physically hanging down.

Expected: All joint positions should be near zero (within ±0.1 rad)
If NOT near zero: Motors were calibrated at a different pose, need recalibration
                  OR the offset convention is wrong

Usage:
    python diagnostics/test1_motor_positions.py
"""

import sys
import numpy as np

try:
    import openarm_driver
except ImportError:
    print("ERROR: openarm_driver not installed. Run: pip install openarm-driver")
    sys.exit(1)


def test_arm(side: str, config_path: str):
    """Test a single arm's motor positions."""
    print(f"\n{'='*60}")
    print(f"Testing {side.upper()} arm")
    print(f"Config: {config_path}")
    print(f"{'='*60}")
    
    try:
        config = openarm_driver.Config(config_path)
        arm = openarm_driver.SingleArmDriver(f"{side}_arm", config)
        arm.start()
        
        # Fetch current position
        position = np.array(arm.fetch_position(), dtype=np.float32)
        
        print(f"\nRaw motor positions (radians):")
        for i, pos in enumerate(position):
            deg = np.rad2deg(pos)
            status = "✓ OK" if abs(pos) < 0.2 else "⚠ OFF"
            print(f"  J{i+1}: {pos:+.4f} rad ({deg:+.1f}°) {status}")
        
        print(f"\nMax deviation from zero: {np.max(np.abs(position)):.4f} rad")
        
        arm.stop()
        return position
        
    except Exception as e:
        print(f"ERROR: {e}")
        return None


def main():
    print("="*60)
    print("TEST 1: Motor Position Verification")
    print("="*60)
    print("\nINSTRUCTIONS:")
    print("1. Make sure robot arms are physically hanging straight down")
    print("2. Motors should be enabled but not receiving commands")
    print("3. No other dataflow should be running")
    print("\nPress Enter to continue...")
    input()
    
    # Test both arms
    right_pos = test_arm("right", "config/openarm_v1_right.yaml")
    left_pos = test_arm("left", "config/openarm_v1_left.yaml")
    
    print("\n" + "="*60)
    print("DIAGNOSIS")
    print("="*60)
    
    if right_pos is not None and left_pos is not None:
        max_right = np.max(np.abs(right_pos))
        max_left = np.max(np.abs(left_pos))
        
        if max_right < 0.2 and max_left < 0.2:
            print("✓ PASS: Motor positions are near zero when arms are down.")
            print("  Motors are calibrated correctly.")
        else:
            print("✗ FAIL: Motor positions are NOT near zero!")
            print("\n  This means either:")
            print("  a) Motors were calibrated at a different pose (need recalibration)")
            print("  b) Arms are not actually hanging straight down")
            print("\n  To fix: Use Damiao Debugging Tool and click 'SaveZero'")
            print("          with arms in the down position for each motor.")


if __name__ == "__main__":
    main()
