#!/usr/bin/env python3
"""
TEST 3: Compare IK initial command vs actual robot position
===========================================================
This is the CRITICAL test. It shows what the IK would command vs
where the robot actually is.

If the delta is large (>0.5 rad on any joint), that's the cause of motor faults.

Usage:
    python diagnostics/test3_ik_vs_actual.py
"""

import numpy as np

# IK's _V1_ZERO_POSITION from ik.py - what IK believes is "arms down"
IK_ZERO_POSITION = {
    "right": [0.0, 0.506145, -1.570796, 1.745329, 0.0, -0.331612, 1.570796, 0.0],
    "left":  [0.0, -0.506145, 1.570796, 1.745329, 0.0, 0.331612, -1.570796, 0.0],
}

# Joint offsets from config files
JOINT_OFFSETS = {
    "right": [0.0, -0.506145, 1.570796, -1.745329, 0.0, 0.331612, -1.570796, 0.0],
    "left":  [0.0, 0.506145, -1.570796, -1.745329, 0.0, -0.331612, 1.570796, 0.0],
}


def main():
    print("="*60)
    print("TEST 3: IK Command vs Actual Robot Position")
    print("="*60)
    
    try:
        import openarm_driver
    except ImportError:
        print("ERROR: openarm_driver not installed")
        return
    
    print("\nFetching actual robot positions...")
    
    for side in ["right", "left"]:
        print(f"\n{'='*60}")
        print(f"{side.upper()} ARM ANALYSIS")
        print(f"{'='*60}")
        
        config_path = f"config/openarm_v1_{side}.yaml"
        try:
            config = openarm_driver.Config(config_path)
            arm = openarm_driver.SingleArmDriver(f"{side}_arm", config)
            arm.start()
            actual_pos = np.array(arm.fetch_position(), dtype=np.float32)
            arm.stop()
        except Exception as e:
            print(f"ERROR reading {side} arm: {e}")
            continue
        
        ik_initial = np.array(IK_ZERO_POSITION[side], dtype=np.float32)
        offsets = np.array(JOINT_OFFSETS[side], dtype=np.float32)
        
        print(f"\n1. Actual motor position (from driver):")
        print(f"   {actual_pos[:4]}...")
        
        print(f"\n2. IK's initial belief (_V1_ZERO_POSITION):")
        print(f"   {ik_initial[:4]}...")
        
        print(f"\n3. Joint offsets (from config):")
        print(f"   {offsets[:4]}...")
        
        # The IK outputs positions in MODEL space
        # The driver receives positions and sends them to motors
        # Question: Does the driver apply offsets?
        
        # If IK outputs ik_initial, what does the motor receive?
        # Scenario A: Driver applies NO offset → motor receives ik_initial
        # Scenario B: Driver applies offset → motor receives ik_initial + offset
        
        delta_no_offset = ik_initial - actual_pos
        delta_with_offset = (ik_initial + offsets) - actual_pos
        
        print(f"\n4. DELTA if driver applies NO offset (IK output → motor directly):")
        print(f"   Delta = IK_belief - actual = {delta_no_offset[:4]}...")
        print(f"   Max delta: {np.max(np.abs(delta_no_offset)):.3f} rad ({np.rad2deg(np.max(np.abs(delta_no_offset))):.1f}°)")
        
        print(f"\n5. DELTA if driver ADDS offset (IK_output + offset → motor):")
        print(f"   Delta = (IK_belief + offset) - actual = {delta_with_offset[:4]}...")
        print(f"   Max delta: {np.max(np.abs(delta_with_offset)):.3f} rad ({np.rad2deg(np.max(np.abs(delta_with_offset))):.1f}°)")
        
        # Determine which scenario is safer
        max_no = np.max(np.abs(delta_no_offset))
        max_with = np.max(np.abs(delta_with_offset))
        
        print(f"\n6. DIAGNOSIS:")
        if max_no < 0.3:
            print(f"   ✓ Scenario A (no offset in driver) has small delta ({max_no:.2f} rad)")
            print(f"     → IK's _V1_ZERO_POSITION matches motor reality")
        elif max_with < 0.3:
            print(f"   ✓ Scenario B (offset in driver) has small delta ({max_with:.2f} rad)")
            print(f"     → Driver should be adding offsets (or IK needs adjustment)")
        else:
            print(f"   ✗ BOTH scenarios have large deltas!")
            print(f"     → Motor calibration doesn't match ANY expected convention")
            print(f"     → Need to recalibrate motors OR update _V1_ZERO_POSITION")
    
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    print("If deltas are large (>0.3 rad / 17°), the first IK command will")
    print("cause a huge position jump → torque spike → motor overload fault.")
    print("\nFIX: Update _V1_ZERO_POSITION in ik.py to match actual motor positions")
    print("     when arms are physically hanging down.")


if __name__ == "__main__":
    main()
