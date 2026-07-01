#!/usr/bin/env python3
"""
Parse gripper calibration test output and extract the optimal GRIPPER_HOLD value.
"""

import sys
import re

def parse_calibration_output(log_text):
    """Extract calibration results from test output."""
    
    # Look for the results table
    results = []
    lines = log_text.split('\n')
    
    in_table = False
    for line in lines:
        # Detect table start
        if 'Grip Value (rad)' in line:
            in_table = True
            continue
        
        # Detect table end
        if in_table and '=' in line and 'UPDATE' in line:
            break
        
        # Parse table rows
        if in_table and '✓' in line:
            # Extract grip value, angle, z, and status
            match = re.search(r'(-?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+(0\.\d+)\s+✓', line)
            if match:
                grip_val = float(match.group(1))
                angle = float(match.group(2))
                z = float(match.group(3))
                results.append({
                    'grip': grip_val,
                    'angle': angle,
                    'z': z,
                    'success': True
                })
        elif in_table and '✗' in line:
            match = re.search(r'(-?\d+\.\d+)\s+([-+]?\d+\.\d+)\s+(0\.\d+)\s+✗', line)
            if match:
                grip_val = float(match.group(1))
                angle = float(match.group(2))
                z = float(match.group(3))
                results.append({
                    'grip': grip_val,
                    'angle': angle,
                    'z': z,
                    'success': False
                })
    
    # Look for the optimal value in the output
    optimal = None
    optimal_match = re.search(r'Optimal GRIPPER_HOLD = (-?\d+\.\d+)', log_text)
    if optimal_match:
        optimal = float(optimal_match.group(1))
    
    return results, optimal

if __name__ == '__main__':
    if len(sys.argv) > 1:
        with open(sys.argv[1], 'r') as f:
            log_text = f.read()
    else:
        log_text = sys.stdin.read()
    
    results, optimal = parse_calibration_output(log_text)
    
    if optimal:
        print(f"\n✓ CALIBRATION FOUND OPTIMAL VALUE:")
        print(f"  GRIPPER_HOLD = {optimal:.2f} rad")
        print(f"\nUpdate vision_autopilot_simple.py line ~183:")
        print(f"  GRIPPER_HOLD = {optimal:.2f}")
    else:
        print("\n✗ Could not find optimal value in output")
        print(f"  Found {len(results)} test results")
        if results:
            successful = [r for r in results if r['success']]
            print(f"  {len(successful)} successful pickups")
            if successful:
                best = max(successful, key=lambda x: x['grip'])
                print(f"\n  Best grip value: {best['grip']:.3f} rad (z={best['z']:.5f}m)")
