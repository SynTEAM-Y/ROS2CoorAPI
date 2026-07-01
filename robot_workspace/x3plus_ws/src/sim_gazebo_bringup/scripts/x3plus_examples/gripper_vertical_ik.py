#!/usr/bin/env python3
"""
Compute REACH_DOWN arm pose that makes the gripper pads vertical (pointing
straight down in the Z direction) and the fingertip centred on the cube.

Arm convention (from URDF xacro):
  - arm_joint1: rotation around base Z (ignored for 2D XZ plane)
  - arm_joint2: shoulder, axis=-Z in xacro frame
  - arm_joint3: elbow, axis=-Z
  - arm_joint4: wrist, axis=-Z
  - arm_joint5: wrist roll, axis=+Z, rpy=(pi/2,0,0)

For a vertically-down gripper, the gripper's opening direction is horizontal
(X/Y) and j5 can be left at 0. The key constraint is that the last link points
down, which in this xacro convention means:

    j2 + j3 + j4 = -pi   (mod 2pi)

The fingertip (rlink2/llink2) extends roughly L4+L5 from arm_link5. Using the
URDF lengths L2=0.0829, L3=0.0829, L4=0.17455 and a fingertip offset of ~0.06 m,
we search for the pose that places the gripper tip at the desired (x,z).

The cube centre is at z_cube ~ 0.03 m (2 cm cube sitting on ground at z=0.02,
centre at z=0.03). The fingertip should be slightly above the cube centre so
closing the gripper clamps the cube faces.
"""
import math

L2 = 0.0829
L3 = 0.0829
L4 = 0.17455
TIP_OFFSET = 0.055  # approximate rlink2/llink2 extension past arm_link5 origin


def forward(j2, j3, j4, j5=0.0, tip_offset=TIP_OFFSET):
    """Forward kinematics to wrist (arm_link4->arm_link5) + fingertip offset."""
    # wrist position (end of L4)
    xw = L2 * math.sin(-j2) + L3 * math.sin(-(j2 + j3)) + L4 * math.sin(-(j2 + j3 + j4))
    zw = L2 * math.cos(j2) + L3 * math.cos(j2 + j3) + L4 * math.cos(j2 + j3 + j4)

    # Gripper vertical => tip extends straight down from wrist by tip_offset
    # In vertical pose j2+j3+j4 = -pi, so sin(-(j2+j3+j4))=0 and cos(j2+j3+j4)=-1,
    # meaning L4 already points down.  Add tip_offset straight down.
    xt = xw
    zt = zw - tip_offset
    return xt, zt, xw, zw


def search_vertical(target_x, target_z):
    """Find j2,j3,j4 satisfying j2+j3+j4=-pi and placing fingertip near target."""
    best = None
    best_err = 1e9
    for j2_deg in range(-170, -30, 2):
        for j3_deg in range(-80, 10, 2):
            j2 = math.radians(j2_deg)
            j3 = math.radians(j3_deg)
            # Vertical constraint
            j4 = -math.pi - j2 - j3
            if j4 > 0.0 or j4 < -math.pi / 2 - 0.01:
                continue
            x, z, xw, zw = forward(j2, j3, j4)
            err = math.hypot(x - target_x, z - target_z)
            if err < best_err:
                best_err = err
                best = (j2, j3, j4, x, z, xw, zw)
    return best, best_err


if __name__ == '__main__':
    # The standoff distance is 0.292 m from base_footprint.
    # base_footprint -> base_link is ~0.098 m forward (chassis origin offset),
    # so fingertip X relative to base_link should be about 0.292 - 0.098 = 0.194 m.
    # Cube centre Z is ~0.03 m; place fingertip at roughly same Z or 1 cm above.
    target_x = 0.20
    target_z = 0.03

    best, err = search_vertical(target_x, target_z)
    if best:
        j2, j3, j4, x, z, xw, zw = best
        print(f"Target fingertip: X={target_x:.3f} m, Z={target_z:.3f} m")
        print(f"Best vertical pose (j2,j3,j4 in DEGREES): "
              f"{math.degrees(j2):.1f}, {math.degrees(j3):.1f}, {math.degrees(j4):.1f}")
        print(f"Best vertical pose (j2,j3,j4 in RADIANS): "
              f"{j2:.4f}, {j3:.4f}, {j4:.4f}")
        print(f"Sum j2+j3+j4 = {math.degrees(j2+j3+j4):.1f}° (should be -180°)")
        print(f"Fingertip: X={x:.4f} m, Z={z:.4f} m")
        print(f"Wrist:     X={xw:.4f} m, Z={zw:.4f} m")
        print(f"Fit error: {err*1000:.2f} mm")
    else:
        print("No valid vertical pose found")
