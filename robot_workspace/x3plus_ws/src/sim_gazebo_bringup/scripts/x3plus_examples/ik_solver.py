import math

L2 = 0.0829
L3 = 0.0829
L4 = 0.17455

# Target is for the wrist joint (joint 4)
# User wants gripper to point straight DOWN.
# That means j2 + j3 + j4 = -3.14159
# The wrist will be exactly 0.17455 m above the gripper tip.

def check_pose(j2, j3, j4):
    x2 = L2 * math.sin(-j2)
    z2 = L2 * math.cos(j2)
    x3 = x2 + L3 * math.sin(-(j2+j3))
    z3 = z2 + L3 * math.cos(j2+j3)
    x4 = x3 + L4 * math.sin(-(j2+j3+j4))
    z4 = z3 + L4 * math.cos(j2+j3+j4)
    return x4, z4

print("Old REACH_DOWN = -1.45, -0.54, -1.21")
old_x, old_z = check_pose(-1.45, -0.54, -1.21)
print(f"X: {old_x:.3f}, Z: {old_z:.3f}")

print("Let's try to extend X by 0.06 but keep Z the same or lowering Z if it's too high.")

# Find max X possible natively:
# for 2 links L2 and L3, max reach is L2+L3 = 0.1658 (this is where j3=0)
print("When j3=0, the arm is straight.")
for j2_deg in range(-180, 0, 5):
    j2 = math.radians(j2_deg)
    j3 = 0
    # To keep gripper vertical, j4 = -3.14 - j2 - j3
    j4 = -3.14159 - j2
    x, z = check_pose(j2, j3, j4)
    print(f"j2: {j2_deg:4.0f} deg -> X: {x:.3f}, Z: {z:.3f}")

