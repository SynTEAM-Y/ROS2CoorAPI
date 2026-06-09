import math

# Link lengths
L2 = 0.0829
L3 = 0.0829
# To joint 5 (gripper base)
L4 = 0.17455

# In the arm_link2 frame (all angles 0):
# X is X (forward)
# Y is old -Z (down)
# Z is old Y (left)

# Wait. rpy=" -pi/2 0 0 " (rotate around X by -90 deg)
# X_new = X_old
# Y_new = Z_old * cos(-90) - Y_old * sin(-90) = Y_old (Wait, standard rotation matrix)
# Let's use simple ROS conventions:
# rotX(-pi/2) -> Y_new points in +Z_old, Z_new points in -Y_old
# If Y_new points in +Z_old, then a translation of "0 -L 0" means moving in -Y_new, which is -Z_old.
# So at joint 0, it extends DOWN ?? No, the arm goes UP.
# Let's write a simple script:

def pose(j2, j3, j4):
    # angle 0 = UP (Z axis)
    # angle -90 (-pi/2) = FORWARD (X axis)
    # angle -180 (-pi) = DOWN (-Z axis)
    
    # We want to know X and Z coordinates of the end of link 4.
    # Joint 2 is at Origin (0, 0)
    # Actually base_link -> joint2 is offset: x=0.098, z=0.102+0.040=0.142
    
    # Link 2 ends at:
    x2 = L2 * math.sin(-j2)
    z2 = L2 * math.cos(j2)
    
    # Link 3 ends at:
    x3 = x2 + L3 * math.sin(-(j2+j3))
    z3 = z2 + L3 * math.cos(j2+j3)
    
    # Link 4 ends at:
    x4 = x3 + L4 * math.sin(-(j2+j3+j4))
    z4 = z3 + L4 * math.cos(j2+j3+j4)
    
    return x4, z4

print("Old REACH_DOWN (j2=-1.45, j3=-0.54, j4=-1.21):")
print(pose(-1.45, -0.54, -1.21))

print("My REACH_DOWN (-1.30, -0.14, -1.56):")
print(pose(-1.30, -0.14, -1.56))

# Let's find a REACH_DOWN where j2+j3+j4 = -3.14 (perfectly down)
# and X is approx 0.06 larger than old REACH_DOWN X.
# old REACH_DOWN X: 
x_old, z_old = pose(-1.45, -0.54, -1.21)
print(f"Target X: {x_old + 0.06}, Target Z: {z_old}")

def find_inverse_kinematics(target_x, target_z):
    # we know j2+j3+j4 = -3.14 (so link 4 points straight down)
    # x3 = target_x - L4 * sin(pi) = target_x
    # z3 = target_z - L4 * cos(pi) = target_z + L4
    x3 = target_x
    z3 = target_z + 0.17455
    
    # Now we need to find j2 and j3 to reach (x3, z3) with two links of length 0.0829
    # Distance to (x3, z3) must be <= 2 * 0.0829 = 0.1658
    d = math.hypot(x3, z3)
    if d > 2 * 0.0829:
        print(f"Unreachable! d={d} > 0.1658")
        return
        
    # Standard 2-link IK
    # cos(j3) = (d^2 - L2^2 - L3^3) / (2 * L2 * L3)
    cos_j3_val = (d**2 - L2**2 - L3**2) / (2 * L2 * L3)
    j3_candidate = math.acos(cos_j3_val)
    # we usually want negative j3 for this robot? Let's check old j3 is -0.54
    # Actually wait, in my script, 0 is UP, -90 is FORWARD.
    # Angle to target:
    alpha = math.atan2(x3, z3)
    # angle inside triangle:
    beta = math.acos((d**2 + L2**2 - L3**2) / (2 * d * L2))
    
    j2 = -(alpha + beta) # or -(alpha - beta)
    j3 = -j3_candidate # or +
    
    # Test combinations:
    for s_beta in [-1, 1]:
        for s_j3 in [-1, 1]:
            j2_test = -(alpha + s_beta * beta)
            # using my standard angles: j2_test is the absolute angle of link 2.
            # but my j3 is relative to j2.
            j3_test = s_j3 * j3_candidate 
            p_x, p_z = pose(j2_test, j3_test, -3.14159 - (j2_test + j3_test))
            if abs(p_x - target_x) < 0.001 and abs(p_z - target_z) < 0.001:
                print(f"Found IK: j2={j2_test:.3f}, j3={j3_test:.3f}, j4={-3.14159 - (j2_test + j3_test):.3f}")

find_inverse_kinematics(x_old + 0.06, z_old)

