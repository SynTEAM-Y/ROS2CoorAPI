import math

L2 = 0.0829
L3 = 0.0829
L4 = 0.17455

def check_pose(j2, j3, j4):
    x2 = L2 * math.sin(-j2)
    z2 = L2 * math.cos(j2)
    x3 = x2 + L3 * math.sin(-(j2+j3))
    z3 = z2 + L3 * math.cos(j2+j3)
    x4 = x3 + L4 * math.sin(-(j2+j3+j4))
    z4 = z3 + L4 * math.cos(j2+j3+j4)
    return x4, z4

print("Searching for j2, j3, j4 that gives X around 0.22 and Z as low as possible...")
best_z = 100
best_joints = None
for j2_deg in range(-150, -60, 5):
    for j3_deg in range(-120, 0, 5):
        for j4_deg in range(-120, 0, 5):
            j2 = math.radians(j2_deg)
            j3 = math.radians(j3_deg)
            j4 = math.radians(j4_deg)
            x, z = check_pose(j2, j3, j4)
            if 0.215 < x < 0.225:
                # We want the gripper as vertical as possible, so j2+j3+j4 close to -180 (-3.14)
                # But constrained to give us X=0.22
                angle_diff = abs(-3.14159 - (j2+j3+j4))
                if angle_diff < 0.5: # within 30 deg of vertical
                    if z < best_z:
                        best_z = z
                        best_joints = (j2_deg, j3_deg, j4_deg, x, z, math.degrees(j2+j3+j4))
                        
print("Best:", best_joints)
