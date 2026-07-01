# 4-Bar Mechanism Study — Critical Findings

## Summary of the study
Driven `grip_joint` from `-1.5` to `+0.45` rad and recorded the TF of all 6
gripper links (3 per side: input crank, output bar, coupler). For each q,
computed:
- A = grip_joint origin (ground pivot for input crank) — FIXED
- C = rlink_joint3 origin (ground pivot for coupler) — FIXED
- B = rlink2 origin (pad frame origin) — moves with q
- D_pad = end of pad mesh (B + 0.054 * pad_x_axis)
- D_coup = end of coupler mesh (C + 0.036 * coup_x_axis)

In a REAL 4-bar, D_pad and D_coup should be the same point (the coupler is
pinned to the pad at joint D).  In this URDF, they are NOT the same point.

## Critical Finding #1: The 4-bar is NOT geometrically valid

Grashof's law check (4-bar must have s+l < m+m to complete a full rotation):
```
L1 (crank)    = 0.030 m
L2 (pad)      = 0.054 m
L3 (coupler)  = 0.036 m
L4 (ground)   = 0.0197 m
smallest + largest = 0.0197 + 0.0540 = 0.0737
middle two         = 0.0300 + 0.0360 = 0.0660
Grashof (0.0737 < 0.0660): FALSE
```
The 4-bar **CANNOT close** with these link lengths, even in principle.

## Critical Finding #2: The coupler is NOT a real constrainer
The "drift" between the pad tip (D_pad) and the coupler tip (D_coup):
| grip_joint | D_pad (mm) | D_coup (mm) | drift (mm) |
|---:|---|---|---:|
| -1.5 (OPEN) | (-3.5, -42.6, -13.1) | (-3.5, -40.4, -48.0) | **34.9** |
| -0.676 | (-3.5, -32.0, +8.4) | (-3.5, -27.0, -22.4) | **31.2** |
| 0 (mid) | (-3.5, -13.4, +15.5) | (-3.5, -4.5, -14.5) | **31.3** |
| 0.45 (CLOSED) | (-3.5, -0.3, +12.8) | (-3.5, +11.2, -18.1) | **33.0** |

The pad and coupler trace **completely different paths**. They are
geometrically independent. The coupler does NOT physically constrain the pad.

## What actually keeps the pads parallel?
The pads are constrained by the **mimic on rlink_joint2** (mimic=-1):
- grip_joint rotates rlink1 by +q
- rlink_joint2 rotates rlink2 by -q (mimic)
- Net rotation on rlink2 = +q + (-q) = 0

So rlink2's orientation in arm_link5 frame is **constant** regardless of q:
- pad x-axis = (0, 0, +1) (the direction the pad extends)
- pad y-axis = (0, +1, 0) (perpendicular to the pad surface, right side)
- pad y-axis = (0, -1, 0) (perpendicular, left side — mirror)

The right and left pads have opposite y-axes, so they face each other.
Their x-axes are parallel (both at +Z in arm_link5 frame). The pads stay
parallel because of the mimic relationship, not because of the coupler.

## What the 4-bar SHOULD look like (in a real parallel gripper)
A real 4-bar has 4 revolute joints A-B-C-D forming a closed loop:
- A: arm_link5 → rlink1 (input crank, ground pivot)
- B: rlink1 → rlink2 (pad, intermediate joint)
- C: arm_link5 → rlink3 (coupler, ground pivot)
- D: rlink2 → rlink3 (pad-coupler joint — **NOT IN THIS URDF**)

In a real 4-bar:
- The pad (rlink2) and coupler (rlink3) are Pinned at joint D
- D's position is determined by the closure of the 4-bar
- The 4-bar must be Grashof (s+l < m+m) for full rotation
- The pad translates linearly (constant orientation) because the 4-bar
  constrains its motion

## What this URDF has instead
- rlink1, rlink2, rlink3 are all independent chains
- rlink2's orientation is constant (via mimic=-1 on rlink_joint2)
- rlink3 rotates (via mimic=+1 on rlink_joint3) but its position is FIXED
  (parented to arm_link5)
- **No joint D** between rlink2 and rlink3
- The pads are parallel by mimic, not by 4-bar closure
- The coupler is a **decorative ghost link** — it rotates in sync with the
  gripper but doesn't physically constrain anything

## The "mess" the user saw
The 4-bar mechanism LOOKS like a real 4-bar (3 links per side, coupler between
pads and ground) but it's actually a "5-bar with coupled inputs":
- 5-bar: arm_link5 → rlink1 → rlink2 (one chain) and arm_link5 → rlink3 (another)
- Coupled inputs: rlink1 and rlink3 rotate together (mimic=+1 on rlink_joint3)
- This reduces the 5-bar to a 4-bar DOF-wise
- But the coupler (rlink3) is just a "ghost" — it doesn't pin the pad

The visual "mess" comes from:
1. The coupler traces a different path than the pad (31-35mm drift)
2. The coupler rotates in sync with the gripper but isn't pinned to the pad
3. In the sim, the coupler LOOKS like it's constraining the pad, but it isn't

## Two options to fix

**Option A: Remove the coupler (clean 4-link gripper)**
- Remove rlink3, llink3, rlink_joint3, llink_joint3
- The pads stay parallel via the mimic on rlink_joint2 alone
- Cleaner visual, simpler mechanism
- Same behaviour (pads still parallel, still close on the cube)

**Option B: Make a REAL 4-bar constrainer**
- Reposition rlink_joint3 so the 4-bar geometry closes
- Add a D joint between rlink2 and rlink3 (and similarly for left)
- Use closed-loop mimic that respects the 4-bar constraint
- Grashof's law check needed for full rotation
- This is the "real" 4-bar but requires careful geometry tuning
