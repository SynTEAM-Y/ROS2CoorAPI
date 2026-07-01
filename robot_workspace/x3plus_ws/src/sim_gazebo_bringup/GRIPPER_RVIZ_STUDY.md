# Gripper 4-Bar Mechanism — RViz TF Study

## Method
Used `robot_state_publisher` + `/joint_states` to compute TF tree from the URDF.
Drove `grip_joint` from `-1.57` to `+0.48` rad in `0.05` rad steps and recorded
the TF of every gripper link in `arm_link5` frame. The gripper_mechanism is
unchanged (5 mimic joints: rlink_joint2/3, llink_joint1/2/3).

## Key Finding #1: The pads stay parallel
The pad (rlink2) orientation in arm_link5 frame is `(±180°, 90°, 0°)` at every q.
The pads translate but do NOT rotate. 4-bar mechanism works correctly.

## Key Finding #2: Pad gap is much steeper than your formula
- **Your formula** (from `GRIPPER_QUICK_REFERENCE.md`): `Y_diff = 27 − 20.06 × q` mm
- **Actual URDF** (from this study):

| grip_joint (rad) | r2.y (mm) | l2.y (mm) | Pad gap (mm) |
|---:|---:|---:|---:|
| -1.57 (URDF lower) | -42.63 | +42.38 | **85.0** (full open) |
| -1.50 (your "OPEN")| -42.62 | +42.37 | **85.0** |
| **-0.676 (your "HOLD")** | **-31.85** | **+31.51** | **63.4** ← NOT 48! |
| -0.37 (matches 48mm) | -24.18 | +23.83 | **48.0** ✓ |
| 0.00 (mid) | -13.38 | +13.03 | 26.4 |
| +0.45 (URDF upper) | -0.80 | +0.46 | 1.3 (almost closed) |

**Slope**: actual 56 mm/rad (URDF), vs 20 mm/rad (your formula). The 2.8× slope
difference is why `q=-0.676` doesn't give 48mm.

## Key Finding #3: Right/left symmetry
- rlink1 at `y = -12.62mm`, llink1 at `y = +12.38mm` (avg offset = -0.12mm)
- rlink3 at `y = -4.50mm`, llink3 at `y = +4.50mm` (perfectly symmetric)
- 0.25mm asymmetry comes from the URDF values: `xyz="-0.0035 -0.012625 -0.0685"`
  vs `"xyz=-0.0035 0.012375 -0.0685"`. Negligible (0.25mm vs 85mm travel).

## Key Finding #4: Joint range
- `grip_joint` lower = `-π/2 ≈ -1.57`, upper = `+0.45` (URDF limits)
- Pad gap range: **85.0mm (q=-1.57) to 1.3mm (q=+0.45)**
- The "fully closed" position (1.3mm gap) is at q=+0.45, NOT q=0
- 0 rad is the MID position (26.4mm gap), not the closed position
- **You were commanding the gripper the WRONG WAY**: "GRIPPER_HOLD = -0.676"
  is in the "more open" direction; "GRIPPER_CLOSE = 0.0" is the mid position
- The actual closed position needs `q = +0.45` (positive direction)

## Recommended corrections to the autopilot
| Old (wrong) | New (correct) | Pad gap |
|---|---|---|
| `GRIPPER_OPEN = -1.54` | `GRIPPER_OPEN = -1.54` | 85mm (unchanged) |
| `GRIPPER_HOLD = -0.676` | `GRIPPER_HOLD = -0.37` | 48mm ✓ (4cm cube + 8mm) |
| `GRIPPER_CLOSE = 0.0` | `GRIPPER_CLOSE = 0.45` | 1.3mm (fully closed) |

The new values use **the right direction** (positive q = close) and put
GRIPPER_HOLD at the right gap for the 4cm cube with 8mm clearance.
