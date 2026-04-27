# 90-Degree Turn Visual Guide

> **Note:** This document uses **theoretical formula values** (v=0.5 m/s, ω=4.699 rad/s,
> t=0.334s) to illustrate the differential drive math. The actual implementation in
> `manual_control.py` uses **closed-loop IMU/odom feedback** with `turn_omega=0.9 rad/s`
> — it does NOT rely on open-loop timing. The robot monitors yaw displacement and stops
> when exactly 90° is reached, regardless of timing.

## Top-View Diagram of In-Place 90° Turn

```
INITIAL POSITION (t=0):

    Desired Forward Direction
              ↑
              │
         ╭─[5]─╮
         │  🤖 │  Robot (bird's eye view)
         ╰─[0]─╯
              │
              │
         __________
    

DURING TURN (t=0 to 0.334 seconds):

Left pair velocity: -0.5 m/s (backward →)  ←── Left wheels (front+back)
                                │
        ╭─────────────────────[1]────────────────────╮
        │                       │                      │
Right pair velocity: +0.5 m/s   │   Robot rotates      │
(forward →)                     │   counterclockwise   │
        │                       ↓                      │
        │                    ⟲ 🤖                     │
        │                     [0]                     │
        │                                            │
        ╰────────────────────────────────────────────╯
                    Wheel Separation L = 0.2128m
                    (4 wheels: 2 left + 2 right)


FINAL POSITION (t=0.334 seconds):

    Original Forward
    Direction (now sideways)
        ←
        │
    ╭─[0]─╮
    │  🤖 │  Robot has rotated 90°
    ╰─[5]─╯
        │
        ↓
    New Forward Direction
```

---

## Velocity Vector Diagram

```
DURING 90° LEFT TURN (IN-PLACE):

                     Left Pair
                    (front+back)
                    -0.5 m/s
                        ↑
                    
    ┌─────────────────────────────────────┐
    │                                     │
    │     ╭──╮           ╭──╮             │  front wheels
    │     │  │ ← -v_R    │  │ ← -v_L      │
    │     ╰──╯           ╰──╯             │
    │                                     │
    │         Robot Base (top view)       │
    │                                     │
    │     ╭──╮           ╭──╮             │  back wheels
    │     │  │ ← +v_L    │  │ ← +v_R      │
    │     ╰──╯           ╰──╯             │
    │                                     │
    │    +0.5 m/s                         │
    │        ↑                        ↑   │
    │                          +0.5 m/s   │
    │                                     │
    │      Angular velocity ω             │
    │        ↺ (counterclockwise)         │
    │    ω = 2v/L = 4.699 rad/s           │
    │                                     │
    └─────────────────────────────────────┘

    Left pair backward + Right pair forward (all 4 wheels)
    = Robot rotates on center without drifting
```

---

## Time vs Angle Graph

```
ROTATION ANGLE vs TIME:

Angle (degrees)
    │         
    │        ╱─────── (forward motion ends)
 90 │       ╱
    │      ╱
 75 │     ╱
    │    ╱
 60 │   ╱
    │  ╱
 45 │ ╱  (linear slope = constant angular velocity)
    │╱
 30 │ ─────
    │     
 15 │        ω = 4.699 rad/s = 269.2°/sec
    │     
  0 └──────────────────────────────────────► Time (seconds)
    0   0.05  0.10  0.15  0.20  0.25
    
Formula: θ(t) = ω * t = 4.699 * t degrees
At t=0.334s: θ = 4.699 * 0.3343 = 1.57 rad = 90°
```

---

## Speed and Turn Time Comparison

```
DIFFERENT WHEEL SPEEDS - TURN TIME COMPARISON (theoretical):

Wheel Speed (m/s) │ Angular Vel │ 90° Turn Time │ Visual
──────────────────┼─────────────┼───────────────┼─────────────────────
    0.3 m/s       │ 2.82 rad/s  │   0.557 sec   │ ──────────────●
    0.5 m/s       │ 4.699 rad/s │   0.334 sec   │ ───● (formula default)
    0.8 m/s       │ 7.52 rad/s  │   0.209 sec   │ ─●
    1.0 m/s       │ 9.40 rad/s  │   0.167 sec   │ ●

Note: Actual implementation commands turn_omega=0.9 rad/s directly
and uses closed-loop yaw feedback (not open-loop timing).

Key insight (for open-loop formula): 
FASTER wheel speed → FASTER turn → SHORTER duration
Inverse relationship: t ∝ 1/v
```

---

## Different Wheel Separations

```
DIFFERENT ROBOT WIDTHS - TURN TIME:

Wheel Sep (m) │ Angular Vel │ 90° Turn Time │ Robot Type
──────────────┼─────────────┼───────────────┼────────────────────
  0.10 m      │ 10.0 rad/s  │   0.157 sec   │ ●── Narrow (compact)
  0.2128 m     │  4.699 rad/s │   0.334 sec   │ ●──── Medium (X3plus)
  0.20 m      │  5.0 rad/s  │   0.314 sec   │ ●────── Wide (stability)
  0.25 m      │  4.0 rad/s  │   0.393 sec   │ ●────────── Very wide

Key insight:
WIDER wheel separation → SLOWER turn → LONGER duration
Direct relationship: t ∝ L
```

---

## Formula Derivation Flow Chart

```
START: We want 90° turn
    │
    ├─► Difference in wheel velocities
    │   v_R - v_L = v - (-v) = 2v
    │
    └─► Divided by wheel separation
        Angular velocity: ω = 2v/L
    
    │
    ├─► 90° = π/2 radians
    │
    └─► Time = angle / angular_velocity
        t = (π/2) / (2v/L)
        t = π*L / (4*v)
    
END: Turn time calculated
```

---

## Motor Control Signals During 90° Turn

> **Theoretical diagram** — the actual implementation sends `cmd_vel.angular.z = 0.9`
> and the DiffDrive plugin converts that to wheel velocities internally. The turn
> ends via IMU/odom yaw feedback, not a fixed timer.

```
TIME (seconds) — theoretical open-loop model
0.000 ─────────────────────────────────────► 0.334s ─► 0.534s

LEFT MOTOR (backward):
0    ┌─────────────────────────────────────────┐
     │ -0.5 m/s (backward)                     │
-0.5 │                                         │
     │                                         └─────► 0
     │

RIGHT MOTOR (forward):
0.5  │                                         ┐
     │ +0.5 m/s (forward)                      │
0    └─────────────────────────────────────────┘ ─► 0
     │

ROBOT ROTATION ANGLE:
90°  │                                    ╱
     │                                 ╱
60°  │                              ╱
     │                           ╱
30°  │                        ╱
     │                     ╱
0°   └──────────────────╱─────────────────► 0°
     ▲ Start             ▲ Goal reached     ▲ Stop
     0.000s             0.334s             0.534s
     
LEGEND:
├── Acceleration phase (minimal)
├── Constant velocity phase (4.699 rad/s)
├── Deceleration phase (stop)
└── Hold position
```

---

## Comparison: In-Place vs Arc Turn

```
IN-PLACE TURN (Type 1):

Start            During Turn          End
  │             ╱ ─ ─ ─ ─            
  ↓            ╱                      →
Robot ────► Robot rotates ────────► Robot
  ↓          in place           (same location)
            Closed-loop (yaw feedback, ~0.9 rad/s)
            No forward drift


MOVING ARC TURN (Type 2):

Start         During Turn            End
  ↓          ╱                    
Robot ──► Robot turns while    ───► Robot
  ↓        moving forward          (moved forward)
           Duration: ~0.3s
           Forward motion: ~0.3m


CHOICE DEPENDS ON:
┌──────────────────────┬──────────────────────┐
│   In-Place Turn      │    Arc Turn          │
├──────────────────────┼──────────────────────┤
│ • Need precise turn  │ • Want smooth arc    │
│ • Limited space      │ • More forward dist. │
│ • Slow execution     │ • Efficient use of   │
│ • No forward drift   │   forward momentum   │
└──────────────────────┴──────────────────────┘
```

---

## Formula Quick Reference Card

```
╔════════════════════════════════════════════════════════════════╗
║          90-DEGREE TURN FORMULA - QUICK REFERENCE              ║
╠════════════════════════════════════════════════════════════════╣
║                                                                ║
║  ANGULAR VELOCITY:  ω = 2v / L                                 ║
║  Wheel Speed:       v = 0.5 m/s (formula example)              ║
║  Wheel Separation:  L = 0.2128 m (X3plus)                      ║
║  RESULT:            ω = 4.699 rad/s (theoretical)              ║
║                                                                ║
║  TURN TIME:         t = π*L / (4*v)                            ║
║  Angle:             θ = π/2 (90 degrees)                       ║
║  RESULT:            t = 0.334 seconds (theoretical)            ║
║  VERIFICATION:      θ = ω × t                                  ║
║  Expected:          1.57 rad = 90° ✓                           ║
║                                                                ║
╠════════════════════════════════════════════════════════════════╣
║  ACTUAL IMPLEMENTATION (manual_control.py):                    ║
║  Command:     cmd_vel.angular.z = 0.9 rad/s                    ║
║  Feedback:    Closed-loop IMU/odom yaw tracking                ║
║  Stops when:  Yaw displacement ≥ π/2 (90°)                     ║
║  NOT timed — stops via sensor feedback                         ║
╚════════════════════════════════════════════════════════════════╝
```

---

## Actual Testing Workflow

```
STEP 1: Launch simulation
┌─────────────────────────────────────────┐
│ ros2 launch sim_gazebo_bringup          │
│   gazebo.launch.py                      │
│                                         │
│ Then in another terminal:               │
│ ros2 run x3plus_examples manual_control │
└─────────────────────────────────────────┘

STEP 2: Execute 90° turn
┌─────────────────────────────────────────┐
│ Press keys 1-4 for turn type            │
│ Implementation:                         │
│ - Sends cmd_vel.angular.z = 0.9 rad/s   │
│ - Monitors IMU yaw (or odom fallback)   │
│ - Stops when yaw displacement ≥ π/2     │
│ - Closed-loop: no fixed timing needed   │
└─────────────────────────────────────────┘

STEP 3: Verify
┌─────────────────────────────────────────┐
│ Console shows actual rotation angle     │
│ Expected: ~90° (typically within ±1°)   │
│ Check position drift:                   │
│ Expected: minimal (in-place turn)       │
└─────────────────────────────────────────┘
```

---

## Summary Visualization

```
                    THE 90-DEGREE TURN
                    ═════════════════════

    BEFORE          EXECUTION           AFTER
    (0.000s)        (0.334s)           (0.334s)
    
      ↑                ⟲                 ←
      │                │                 │
    ┌─┴─┐            ┌─┼─┐            ┌─┴─┐
    │ROB│            │ROB│            │ROB│
    └───┘            └───┘            └───┘
      │                │                 │
      ↓               ∨                  →
   
   Forward         Rotating          Rotated 90°
  Direction    (ω = 4.699 rad/s)    (new forward)
  
        L = 0.2128m
        Theoretical: v=0.5 m/s, ω=4.699 rad/s, t=0.334s
        Actual: turn_omega=0.9 rad/s, closed-loop yaw feedback
```
