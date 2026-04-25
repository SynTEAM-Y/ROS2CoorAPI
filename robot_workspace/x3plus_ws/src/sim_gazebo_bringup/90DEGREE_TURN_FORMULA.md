# 90-Degree Turn Formula - Quick Reference Guide

## Problem Statement
**Rotate the robot's chassis (car base) by exactly 90 degrees using differential drive control.**

---

## Mathematical Formula

### The Differential Drive Model

For a 4-wheel skid-steer differential drive robot (left pair + right pair):

```
Robot Top View:

       Robot Forward Direction
            ↑
            │
    ────────┼────────
   │  ⊙     │     ⊙  │  front-left / front-right wheels
   │        │        │
   │  ⊙     │     ⊙  │  back-left / back-right wheels
   └────────┴────────┘
        ↑       ↑
       v_L     v_R
   (left pair) (right pair)

   Separation Distance: L = 0.2128 m
```

The left pair (front_left + back_left) and right pair (front_right + back_right)
are driven together. The DiffDrive plugin commands all 4 wheels simultaneously.

### Core Formula: Angular Velocity

The angular velocity of a differential drive robot is determined by the difference in wheel velocities:

$$\Large \omega = \frac{v_R - v_L}{L}$$

Where:
- **ω** = angular velocity of robot (rad/s)
- **v_R** = right wheel pair velocity (m/s) — front_right + back_right
- **v_L** = left wheel pair velocity (m/s) — front_left + back_left
- **L** = wheel separation distance (m) = 0.2128 m for X3plus

---

## In-Place Turn (Point Turn) - 90°

### Setup
- **Left pair** (front_left + back_left): Move backward at speed $v$ → $v_L = -v$
- **Right pair** (front_right + back_right): Move forward at speed $v$ → $v_R = +v$
- Result: Robot spins on its center (skid-steer)

### Derived Angular Velocity

Substitute into the formula:

$$\omega = \frac{v - (-v)}{L} = \frac{2v}{L}$$

### Turn Duration for 90°

To rotate 90 degrees (π/2 radians):

$$t = \frac{\theta}{\omega} = \frac{\pi/2}{2v/L}$$

$$\boxed{t = \frac{\pi L}{4v}}$$

### Practical Calculation

**Given** (default X3plus parameters):
- Wheel separation: **L = 0.2128 m**
- Wheel speed: **v = 0.5 m/s**

**Step 1: Calculate angular velocity**
$$\omega = \frac{2 \times 0.5}{0.2128} = \frac{1.0}{0.2128} = 4.699 \text{ rad/s}$$

**Step 2: Calculate turn time**
$$t = \frac{3.14159 \times 0.2128}{4 \times 0.5} = \frac{0.6685}{2.0} = 0.3343 \text{ seconds}$$

**Step 3: Verify the result**
- Angle rotated: $\theta = \omega \times t = 4.699 \times 0.3343 = 1.5708$ rad
- Convert to degrees: $1.5708 \text{ rad} \times \frac{180°}{\pi} = 90°$ ✓

---

## With Different Parameters

### If you change wheel speed to 0.8 m/s:
$$t = \frac{3.14159 \times 0.2128}{4 \times 0.8} = \frac{0.6685}{3.2} = 0.209 \text{ seconds}$$ (FASTER!)

### If wheel separation is 0.20 m (narrower robot):
$$t = \frac{3.14159 \times 0.20}{4 \times 0.5} = \frac{0.6283}{2.0} = 0.314 \text{ seconds}$$ (FASTER!)

### If you want 45° instead of 90°:
$$t_{45°} = \frac{\pi L}{8v} = \frac{t_{90°}}{2}$$

---

## Implementation in Code

The actual implementation uses **closed-loop feedback** (IMU or odometry) rather
than open-loop timing. The theoretical values are logged for reference:

```python
# Robot parameters (manual_control.py)
wheel_separation = 0.2128  # m  — L
wheel_radius     = 0.04    # m  — r
turn_wheel_speed = 0.5     # m/s — v  (used only for theoretical display)
max_linear_velocity  = 0.3   # m/s
max_angular_velocity = 1.0   # rad/s

# Theoretical calculation (displayed in console, NOT used to time the turn)
omega_theoretical = (2 * turn_wheel_speed) / wheel_separation  # = 4.699 rad/s
turn_time_estimate = (math.pi * wheel_separation) / (4 * turn_wheel_speed)  # = 0.334 s

# Actual command published on /cmd_vel:
angular_z_command = ±1.50  # rad/s  ← NOT 4.699 rad/s
                            # (lower value for stable, smooth closed-loop tracking)
linear_x_command  = 0.0    # m/s  (in-place) or 0.3 m/s (arc turn)
target_angle      = π/2    # = 1.5708 rad

# Closed-loop execution:
# 1. Read start_yaw from /imu (Gazebo) or /odom (RViz-only)
# 2. Publish cmd_vel with angular_z = ±1.50 rad/s
# 3. Each cycle: compute delta = current_yaw − start_yaw
# 4. When |delta| >= π/2 → publish zero cmd_vel → done
```

> **Why 1.50 rad/s instead of 4.699 rad/s?**  
> The 4.699 rad/s is the theoretical angular rate if wheels spin at ±0.5 m/s in
> opposite directions. The commanded `/cmd_vel angular.z = 1.50 rad/s` is the
> velocity setpoint sent to the DiffDrive plugin — it translates to different
> individual wheel speeds via the plugin's internal controller.

---

## Control Commands

| Key   | Action                       | Formula Used                                    |
|-------|------------------------------|-------------------------------------------------|
| **1** | 90° left turn (in-place)     | $\omega = \frac{2v}{L}$, $t = \frac{\pi L}{4v}$ |
| **2** | 90° right turn (in-place)    | Same but negative ω                             |
| **3** | 90° left + forward movement  | $\omega = \frac{2v}{L}$ with forward component  |
| **4** | 90° right + forward movement | Same with forward component                     |

---

## Common Scenarios

### Scenario 1: 180-Degree Turn
$$t_{180°} = 2 \times t_{90°} = 2 \times 0.334 = 0.669 \text{ seconds}$$

### Scenario 2: 45-Degree Turn
$$t_{45°} = \frac{t_{90°}}{2} = 0.167 \text{ seconds}$$

### Scenario 3: 360-Degree Full Rotation
$$t_{360°} = 4 \times t_{90°} = 1.337 \text{ seconds}$$

---

## What Affects Turn Duration?

| Parameter              | Effect        | Relationship                      |
|------------------------|---------------|-----------------------------------|
| ↑ Wheel speed (v)      | ↓ Turn FASTER | $t \propto \frac{1}{v}$ (inverse) |
| ↑ Wheel separation (L) | ↑ Turn SLOWER | $t \propto L$ (direct)            |
| ↑ Desired angle (θ)    | ↑ Turn LONGER | $t \propto \theta$ (direct)       |

---

## Verification Checklist

After executing a 90° turn, verify:

- [ ] Robot rotated approximately 90 degrees
- [ ] Robot stayed in roughly the same location (minimal forward drift)
- [ ] Turn duration matches calculated time (±50ms tolerance)
- [ ] All 4 wheels moved (left pair backward, right pair forward)
- [ ] No excessive wheel slipping observed

---

## Advanced: Custom Angle Formula

For **any angle θ** (in degrees):

1. Convert to radians: $\theta_{rad} = \theta \times \frac{\pi}{180}$

2. Calculate time:
$$t_{\theta} = \frac{\theta_{rad}}{\omega} = \frac{\theta_{rad} \times L}{2v}$$

### Example: 270-Degree Turn
- $\theta = 270°$
- $\theta_{rad} = 270 \times \frac{\pi}{180} = 4.712$ rad
- $t = \frac{4.712 \times 0.2128}{2 \times 0.5} = \frac{1.003}{1.0} = 1.003$ seconds

---

## File Locations

- **Manual control node**: `x3plus_examples/x3plus_examples/manual_control.py`
- **Arm controller**: `x3plus_examples/x3plus_examples/arm_controller.py`
- **Gazebo launch**: `sim_gazebo_bringup/launch/gazebo.launch.py`
- **RViz launch**: `sim_gazebo_bringup/launch/robot_rviz.launch.py`

---

## Testing the Formula

```bash
# Terminal 1: Start simulation
ros2 launch sim_gazebo_bringup gazebo.launch.py use_rviz:=false

# Terminal 2: Run control node
ros2 run x3plus_examples manual_control

# Terminal 3: Monitor the output
# You'll see theoretical + actual execution parameters when pressing '1'-'4'
#
# Theoretical: ω = 2v/L = 4.699 rad/s, t = 0.3343 s
# Actual: Command ω = 1.50 rad/s, closed-loop /odom yaw tracking
```

---

## Summary

| What               | Formula                    | Default Values   | Result         |
|--------------------|----------------------------|------------------|----------------|
| Angular velocity   | $\omega = \frac{2v}{L}$    | v=0.5, L=0.2128  | 4.699 rad/s    |
| Turn time (90°)    | $t = \frac{\pi L}{4v}$     | v=0.5, L=0.2128  | 0.334 sec      |
| Turn angle vs time | $\theta = \omega \times t$ | ω=4.699, t=0.334 | 1.57 rad = 90° |

**The robot will complete a perfect 90-degree turn automatically using these formulas!**
