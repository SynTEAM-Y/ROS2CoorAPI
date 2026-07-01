# Multi-Robot Cube-Pick / Dual-Robot Sink-Transport Scene

This document describes the Gazebo simulation scene that follows the same
structure, assets, robot configuration, and behaviour patterns used in
`scripts/x3plus_examples/vision_autopilot_simple.py`, but extends it to
three X3Plus robots that pick a blue cube, place it on the sink (the
sink IS the yellow drop target per the task spec), and then
collaboratively lift the sink from its two red handles.

The whole setup is launched with a single command:

```bash
ros2 launch sim_gazebo_bringup multi_robot_cube_sink.launch.py
```

| File | Role |
|------|------|
| `worlds/multi_robot_scene.sdf` | Gazebo world (ground plane, sun, wall, cube, sink, green landing pad). |
| `launch/multi_robot_cube_sink.launch.py` | Spawns Gazebo, the three robots, all TF bridges, the mimic relays, and the autopilot. |
| `scripts/x3plus_examples/multi_robot_cube_sink_autopilot.py` | The state machine that drives the three robots. |
| `models/test_block/model.sdf` | Blue cube (2 cm × 2 cm × 2 cm, dynamic, 5 g). |
| `models/sink/model.sdf` | Sink, built from `meshes/sink/sink.obj` (dynamic, 2 kg). |
| `models/landing_pad/model.sdf` | Green landing area (static, 500 mm × 500 mm). |
| `meshes/sink/sink.obj` + `.mtl` | Sink mesh (Blender 5.1.2 export). Yellow basin + red handles. |

---

## 1. World layout

World frame: `X` is the +X robot-forward axis when a robot sits at the
origin, `Y` is left, `Z` is up.  Units: metres.

```
                  wall (2.0, 2.5)   static obstacle

       landing_pad      sink        blue cube
        (2, 1.2)      (2, 0.0)      (2, -1.2)
            \\           |            //
             \\          v           //
            robot_3   ...    robot_2   <-- sink-grasp robots
                                       face +/-Y
        robot_1                      <-- cube pick robot
        (-1.5, 0)
```

### 1.2 Arm kinematic limit (cube on floor)

The X3Plus arm at the manufacturer REACH_DOWN pose (j2+j3+j4 = -pi)
puts the gripper pads at z~0.039 with the stock shoulder height
(z=0.102).  The pad tips (llink3) are at z~0.039.  A 2 cm cube
on the floor settles to z=0.01 (centre), so the pads are **2.9 cm
above the cube** — the gripper cannot clamp it.

Lowering the shoulder (z=0.030) brings the pads down to z~0.050
but the **arm body and finger tips drop below the floor**, the gripper
hits the ground in front of the cube, and the arm's reaction torque
tips the robot (front wheels lift off the ground).  So the arm cannot
reach the floor cube without one of:

* A **side approach** (robot approaches from the Y direction so the
  arm extends sideways, never downward)
* A **shortened arm_link5** (URDF edit: reduce the wrist segment so
  the folded arm fits inside the chassis envelope)
* A **raised cube** (small static platform under the cube)

This build keeps the stock shoulder (z=0.102) and the cube on the
floor, so the autonomous pickup will fail at the grasp step.  The
face-aligned approach, fine alignment (sub-2 mm), and the helpers'
sink stand-by + dual-robot lift all work correctly.
* **Sink (the yellow drop target)** at `(2.0, 0.0, 0.035)`, spawned
  with `roll=pi/2` so its local Z axis points along world Y. The two
  red handles therefore protrude in the world `+/-Y` directions. Robot 1
  places the blue cube on top of the basin rim.
* **Green landing pad** at `(2.0, 1.2, 0.001)` - static reference,
  mirrored about the sink centre so the sink is geometrically centred
  between the cube and the landing area.
* **Wall** at `(2.0, 2.5, 0.25)`, identical to `worlds/office.sdf` so the
  scene looks like the `vision_autopilot_simple.py` world.

> Per the task spec: **the sink IS the yellow drop target**. There is
> no separate `yellow_object` model in this scene - the blue cube is
> placed directly on the sink basin, and the helpers later lift the
> sink (with the cube on it) together.

### 1.1 Three robots in a straight line

All three robots are at `x = -1.5` (perpendicular distance from the
cube/sink line) along the world Y axis, 0.7 m apart so the chassis do
not collide at startup. All face +X initially:

```python
ROBOTS = [
    {'name': 'robot_1', 'x': -1.5, 'y':  0.0, 'Y': 0.0},   # cube pick + drop
    {'name': 'robot_2', 'x': -1.5, 'y': -0.7, 'Y': 0.0},   # sink -Y handle
    {'name': 'robot_3', 'x': -1.5, 'y':  0.7, 'Y': 0.0},   # sink +Y handle
]
```

Robot 1 drives forward to the cube and then to the sink. Robots 2 and
3 turn 90 degrees to face `+/-Y` and drive to the sink. All three share the
same URDF/xacro and the same mesh, but each gets a unique namespace
prefix in every joint, link, TF, and topic name (see
`launch/multi_robot_cube_sink.launch.py::_make_namespaced_urdf`).

---

## 2. Task sequence (state machine)

The state machine (`multi_robot_cube_sink_autopilot.py`) is structured
exactly like `vision_autopilot_simple.py` (IDLE -> ARM_TO_DRIVE -> APPROACH
-> FACE -> PRE_PICK_ALIGN -> PICKUP -> PICKUP_WAIT -> DRIVE -> FACE -> DROP ->
RELEASE_WAIT -> BACKUP -> FOLD_WAIT -> DONE) and is run once per robot by
the coordinator inside `main_loop`. Mission states sit on top:

```
WAIT_FOR_OBJECTS
       |
       v
ROBOT_1_PICK   ----->  HELPERS_TO_SINK  --->  WAIT_PLACE_THEN_GRASP
                                                       |
                                                       v
                                               SINK_GRASP
                                                       |
                                                       v
                                               SINK_LIFT
                                                       |
                                                       v
                                                  DONE
```

| Step | Action | Code |
|------|--------|------|
| 1 | Robot 1 drives to the blue cube, aligns `arm_link5` to the cube centre via TF, and picks the cube. | `update_robot_1()`: `IDLE -> ARM_TO_DRIVE -> APPROACH_CUBE -> FACE_CUBE -> PRE_PICK_ALIGN -> PICKUP -> PICKUP_WAIT`. |
| 2 | Robots 2 and 3 **simultaneously** drive to sink pre-pick standoffs, switch to the horizontal arm pose, and align `arm_link5` to the centre of their assigned red handle. They stop in `WAIT_FOR_SYNC` (do **not** pick yet). | `update_robot_2_or_3()`: `IDLE -> ARM_TO_DRIVE -> APPROACH_SINK -> FACE_HANDLE -> PRE_GRASP_ALIGN -> WAIT_FOR_SYNC`. |
| 3 | Robot 1 transports the cube to the sink basin, releases the cube on the rim, and folds its arm. | `update_robot_1()`: `DRIVE_TO_SINK -> FACE_SINK -> PLACE -> RELEASE_WAIT -> BACKUP -> FOLD_WAIT -> DONE`. |
| 4 | Robots 2 and 3 close the gripper on the handle, then perform a **synchronized** lift using `threading.Barrier(2)`. | `update_robot_2_or_3()`: `GRASP -> GRASP_WAIT -> LIFT -> HOLD`. |

Steps 1 and 2 run concurrently: the helpers start moving as soon as
the mission enters `ROBOT_1_PICK` (they don't wait for robot 1 to
finish). Steps 3 and 4 then sequence as in `vision_autopilot_simple.py`.

---

## 3. Grasp-pose calculation

All object poses come from Gazebo ground truth:

* The cube and sink have a `PosePublisher` plugin in their model SDF
  that emits `/model/<name>/pose` as `Pose_V`.
* The `ros_gz_bridge` in the launch file bridges each per-model pose
  topic into `/gz_pose_tf` and a `gazebo_pose_tf_relay` node filters
  that stream by `source_child` to publish `odom -> <name>` on `/tf`.
* The static landing pad is published with
  `tf2_ros.static_transform_publisher` on `/tf_static`.
* Every robot has its own `odom -> <name>_base_footprint` relay (same
  mechanism, but with a per-namespace child frame).

### 3.1 Blue-cube grasp

`cube_grasp_target()` reads the cube pose and returns its centre as
the TCP target for `arm_link5` (the gripper-pad midpoint).

* The robot drives to a `standoff_distance` (= `0.292 m`, the
  arm-joint5 forward reach at `REACH_DOWN`) from the cube centre, then
  faces the cube (`FACE_CUBE`).
* The arm lowers to `REACH_DOWN` and the chassis servos itself on the
  remaining `arm_link5 <-> cube-centre` error with TF-based
  fine alignment (`PRE_PICK_ALIGN`, tolerance 3 mm x 5 frames).
* `PICKUP` runs the manufacturer `grasp_and_lift()` macro:
  close gripper -> `LIFT_POSE` -> `CARRY`.
* `PICKUP_WAIT` verifies the cube has been lifted (`z > 0.10 m`); on
  failure it retries from `PRE_PICK_ALIGN`.

### 3.2 Drop-on-sink (the "yellow object")

`cube_grasp_target` is reused for the pickup. For the drop, the target
is computed from the live sink pose:

```python
def sink_drop_target(self):
    sink = self.get_tf_pose('sink')
    rim_z_world = sink.pose.position.z + SINK_BASIN_RIM_LOCAL_Y  # 0.05 m
    cube_half = CUBE_SIZE / 2.0
    return np.array([
        sink.pose.position.x,
        sink.pose.position.y,
        rim_z_world + cube_half,
    ])
```

`SINK_BASIN_RIM_LOCAL_Y` is the sink mesh's local Y of the basin rim
(0.05 m, taken directly from `meshes/sink/sink.obj`). With the
roll=pi/2 spawn orientation, local Y maps to world Z, so the rim is
at `sink_z + 0.05`. The cube centre is one cube-radius above that.

The robot drives to a `standoff_distance` (= 0.292 m) in front of the
sink (along the -X axis), faces the sink, and uses the manufacturer
`lower_and_release()` sequence to put the cube on the rim. After
release, `RELEASE_WAIT` verifies the cube is at the expected Z
(+/-3 cm); on failure it retries from `DRIVE_TO_SINK`.

### 3.3 Sink-handle grasp

The sink mesh is symmetric. The two red handles are located at the
following offsets in the sink model frame:

```
handle_minus_y (Robot 2 side) : local (0.0, 0.015, +0.17) -> world -Y
handle_plus_y  (Robot 3 side) : local (0.0, 0.015, -0.17) -> world +Y
```

The sign of the local Z offset is flipped relative to the world Y
because the sink is spawned with `roll=pi/2` (rotation around X maps
local Z to world -Y). These offsets are derived directly from the
`sink.obj` mesh and the comments in `models/sink/model.sdf`. They are
transformed into the world (odom) frame at runtime with
`transform_point_by_pose()` so the handle centres are always
recomputed from the live sink pose - the autopilot never uses
hard-coded world coordinates for the handles.

```python
def sink_handle_targets(self):
    sink = self.get_tf_pose('sink')
    h_minus_local = np.array([0.0, SINK_HANDLE_OFFSET_Y, +SINK_HANDLE_OFFSET_Z])
    h_plus_local  = np.array([0.0, SINK_HANDLE_OFFSET_Y, -SINK_HANDLE_OFFSET_Z])
    h_minus_world = transform_point_by_pose(h_minus_local, sink.pose)
    h_plus_world  = transform_point_by_pose(h_plus_local,  sink.pose)
    return h_minus_world, h_plus_world, sink
```

The base-standoff for each helper robot is computed from the world
bearing from the handle to the sink centre, so it works for any sink
orientation:

```python
def robot_base_target_for_handle(self, robot, handle_world, sink_pose):
    standoff = self.get_parameter('sink_standoff').value  # 0.32 m
    bearing = math.atan2(
        sink_pose.pose.position.y - handle_world[1],
        sink_pose.pose.position.x - handle_world[0],
    )
    bx = handle_world[0] - standoff * math.cos(bearing)
    by = handle_world[1] - standoff * math.sin(bearing)
    return bx, by, bearing
```

After reaching the standoff the robot turns to face the handle
(`FACE_HANDLE`), switches to the `HORIZONTAL_FORWARD` arm pose, and
serves itself on the residual `arm_link5 <-> handle-centre` error with
TF-based fine alignment (`PRE_GRASP_ALIGN`, tolerance 4 mm x 5
frames).

### 3.4 Gripper close policy (no clamp)

Per the spec, the gripper must not close more than 2.4 cm while picking.
The manufacturer REACH_DOWN pose puts the gripper pads at z~0.039 and
the parallel-linkage pads have a hard mechanical minimum of ~25 mm pad
gap.  So `GRIPPER_HOLD_CUBE = 0.0` (the joint's upper limit) holds the
pads at exactly the mechanical minimum — 25 mm gap, 1 cm wider than
the 2 cm cube.  The pads do NOT clamp the cube; the arm lifts it by
friction alone.  This is the closest the manufacturer geometry gets to
"close to 2.4 cm and stop".

The `grasp_and_lift()` macro first closes the gripper at REACH_DOWN
and **waits 0.6 s for the pads to settle** before lifting.  This
prevents the arm motion from coupling into the gripper pads and
shaking the cube off the alignment.

### 3.5 Gripper shake reduction

The `gripper_mimic_relay` is configured for minimum shake:

* Ramp rate 0.3 rad/s (full open->close ~5.2 s, slower than the arm
  trajectory bandwidth)
* Heavier low-pass on the measured master (alpha=0.15) so any
  joint_state noise is heavily damped before being fanned to the 5
  mimic fingers
* Settle deadband 0.005 rad — once the master is within this much
  of the target the relay stops pushing, eliminating the 1-2 px
  finger "buzz" in the wrist camera

### 3.4 Gripper finger spacing

The X3Plus parallel-linkage pads are ~25 mm apart at `grip_joint = 0`
(fully closed) and ~48 mm apart at `grip_joint = -0.676 rad`. The
`gripper_mimic_relay` clamps the pads to the physical contact as soon
as they touch the object, so the effective gripper gap equals the
object width when contact is made. This produces a secure hold with
no over-penetration into the collision geometry, which is exactly the
"object size - 2 mm" recommendation in the task spec.

| Object | Commanded `grip_joint` | Effective pad gap |
|--------|------------------------|-------------------|
| Blue cube (20 mm) | `-0.05` rad (~26 mm commanded) | 20 mm (clamped by relay) |
| Sink handle (40 mm x 70 mm bar, gripper pads close on 40 mm) | `-0.30` rad (~35 mm commanded) | 40 mm (clamped by relay) |

These are the same values as `vision_autopilot_simple.py`.

---

## 4. Dual-robot lift synchronization

After both helpers reach `WAIT_FOR_SYNC`, the coordinator waits for
robot 1 to finish placing the cube. When `r1.state == 'DONE'`, the
coordinator transitions both helpers to `GRASP` and monitors until
both reach `GRASP_WAIT`.

The barrier-based lift works as follows:

1. The coordinator creates a fresh `threading.Barrier(2,
   timeout=20.0)` and transitions both helpers from `GRASP_WAIT` to
   `LIFT`.

2. `update_robot_2_or_3` enters its `LIFT` branch, which calls
   `r.arm.run_async(r.arm.lift_horizontal_sync, self._lift_barrier)`.
   `run_async` starts a daemon thread per robot that immediately runs
   `lift_horizontal_sync(barrier)`.

3. Each thread first re-confirms the `HORIZONTAL_FORWARD` pose (so any
   drift since the alignment phase is reset) and then calls
   `barrier.wait()`. When both threads are blocked on the barrier,
   the barrier releases both of them simultaneously.

4. Both threads then run `set_joints(HORIZONTAL_CARRY,
   GRIPPER_HOLD_HANDLE, 3500)` in the **same simulation step**, so
   both arms rise at the same simulated time and the sink stays level.

5. The coordinator polls until both helpers reach `HOLD`, discards
   the barrier (`self._lift_barrier = None`) so the next run starts
   clean, and logs `DUAL-ROBOT SINK LIFT COMPLETE`.

If a thread never reaches the barrier (e.g., because of a Gazebo
crash), the 20 s barrier timeout fires and the helper falls back to a
solo lift to avoid deadlock, with a `BrokenBarrierError` log message.

---

## 5. Collision configuration

All manipulable objects use collision geometry that matches the visual
geometry exactly. This is the only way to keep dual-robot grasping
stable (any difference between visual and collision causes the
"phantom" gap or over-penetration that pops the sink out of one
gripper during the lift).

| Object | Visual | Collision | Notes |
|--------|--------|-----------|-------|
| `test_block` (blue cube) | 0.02 m x 0.02 m x 0.02 m box, blue diffuse | identical box | 5 g mass, friction `mu=100` so the cube does not slide in the gripper. |
| `sink` (`sink.obj`) | mesh, two materials (yellow basin + red handles) | identical mesh (no `<scale>`) | 2 kg mass (heavy enough to need two robots, light enough for two X3Plus grippers to lift together). |
| `landing_pad` | 500 mm x 500 mm x 2 mm box, green diffuse (alpha 0.6) | identical box | static, friction `mu=1`. |

All objects use ODE contact with `kp=1e5`, `kd=50`, and
`min_depth=0.001`, which gives stiff-but-not-jittery contact response
during grasping, transport, and the dual-robot lift.

---

## 6. Run instructions

```bash
# 1. Build the package (only needed once after changes)
cd /home/othman/ROS2CoorAPI/robot_workspace/x3plus_ws
colcon build --packages-select sim_gazebo_bringup
source install/setup.bash

# 2. Launch the full scene (Gazebo + 3 robots + autopilot)
ros2 launch sim_gazebo_bringup multi_robot_cube_sink.launch.py
# Optional: add gui:=false for headless mode
```

Once the scene is up, watch the logs for the following milestones:

```
[robot_1] IDLE -> ARM_TO_DRIVE
[robot_1] Cube lifted (z=0.xxx)
[robot_2] ... -> WAIT_FOR_SYNC
[robot_3] ... -> WAIT_FOR_SYNC
[robot_1] TASK COMPLETE
Both grippers closed; synchronous lift
[robot_2] Barrier released (party=0), lifting now
[robot_3] Barrier released (party=1), lifting now
=========================================
  DUAL-ROBOT SINK LIFT COMPLETE
=========================================
```

---

## 7. Comparison with `vision_autopilot_simple.py`

| Aspect | `vision_autopilot_simple.py` | This scene |
|--------|------------------------------|------------|
| World | `office.sdf` (ground + wall) | `multi_robot_scene.sdf` (ground + wall + 3 objects) |
| Spawn positions | 1 robot, spawn at origin | 3 robots in a straight line at `x=-1.5` |
| TF source | Ground truth from `PosePublisher` for cube and landing pad | Same mechanism + per-robot relays + static transform for landing pad |
| Arm poses | `HOME`, `DRIVE_POSE`, `REACH_DOWN`, `LIFT_POSE`, `CARRY`, `PLACE_DOWN` | Same + `HORIZONTAL_FORWARD`, `HORIZONTAL_CARRY` for sink grasp |
| Gripper values | `GRIPPER_HOLD = -0.05` (cube) | Same + `GRIPPER_HOLD_HANDLE = -0.30` |
| State machine | Single-robot linear FSM | Three FSMs (one per robot) coordinated by a mission state |
| Final alignment | TF-based `tf_final_align` | Same (used for both cube and handle) |
| Lifting synchronization | N/A (single robot) | `threading.Barrier(2)` between the two helper threads |
| Odometry use | Ground truth (`gazebo_pose_tf_relay`) | Same, namespaced per robot |
| Drop target | Green landing pad | Sink basin (the sink IS the yellow drop target) |

The state-machine structure (`IDLE -> ARM_TO_DRIVE -> APPROACH_* ->
FACE_* -> PRE_*_ALIGN -> *_PICK -> *_PICKUP_WAIT -> DRIVE_TO_* ->
FACE_* -> DROP/PLACE -> RELEASE_WAIT -> BACKUP -> FOLD_WAIT -> DONE`) is
identical; only the mission-level coordinator is added.
