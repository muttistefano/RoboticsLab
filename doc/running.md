# Running the sandbox

*Every launch file, what it starts, and every argument it takes. The three-terminal workflow is
the happy path; the optional sections build on it. Back to the [index](../README.md).*

Every terminal needs the two `source` lines from [install.md](install.md) first.

## Terminal 1 — the world

```bash
ros2 launch lampo_description lampo_sandbox.launch.py
```

Starts Gazebo with the warehouse world, the `/clock` bridge, and RViz.

| argument | default | meaning |
|---|---|---|
| `world` | `worlds/warehouse.sdf` | SDF world to load |
| `gui` | `true` | `false` runs the server headless — much faster |
| `rviz` | `true` | start RViz alongside |
| `verbosity` | `3` | `gz sim` log level, 0–4 |

To use a different world, download one from [Gazebo Fuel](https://app.gazebosim.org/fuel) and pass
`world:=/absolute/path/to/world.sdf`.

## Terminal 2 — a robot

```bash
# omnidirectional base only
ros2 launch lampo_description lampo_gz_mm.launch.py

# full mobile manipulator: UR arm + gripper + RGBD camera
ros2 launch lampo_description lampo_gz_mm.launch.py mm:=true

# a second robot, in its own namespace
ros2 launch lampo_description lampo_gz_mm.launch.py namespace:=r2_ x:=-2.0 y:=1.0
```

This starts `robot_state_publisher`, spawns the model into the running Gazebo, brings up the
ROS ↔ gz bridge, and activates the controllers **in order** using launch event handlers.

| argument | default | meaning |
|---|---|---|
| `namespace` | `r1_` | namespace **and** TF prefix — see the note below |
| `mm` | `false` | `true` adds the UR arm, gripper and camera |
| `omni` | `true` | `false` gives the differential-drive base |
| `x`, `y`, `z`, `yaw` | `-3.5, 2.2, 0.2, 0.3` | spawn pose |
| `arm_controller` | `forward` | `pid` / `effort_pid` swap in a tunable PID, `servo` adds Cartesian jogging, `effort` is the raw-torque mode for the dynamics labs — see [control.md](control.md) |
| `gripper_control` | `false` | expose the gripper through `ros2_control` — **experimental**, see *Known limitations* in [architecture.md](architecture.md) |

> **The namespace must end with a separator.** Every link and joint is named `${namespace}<name>`,
> so `r2_` gives `r2_base_footprint` while a bare `r2` would give `r2base_footprint`. Use a
> trailing underscore.

## Terminal 3 — navigation

```bash
ros2 launch lampo_description lampo_nav_omni.launch.py

# match the namespace AND the spawn pose used above
ros2 launch lampo_description lampo_nav_omni.launch.py namespace:=r2_ x:=-2.0 y:=1.0
```

Brings up Nav2 against `map/map.yaml`. AMCL is seeded from the same `x`/`y`/`yaw` you spawned the
robot at, so pass them if you changed them. Then use **2D Goal Pose** in RViz.

## Optional — joystick teleoperation, arbitrated with `twist_mux`

Two things want to drive the robot: you, and Nav2. Without an arbiter they publish to the same
topic and fight. `twist_mux` subscribes to both and forwards whichever source has the highest
priority *and* is currently publishing:

```
joystick  --(priority 100)--\
                             twist_mux --> <ns>/cmd_vel_safe --> Gazebo
Nav2      --(priority  10)--/
```

Grabbing the joystick therefore overrides autonomy immediately, and releasing it hands control back
after the 0.5 s timeout — the same pattern real robots use for a manual-override.

```bash
ros2 launch lampo_description lampo_joy.launch.py

# Nav2 must publish to cmd_vel_nav instead of driving the robot directly,
# otherwise it bypasses the mux entirely:
ros2 launch lampo_description lampo_nav_omni.launch.py cmd_vel_topic:=cmd_vel_nav
```

| argument | default | meaning |
|---|---|---|
| `namespace` | `r1_` | robot to drive; must match the spawn namespace |
| `joy_id` | `0` | joystick index, i.e. `/dev/input/js<N>` |

Priorities and timeouts live in `config/twist_mux.yaml`; the axis mapping is in `config/joy.yaml`.

## Optional — SLAM: build your own map

`map/map.yaml` ships pre-made, but nothing explains where it came from. This builds it live:

```bash
# T1 world, T2 robot -- as above, then:
ros2 launch lampo_description lampo_slam.launch.py
ros2 launch lampo_description lampo_joy.launch.py     # drive it around
```

Set RViz's fixed frame to `map` and watch the occupancy grid fill in. When you are happy with it:

```bash
ros2 service call /r1_/slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: my_map}}"
```

Then point Nav2 at the result: `lampo_nav_omni.launch.py map:=/path/to/my_map.yaml`.

| argument | default | meaning |
|---|---|---|
| `namespace` | `r1_` | robot to map with |
| `use_ekf` | `false` | also run a `robot_localization` EKF — see below |
| `noisy` | `false` | feed the EKF through `sensor_noise.py` — the Kalman exercise in [control.md](control.md) |

> **Do not run SLAM and `lampo_nav_omni.launch.py` at the same time.** `slam_toolbox` and AMCL both
> publish the `map → odom` transform, and two publishers of the same transform is a fight neither
> wins. SLAM *builds* a map; AMCL *localises against* one already built.

## Optional — sensor fusion with an EKF

```bash
ros2 launch lampo_description lampo_slam.launch.py use_ekf:=true
```

The drive plugin already publishes `odom → base_footprint`, but that transform is **ground truth
from the simulator** — a real robot has no such thing. The `robot_localization` EKF
(`config/ekf.yaml`) instead consumes exactly what real hardware produces, a noisy
`nav_msgs/Odometry` and a `sensor_msgs/Imu`, and estimates the transform itself. It is the honest
version of the same signal, and it gives the otherwise-unused IMU a consumer.

It is off by default because enabling it gives you *two* publishers of `odom → base_footprint`; to
use it properly, remove the `<tf_topic>` from the drive plugin in `urdf/sweepee/sweepee_omni.xacro`
first. Comparing the two is itself the exercise.

## Optional — the control exercises

```bash
ros2 launch lampo_description lampo_gz_mm.launch.py mm:=true arm_controller:=pid
```

Replaces the arm's forward position controller with a PID controller whose gains you tune live
while the arm follows step references; `arm_controller:=servo` instead puts MoveIt Servo in
front of the arm for Cartesian jogging. The full workflow — the PID modes, the Cartesian mode,
what to plot, and the exercises built on them — is in [control.md](control.md).

## Optional — the worked example node

```bash
ros2 run lampo_description nodo_prova.py --ros-args -r __ns:=/r1_ -p use_sim_time:=true
```

A small closed-loop controller that drives the robot to `x = 0` using odometry feedback. It is
deliberately minimal and heavily commented — a good starting point for an exercise.
