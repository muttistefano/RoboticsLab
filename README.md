# RoboticsLab

![LAMPO Robot Simulation](doc/post.png)

**One sandbox, every layer of a real ROS 2 stack.** RoboticsLab is a teaching workspace for
**ROS 2 Kilted** and **Gazebo Sim 9**, built around **LAMPO** — an omnidirectional mobile base
that can also carry a UR20 arm, a Robotiq gripper and an RGBD camera. Instead of learning TF from
one tutorial, Nav2 from another and `ros2_control` from a third, students work one robot in one
warehouse through the whole stack — description, control, navigation, mapping, state estimation,
multi-robot — and because every layer is the real component, what they learn moves to hardware
unchanged.

## Quickstart

```bash
# build (once) — full setup in doc/install.md
cd ros_ws && colcon build --symlink-install && source install/setup.bash

# terminal 1: the warehouse world
ros2 launch lampo_description lampo_sandbox.launch.py

# terminal 2: a robot — add mm:=true for the arm, camera and gripper
ros2 launch lampo_description lampo_gz_mm.launch.py
```

From there: drive it, navigate it, map with it, tune a PID on it — the documentation below takes
over.

## Documentation

| read this | to |
|---|---|
| [Install](doc/install.md) | set up Ubuntu 24.04 + ROS 2 Kilted, build the workspace, or use the Windows devcontainer with VcXsrv |
| [Running](doc/running.md) | launch everything — the world, robots, Nav2, joystick teleop, SLAM, the EKF — with every launch argument explained |
| [Control exercises](doc/control.md) | tune real PID gains live on the arm: step responses, gravity, integral windup — and the proposed future labs |
| [Architecture](doc/architecture.md) | understand how Gazebo, the bridge, TF, namespacing and Nav2 fit together, the repo layout, and the known limitations |
| [Testing](doc/testing.md) | run the fast self-checking suite and the opt-in simulator-in-the-loop tests |
| [Demo run sheet](DEMO.md) | present it: a timed 20-minute script with recovery cheatsheet |

## What's inside

- **Four robot configurations** from one description: mecanum or differential base, with or
  without the UR20 arm (`lampo_gz_mm.launch.py`, composed by xacro)
- **Three onboard sensors** — 2D lidar, IMU, RGBD camera — bridged into ROS with `ros_gz_bridge`
- **Autonomous navigation** with the full Nav2 stack: AMCL, costmaps, MPPI, collision monitor
  (`lampo_nav_omni.launch.py`)
- **Mapping** with `slam_toolbox`, live, while you drive (`lampo_slam.launch.py`)
- **Joystick teleop arbitrated with `twist_mux`** — manual override beats autonomy, like on a
  real robot (`lampo_joy.launch.py`)
- **Sensor fusion** with a `robot_localization` EKF, fed the noisy signals real hardware produces
- **A control lab**: chainable `ros2_control` PID controllers on the arm, with gains you tune at
  runtime
- **Multi-robot** by namespacing alone — a second robot is the same command with a different name
- **A self-checking test suite** where even these documents are executable promises

Everything is namespaced (`r1_` by default), every config uses one `PREFIX_` sentinel, and the
same `ros2_control` controllers that drive the simulated UR20 drive a real one.

## Layout

```
README.md · DEMO.md · doc/        you are here
ros_ws/src/lampo_description/     the single ROS 2 package: urdf/, launch/,
                                  config/, worlds/, map/, test/
.devcontainer/                    Docker + VcXsrv setup for Windows
```

License: Apache-2.0.
