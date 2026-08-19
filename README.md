# RoboticsLab

![LAMPO Robot Simulation](doc/post.png)

A teaching workspace for **ROS 2 Kilted** and **Gazebo Sim (gz-sim 9 / Ionic)**, built around
**LAMPO** — an omnidirectional mobile base that can also carry a UR arm, a Robotiq gripper and an
RGBD camera. It exists to give a full, honest robot software stack that runs on a laptop: URDF and
`xacro`, `ros2_control`, the ROS ↔ Gazebo bridge, TF, namespacing for multi-robot, and the Nav2
navigation stack.

---

## 1. Prerequisites

- Ubuntu 24.04 with **ROS 2 Kilted**
- Gazebo Sim 9 (installed as `ros-kilted-ros-gz-sim`, not as a standalone `gz` package)

Install the dependencies this workspace needs:

```bash
sudo apt update
sudo apt install -y \
  ros-kilted-ros-gz-sim ros-kilted-ros-gz-bridge ros-kilted-ros-gz-image \
  ros-kilted-gz-ros2-control ros-kilted-controller-manager \
  ros-kilted-joint-state-broadcaster ros-kilted-position-controllers \
  ros-kilted-joint-trajectory-controller ros-kilted-velocity-controllers \
  ros-kilted-ur-description ros-kilted-ur-simulation-gz ros-kilted-ur-controllers \
  ros-kilted-robotiq-description \
  ros-kilted-navigation2 ros-kilted-nav2-bringup ros-kilted-slam-toolbox \
  ros-kilted-robot-localization ros-kilted-twist-mux \
  ros-kilted-joy ros-kilted-teleop-twist-joy \
  ros-kilted-rviz2 ros-kilted-topic-tools ros-kilted-tf2-tools
```

Or let `rosdep` work it out from `package.xml`:

```bash
cd ros_ws
rosdep install --from-paths src --ignore-src -r -y
```

> **`gz` is not on your PATH until you source ROS.** The binary ships inside the ROS vendor tree
> (`/opt/ros/kilted/opt/gz_tools_vendor/bin/gz`), and it needs `GZ_CONFIG_PATH` set. If `gz sim
> --versions` says *"I cannot find any available 'gz' command"*, you have not sourced
> `/opt/ros/kilted/setup.bash`.

> **If you have more than one ROS distro installed** (e.g. Jazzy *and* Kilted), be careful which one
> you source. Jazzy ships gz-sim **8**; this workspace targets gz-sim **9**.

## 2. Build

```bash
source /opt/ros/kilted/setup.bash
cd ros_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` symlinks Python, launch and config files into `install/`, so edits to them take
effect without rebuilding. Changes to `package.xml`, `CMakeLists.txt` or the URDF still need a
rebuild.

**Every new terminal needs both `source` lines.** The workflow below uses three terminals.

## 3. Run

### Terminal 1 — the world

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

### Terminal 2 — a robot

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
| `gripper_control` | `false` | expose the gripper through `ros2_control` — **experimental**, see *Known limitations* |

> **The namespace must end with a separator.** Every link and joint is named `${namespace}<name>`,
> so `r2_` gives `r2_base_footprint` while a bare `r2` would give `r2base_footprint`. Use a
> trailing underscore.

### Terminal 3 — navigation

```bash
ros2 launch lampo_description lampo_nav_omni.launch.py

# match the namespace AND the spawn pose used above
ros2 launch lampo_description lampo_nav_omni.launch.py namespace:=r2_ x:=-2.0 y:=1.0
```

Brings up Nav2 against `map/map.yaml`. AMCL is seeded from the same `x`/`y`/`yaw` you spawned the
robot at, so pass them if you changed them. Then use **2D Goal Pose** in RViz.

### Optional — joystick teleoperation, arbitrated with `twist_mux`

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

### Optional — SLAM: build your own map

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

> **Do not run SLAM and `lampo_nav_omni.launch.py` at the same time.** `slam_toolbox` and AMCL both
> publish the `map → odom` transform, and two publishers of the same transform is a fight neither
> wins. SLAM *builds* a map; AMCL *localises against* one already built.

### Optional — sensor fusion with an EKF

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

### Optional — the worked example node

```bash
ros2 run lampo_description nodo_prova.py --ros-args -r __ns:=/r1_ -p use_sim_time:=true
```

A small closed-loop controller that drives the robot to `x = 0` using odometry feedback. It is
deliberately minimal and heavily commented — a good starting point for an exercise.

---

## 4. How it fits together

```
                    Gazebo Sim (gz-sim 9)
       physics · sensors · MecanumDrive · gz_ros2_control
                            |
                     ros_gz_bridge          <- config/bridge.yaml
                            |
   /clock  /r1_/odom  /r1_/lidar  /r1_/imu  /r1_/joint_states  /r1_/tf
                            |
    robot_state_publisher ---+--- Nav2 ---+--- controller_manager
        (URDF -> TF)             (AMCL,   |    (joint_state_broadcaster,
                                  costmaps,     forward_position_controller)
                                  planner,
                                  MPPI)
```

Everything the robot publishes is namespaced, including both transform topics, and both are then
relayed onto the global `/tf` and `/tf_static` so that RViz and `tf2_tools` see one merged tree.
Every frame carries the `r1_` prefix, so any number of robots can share it.

The namespacing is not cosmetic. Nav2's servers are composable nodes launched by `nav2_bringup`,
which remaps `/tf_static` to `<ns>/tf_static`. Publish the static transforms only on the global
topic and that namespaced one has no publisher at all: AMCL never learns
`base_footprint → front_laser`, drops every scan, and never emits `map → odom`. Nav2 still reports
every server *active*, so the stack looks healthy while the robot simply never localises.
`topic_tools/relay` matches the source publisher's QoS, so the relayed `/tf_static` keeps its
`TRANSIENT_LOCAL` durability and late joiners — an RViz restart — are still served.

## 5. Repository layout

```
ros_ws/src/lampo_description/
├── urdf/
│   ├── system.urdf.xacro      entry point; composes everything by xacro:if
│   ├── sweepee/               the mobile base: sweepee_omni (mecanum) / sweepee_diff
│   ├── mm_arm.xacro           UR20 + the gz_ros2_control plugin
│   ├── mm_gripper.xacro       Robotiq 2F-85
│   └── mm_camera.xacro        RGBD camera + its REP-103 optical frame
├── launch/
│   ├── lampo_sandbox.launch.py    world + clock bridge + RViz
│   ├── lampo_gz_mm.launch.py      one robot: RSP, spawn, bridge, controllers
│   ├── lampo_nav_omni.launch.py   Nav2 against a known map
│   ├── lampo_slam.launch.py       slam_toolbox (+ optional EKF)
│   └── lampo_joy.launch.py        joystick + twist_mux
├── config/
│   ├── bridge.yaml            which topics cross the ROS ↔ gz boundary
│   ├── ur_controllers.yaml    ros2_control controller definitions
│   ├── nav2_params_omni.yaml  the whole Nav2 stack
│   └── slam_toolbox.yaml · ekf.yaml · twist_mux.yaml · joy.yaml
├── worlds/warehouse.sdf       the environment
├── map/                       pre-built map for Nav2
└── lampo_description/nodo_prova.py    the worked example node
```

`PREFIX_` inside a config file is a sentinel: the launch files substitute the namespace into it at
runtime, which is how one config serves any number of robots.

## 6. Known limitations

- **The gripper is passive.** Its `ros2_control` block is disabled by default
  (`gripper_control:=true` to enable). The Robotiq 2F-85 fingers form a closed 4-bar linkage, which
  URDF cannot express — it is approximated with `<mimic>` joints. DART cannot create mimic
  constraints at all, and Bullet Featherstone over-constrains the loop and drives the knuckle joint
  outside its limits. `ros2_control`'s joint limiter treats that as a hardware fault and throws a
  **fatal** exception, aborting the simulator. See the comment in `urdf/mm_gripper.xacro`.
- The lidar is a forward-facing ±80° scanner, not a 360° one. AMCL works, but localization is
  weaker than a full scan would give.
- `worlds/depot/` carries ~85 MB of textures, most of which the world never references.
- The lidar is forward-facing, so `slam_toolbox` maps what is in front of the robot. Drive it
  around the aisles rather than spinning on the spot.
- **Nav2 is CPU-hungry, and that is the most likely reason a goal fails.** MPPI evaluates
  `batch_size × time_steps` (2000 × 56) trajectories every control cycle at 20 Hz. On a machine also
  running Gazebo, a 640-beam lidar and an RGBD camera, the control loop can fall to ~5 Hz; the
  progress checker then sees the robot barely moving and aborts with *Failed to make progress*.
  Watch for *Control loop missed its desired rate* in the log. Close other work, or run the
  simulator headless with `gui:=false` — detuning MPPI was tried and made goal-reaching worse, since
  `model_dt` must track the controller period and that lengthens the prediction horizon.

## 7. Tests

```bash
cd ros_ws
colcon test --packages-select lampo_description
colcon test-result --all
```

The default suite needs no simulator and finishes in seconds. It renders the robot in all four
configurations, checks the URDF for structural and physical sanity, and — most usefully — asserts
that the configs, the launch files and *this README* still agree with each other. A documented
argument that no longer exists is a test failure rather than a surprise during a demo.

The simulator-in-the-loop tests are opt-in, because they need Gazebo and a GPU:

```bash
colcon build --packages-select lampo_description --cmake-args -DBUILD_SIM_TESTS=ON
colcon test --packages-select lampo_description
```

Those launch a headless Gazebo and check the things only a running system can show: sensors
publishing, the TF tree resolving, controllers activating in order, the arm reaching a commanded
pose, Nav2 localising and driving to a goal, `slam_toolbox` building a map, `twist_mux` letting the
joystick override navigation, the EKF producing a fused estimate, and two robots sharing one world
without colliding in the ROS graph.

Together: 54 fast tests and 26 simulator tests.

## 8. Windows — devcontainer with VcXsrv

Linux and macOS users can open the devcontainer directly; `DISPLAY` and the X11 socket are passed
through automatically.

1. Install [VSCode](https://code.visualstudio.com/),
   [Docker Desktop](https://www.docker.com/products/docker-desktop),
   the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers),
   and [VcXsrv](https://sourceforge.net/projects/vcxsrv/).
2. **Configure VcXsrv:** launch XLaunch → "Multiple windows", display number `0` → "Start no
   client" → tick **Disable access control**. Save the config.
3. **Firewall:** allow VcXsrv through Windows Defender on both private and public networks.
4. **Point the container at your X server:** in `.devcontainer/devcontainer.json`, set
   `"DISPLAY": "host.docker.internal:0"`.
5. `F1` → *Dev Containers: Reopen in Container*, and wait for the build.
6. Build and run as in sections 2–3. Gazebo and RViz appear on your Windows desktop.

### Troubleshooting

- **No window appears** — VcXsrv running, with *Disable access control* ticked?
- **Connection refused** — check the firewall rule for VcXsrv.
- **Wrong display** — the number in `DISPLAY` must match VcXsrv's (normally `:0`).
- **Gazebo is very slow** — you are almost certainly rendering on the wrong GPU. On an NVIDIA
  Optimus laptop the X server drives the Intel iGPU, so Gazebo's sensors render there or fall back
  to software. Check with:

  ```bash
  glxinfo | grep "OpenGL renderer"        # names the GPU actually in use
  nvidia-smi                              # a near-idle discrete GPU is the tell
  ```

  `lampo_sandbox.launch.py` sets the PRIME offload variables
  (`__NV_PRIME_RENDER_OFFLOAD=1`, `__GLX_VENDOR_LIBRARY_NAME=nvidia`) for exactly this reason; they
  are ignored on machines without an NVIDIA driver. This matters beyond frame rate: starved sensor
  rendering steals the CPU that Nav2's control loop needs, and goals start failing with *Failed to
  make progress*. On Windows, expect llvmpipe and use `gui:=false` where you can.
