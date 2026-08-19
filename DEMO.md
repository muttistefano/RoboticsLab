# RoboticsLab — Demo Run Sheet

**Slot:** ~20 minutes total. Talk ≈ 13 min, live demo ≈ 7 min.
**Audience:** robotics-literate, new to ROS 2.

Every block below has been executed on the presentation machine, and each is covered by an
automated test in `ros_ws/src/lampo_description/test/integration/`.

---

## Pre-flight (run 30 minutes before, not 5)

```bash
# 1. Clean build from scratch -- catches a stale install/ more than anything else
cd ~/RoboticsLab/ros_ws
rm -rf build install log
source /opt/ros/kilted/setup.bash
colcon build --symlink-install          # ~5 s
source install/setup.bash

# 2. Confirm the toolchain is the one you think it is
gz sim --versions                       # must print 9.x
glxinfo | grep 'OpenGL renderer'        # must name the discrete GPU, not the iGPU
echo $ROS_DISTRO                        # must print kilted
ros2 pkg prefix nav2_bringup            # must NOT error

# 3. Render the robot without launching anything
xacro src/lampo_description/urdf/system.urdf.xacro \
      prefix:=r1_ omni:=true mm:=true | check_urdf /dev/stdin | head -3

# 4. One full dry run of the demo below, then close everything.
```

> **Verified.** Nav2 localises at the spawn pose and drives to a goal; `slam_toolbox` builds a map;
> `twist_mux` gives the joystick priority over navigation. Bringing these up for the first time
> exposed four real defects — namespaced `/tf_static`, parameter files keyed on a bare node name,
> `slam_toolbox` needing a lifecycle manager, and its hardcoded `/map` topic — all fixed, each with
> a regression test. Re-run the suite before the talk anyway:
>
> ```bash
> cd ros_ws && colcon test --packages-select lampo_description && colcon test-result --all
> ```

**Checklist**

- [ ] Clean build succeeds
- [ ] `colcon test` green (and the sim tier too, if you have time)
- [ ] All three terminals pre-sourced, commands typed but **not** entered
- [ ] Screen resolution set; font size bumped for the back of the room
- [ ] Screen recordings of each block available as a fallback
- [ ] `htop` closed, notifications off, laptop on mains power
- [ ] Joystick plugged in *if* you plan to show teleop

**Terminal prologue** — every terminal starts with:

```bash
source /opt/ros/kilted/setup.bash && source ~/RoboticsLab/ros_ws/install/setup.bash
```

---

## Block 1 — The world (≈ 1 min)

**T1:**
```bash
ros2 launch lampo_description lampo_sandbox.launch.py
```

**Expect:** Gazebo opens with the warehouse; RViz opens showing an empty grid and a
`Fixed Frame [map] does not exist` warning.

**Say:**
> "One command starts the simulator, the clock bridge and the visualiser. Note RViz is complaining
> there is no `map` frame — that is correct, nothing is publishing one yet. We will fix that by
> starting the localisation stack, not by editing a config."

**If it fails:** the usual cause is an unsourced terminal. Second most likely: another Gazebo still
running — `pkill -f 'gz sim'`.

---

## Block 2 — A robot, and what ROS 2 gives you for free (≈ 3 min)

**T2:**
```bash
ros2 launch lampo_description lampo_gz_mm.launch.py mm:=true
```

**Expect:** the mobile manipulator appears in Gazebo. In the launch log, the controllers come up
**in order**: `joint_state_broadcaster` → `forward_position_controller`.

**Say:**
> "That ordering is not luck. The controller manager only exists after Gazebo has loaded the model,
> so the launch file chains these with event handlers — each waits for the previous process to exit.
> The version of this repo I started from fired them all after a five-second sleep and hoped."

Now the introspection beat — this is the part that lands with people new to ROS 2:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /r1_/odom --once
ros2 topic hz /r1_/lidar
ros2 run tf2_tools view_frames          # writes frames.pdf
```

**Say:**
> "I have not written a single line of code to inspect this robot. The graph is introspectable by
> construction — every node, every topic, the full transform tree. That is the actual argument for
> ROS 2, more than any individual library."

Then show the arm move — the `ros2_control` beat:

```bash
ros2 topic pub --once /r1_/forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [1.0, -1.2, 1.0, -1.4, -1.57, 0.0]}"
```

**Expect:** the UR arm moves to that configuration and holds it.

**Say:**
> "Those are joint position commands going through `ros2_control`. The same controller, with the
> same interface, drives a real UR — you swap the hardware plugin, not the control code. That is
> the abstraction that makes simulation worth doing."

---

## Block 3 — Autonomy (≈ 3 min)

**T3:**
```bash
ros2 launch lampo_description lampo_nav_omni.launch.py
```

**Expect:** Nav2 servers come up, the map appears in RViz, the robot localises near its spawn pose,
costmaps render.

**Do:** click **2D Goal Pose** in RViz, put a goal across the warehouse. The robot plans and drives.

**Say:**
> "AMCL is seeded from the same pose the robot was spawned at — those used to be two independent
> hardcoded numbers two metres apart, which is a wonderful way to spend an afternoon wondering why
> localisation is broken. The global plan comes from a planner plugin, the velocities from an MPPI
> controller, and everything you see is a lifecycle-managed node that Nav2 brought up in order."

**Fallback if localisation drifts:** use **2D Pose Estimate** in RViz to reseed, then re-issue the
goal. (The tool used to publish to the wrong namespace — it works now.)

---

## Block 4 — Multi-robot, if time allows (≈ 1 min)

**T4:**
```bash
ros2 launch lampo_description lampo_gz_mm.launch.py namespace:=r2_ x:=-2.0 y:=1.0
```

**Say:**
> "Same launch file, different namespace. Every topic, every frame and every controller is prefixed.
> This is how ROS 2 handles multi-robot — not a special mode, just names."

---

## Block 5 — SLAM, if the audience asks "where did the map come from?" (≈ 2 min)

Only run this **instead of** block 3, never alongside it — `slam_toolbox` and AMCL both publish
`map → odom` and would fight. Kill T3 first.

**T3:**
```bash
ros2 launch lampo_description lampo_slam.launch.py
```
**T4:**
```bash
ros2 launch lampo_description lampo_joy.launch.py
```

**Expect:** RViz's occupancy grid fills in as you drive; previously-unseen aisles appear.

**Say:**
> "Block 3 localised against a map that was already there. This is where that map comes from — the
> same lidar, the same TF tree, a different algorithm. Nav2 answers *where am I on this map*; SLAM
> answers *what does the map look like*. And notice I am driving with a joystick while the mapper
> runs: `twist_mux` arbitrates between me and autonomy by priority, so grabbing the stick overrides
> the planner and letting go hands it back. That is the manual-override every real robot needs."

**Then save it, live:**
```bash
ros2 service call /r1_/slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: my_map}}"
```

**If it fails:** check `ros2 topic hz /r1_/lidar` — no scan, no map.

---

## Optional extras (only if you are ahead of schedule)

```bash
# record and replay -- the "I can debug this on a train" beat
ros2 bag record -o demo /r1_/odom /r1_/lidar /tf /tf_static

# fuse wheel odometry with the IMU instead of trusting simulator ground truth
ros2 launch lampo_description lampo_slam.launch.py use_ekf:=true

# the worked example node
ros2 run lampo_description nodo_prova.py --ros-args -r __ns:=/r1_ -p use_sim_time:=true
```

---

## Shutdown

`Ctrl-C` each terminal in reverse order (T3 → T1), then **always** run the cleanup below.

`gz sim` is started through a Ruby wrapper. `Ctrl-C` kills the wrapper, but the simulator itself is
a child process that frequently survives — verified repeatedly while testing this repo. A surviving
server keeps publishing `/clock` and the old robot's topics, so the *next* launch appears to work
and then behaves inexplicably. This is the single most likely way to lose a live demo.

```bash
pkill -f 'gz sim'; pkill -f parameter_bridge; pkill -f robot_state_publisher
pgrep -af 'gz sim'        # must print nothing before you start the next block
```

---

## Recovery cheatsheet

| Symptom | Cause | Fix |
|---|---|---|
| `ros2: command not found` | terminal not sourced | run the prologue |
| `gz: command not found` | ditto — `gz` lives in the ROS vendor tree | run the prologue |
| World loads empty / grey | `GZ_SIM_RESOURCE_PATH` not set | use the launch file, not `gz sim <file>` directly |
| Robot spawns but no TF | stale Gazebo from a previous run | `pkill -f 'gz sim'`, restart T1 |
| Second run behaves oddly | orphaned `gz sim` survived Ctrl-C | `pgrep -af 'gz sim'`; kill it, relaunch |
| Nav2 spams "extrapolation into the future" | a node on wall clock | check `ros2 param get /r1_/controller_server use_sim_time` |
| Everything is 5 fps | software rendering | check `/dev/dri` is available; use `gui:=false` |
| "Failed to make progress" | control loop starved of CPU | close other work; check the log for `Control loop missed its desired rate` |
| Robot drives through a wall | old world file | walls have collision geometry now — rebuild |
| Map never appears in SLAM | no scan reaching `slam_toolbox` | `ros2 topic hz /r1_/lidar` |
| Robot ignores the joystick | Nav2 bypassing the mux | relaunch Nav2 with `cmd_vel_topic:=cmd_vel_nav` |
| Map flickers / TF fights | SLAM and Nav2 both running | run one or the other, never both |

---

## Timing summary

| Block | Target | Cumulative |
|---|---|---|
| 1 — world | 1:00 | 1:00 |
| 2 — robot + introspection + arm | 3:00 | 4:00 |
| 3 — navigation | 3:00 | 7:00 |
| 4 — multi-robot (optional) | 1:00 | 8:00 |
| 5 — SLAM (alternative to 3) | 2:00 | — |

Budget 7 minutes. Block 4 is the one to drop if you are running long; block 5 is a *substitute* for
block 3, not an addition to it.
