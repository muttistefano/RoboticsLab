# Architecture

*How the pieces fit together, what lives where in the repository, and the limitations that are
there on purpose. Back to the [index](../README.md).*

## How it fits together

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

## Repository layout

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
│   ├── ur_controllers.yaml    ros2_control controller definitions, incl. the PID lab
│   ├── nav2_params_omni.yaml  the whole Nav2 stack
│   └── slam_toolbox.yaml · ekf.yaml · twist_mux.yaml · joy.yaml
├── worlds/warehouse.sdf       the environment
├── map/                       pre-built map for Nav2
└── lampo_description/nodo_prova.py    the worked example node
```

`PREFIX_` inside a config file is a sentinel: the launch files substitute the namespace into it at
runtime, which is how one config serves any number of robots.

## Known limitations

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
