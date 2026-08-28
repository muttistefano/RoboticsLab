# Testing

*What the test suite checks, and how to run it — including the opt-in tests that launch a real
Gazebo. Back to the [index](../README.md).*

```bash
cd ros_ws
colcon test --packages-select lampo_description
colcon test-result --all
```

The default suite needs no simulator and finishes in seconds. It renders the robot in all four
configurations, checks the URDF for structural and physical sanity, and — most usefully — asserts
that the configs, the launch files and *these documents* still agree with each other. A documented
argument that no longer exists is a test failure rather than a surprise during a demo.

The simulator-in-the-loop tests are opt-in, because they need Gazebo and a GPU:

```bash
colcon build --packages-select lampo_description --cmake-args -DBUILD_SIM_TESTS=ON
colcon test --packages-select lampo_description
```

Those launch a headless Gazebo and check the things only a running system can show: sensors
publishing, the TF tree resolving, controllers activating in order, the arm reaching a commanded
pose, the PID lab controller converging on a step reference, Nav2 localising and driving to a
goal, `slam_toolbox` building a map, `twist_mux` letting the joystick override navigation, the EKF
producing a fused estimate, and two robots sharing one world without colliding in the ROS graph.

Together: 81 fast tests and 41 simulator tests.
