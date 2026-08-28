cmake_minimum_required(VERSION 3.8)
project(lampo_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Resource directories consumed at runtime by the launch files.
install(
  DIRECTORY launch meshes urdf rviz map worlds config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

# Install the Python module so `import lampo_description` works.
ament_python_install_package(${PROJECT_NAME})

# Install runnable nodes into lib/<pkg> so `ros2 run` finds them.
install(PROGRAMS
  lampo_description/nodo_prova.py
  lampo_description/zero_g.py
  lampo_description/joint_observer.py
  lampo_description/lqr_joint.py
  lampo_description/sensor_noise.py
  DESTINATION lib/${PROJECT_NAME}
)

# Simulator-in-the-loop tests need Gazebo and a GPU, and take minutes rather
# than seconds. They are off by default so `colcon test` stays fast and passes
# in the devcontainer, where rendering is software-only. Enable with:
#   colcon build --cmake-args -DBUILD_SIM_TESTS=ON
option(BUILD_SIM_TESTS "Build the tests that launch Gazebo" OFF)

if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_flake8()
  ament_xmllint()
  ament_lint_cmake()
  # D213 wants the docstring summary on the second line, which contradicts
  # D212 -- the style every docstring in this package uses. Ignore the loser.
  ament_pep257(--add-ignore D213)
  # ament_copyright is deliberately not run: package.xml declares Apache-2.0
  # but the repo ships no LICENSE file and no per-file headers. Add both, then
  # enable it.

  # Fast tier: no simulator, no GPU, runs in seconds.
  ament_add_pytest_test(unit_tests test
    TIMEOUT 180
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )

  if(BUILD_SIM_TESTS)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/integration/test_sandbox.py TIMEOUT 180)
    add_launch_test(test/integration/test_robot.py TIMEOUT 300)
    add_launch_test(test/integration/test_arm_pid.py TIMEOUT 300)
    add_launch_test(test/integration/test_servo.py TIMEOUT 300)
    add_launch_test(test/integration/test_dynamics.py TIMEOUT 300)
    add_launch_test(test/integration/test_lqr.py TIMEOUT 300)
    add_launch_test(test/integration/test_estimation.py TIMEOUT 300)
    add_launch_test(test/integration/test_nav2.py TIMEOUT 420)
    add_launch_test(test/integration/test_slam.py TIMEOUT 300)
    add_launch_test(test/integration/test_ekf.py TIMEOUT 300)
    add_launch_test(test/integration/test_multirobot.py TIMEOUT 360)
    # Everything in a test runs on one machine, so pin both middlewares to
    # the local host. An active VPN otherwise swallows the discovery
    # multicast of gz-transport AND DDS with no error anywhere: the spawner
    # hangs at "Requesting list of world names" and subscribers never find
    # their publishers. This must be process-level environment -- launch
    # snapshots its environment at startup, so setting os.environ from
    # inside the test file is already too late.
    set_tests_properties(
      test_integration_test_sandbox.py
      test_integration_test_robot.py
      test_integration_test_arm_pid.py
      test_integration_test_servo.py
      test_integration_test_dynamics.py
      test_integration_test_lqr.py
      test_integration_test_estimation.py
      test_integration_test_nav2.py
      test_integration_test_slam.py
      test_integration_test_ekf.py
      test_integration_test_multirobot.py
      PROPERTIES ENVIRONMENT
      "GZ_IP=127.0.0.1;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
  endif()
endif()

ament_package()
