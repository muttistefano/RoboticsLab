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
    add_launch_test(test/integration/test_nav2.py TIMEOUT 420)
    add_launch_test(test/integration/test_slam.py TIMEOUT 300)
    add_launch_test(test/integration/test_ekf.py TIMEOUT 300)
    add_launch_test(test/integration/test_multirobot.py TIMEOUT 360)
  endif()
endif()

ament_package()
