"""Start the Gazebo world, the /clock bridge and RViz.

Run this first, in its own terminal. Robots are added afterwards with
lampo_gz_mm.launch.py, one launch per robot.

/clock is bridged here rather than per-robot: there is exactly one simulation
clock, and having every robot's bridge republish it gives you N publishers on
/clock and a jittery sim time.
"""

import os
import pathlib

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory('lampo_description')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('rviz')

    args = [
        DeclareLaunchArgument(
            'world', default_value=os.path.join(pkg, 'worlds', 'warehouse.sdf'),
            description='Full path to the SDF world to load.'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Show the Gazebo GUI. false runs headless (server '
                        'only), which is much faster for CI or a laptop.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz alongside Gazebo.'),
        DeclareLaunchArgument(
            'verbosity', default_value='3',
            description='gz sim log verbosity, 0-4.'),
    ]

    # gz-sim resolves package:// by stripping the scheme and searching
    # GZ_SIM_RESOURCE_PATH for "<pkg>/<rest>", so the directory that CONTAINS
    # the package share dir is what has to be on the path.
    #
    # ros_gz_sim populates this variable only from legacy Gazebo-Classic
    # <gazebo_ros gazebo_model_path=...> exports in package.xml. Setting it
    # here is explicit and does not depend on that deprecated mechanism --
    # without it the world loads with no geometry at all.
    resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(pathlib.Path(pkg).parent))

    # Render on the discrete GPU where there is one.
    #
    # On an NVIDIA Optimus laptop the X server drives the Intel iGPU, so
    # Gazebo renders its sensors there -- or falls back to software when EGL
    # cannot get a DRI2 screen. A 640-beam lidar and an RGBD camera then eat
    # the CPU, Nav2's 20 Hz control loop drops to ~5 Hz, and goals start
    # failing with "Failed to make progress". Measured on this machine: the
    # RTX 4080 sat at 93 MiB while glxinfo reported "Mesa Intel(R) Graphics".
    #
    # These two variables are the standard PRIME offload switches. They are
    # simply ignored where there is no NVIDIA driver, so this is safe on
    # single-GPU, AMD and Intel-only machines.
    prime_offload = [
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
    ]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py')),
        launch_arguments={
            # -r starts the world unpaused; -s is server-only (no GUI).
            'gz_args': [' -r -v ', LaunchConfiguration('verbosity'), ' ', world],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(gui),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [' -s -r -v ', LaunchConfiguration('verbosity'),
                        ' ', world],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(gui),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('lampo_description'), 'rviz', 'config.rviz'])],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(args + prime_offload +
                             [resource_path, gz_sim, gz_sim_headless,
                              clock_bridge, rviz])
