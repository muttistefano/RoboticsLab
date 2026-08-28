"""SLAM with slam_toolbox, and optional odom+IMU fusion with an EKF.

Build a map instead of loading the pre-made one:

    T1  ros2 launch lampo_description lampo_sandbox.launch.py
    T2  ros2 launch lampo_description lampo_gz_mm.launch.py
    T3  ros2 launch lampo_description lampo_slam.launch.py
    T4  ros2 launch lampo_description lampo_joy.launch.py

Drive the robot around, watch the map build in RViz, then save it:

    ros2 service call /r1_/slam_toolbox/save_map slam_toolbox/srv/SaveMap \
        "{name: {data: my_map}}"

Do NOT run this at the same time as lampo_nav_omni.launch.py: both would
publish the map -> odom transform and fight over it.
"""

import os
import tempfile

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

PREFIX_SENTINEL = 'PREFIX_'


def _render(name, namespace):
    """Substitute the namespace into a config file, as the other launch files do."""
    src = os.path.join(get_package_share_directory('lampo_description'),
                       'config', name)
    with open(src) as f:
        data = yaml.safe_load(f)

    def sub(o):
        if isinstance(o, dict):
            return {sub(k): sub(v) for k, v in o.items()}
        if isinstance(o, list):
            return [sub(i) for i in o]
        if isinstance(o, str):
            return o.replace(PREFIX_SENTINEL, namespace)
        return o

    out_dir = os.path.join(tempfile.gettempdir(), f'lampo_{os.getuid()}')
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, f'{namespace}{name}')
    with open(out, 'w') as f:
        yaml.dump(sub(data), f)
    return out


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    noisy = LaunchConfiguration('noisy').perform(context).lower() == 'true'

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        output='screen',
        parameters=[_render('slam_toolbox.yaml', namespace),
                    {'use_sim_time': True}],
        # slam_toolbox publishes map -> odom on the global /tf, and reads the
        # rest of the tree from there too.
        #
        # /map and /map_metadata are hardcoded absolute inside slam_toolbox, so
        # without these remappings a second robot's mapper would publish over
        # the first one's map. Everything else in this repo is namespaced;
        # this keeps the map consistent with that.
        remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static'),
                    ('/map', 'map'), ('/map_metadata', 'map_metadata')],
    )

    # Optional. Off by default because the drive plugin already provides
    # odom -> base_footprint; enabling both would give two publishers of the
    # same transform. Turn the plugin's <tf_topic> off first if you use this.
    #
    # With noisy:=true the EKF is fed through sensor_noise.py instead of the
    # simulator's too-perfect topics -- that is the Kalman tuning exercise of
    # doc/control.md: the filter finally has something to filter, and the
    # plugin's clean odom remains available as ground truth to plot against.
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace=namespace,
        output='screen',
        parameters=[_render('ekf.yaml', namespace), {'use_sim_time': True}],
        remappings=[('odom', 'odom_noisy'),
                    ('imu', 'imu_noisy')] if noisy else [],
        condition=IfCondition(LaunchConfiguration('use_ekf')),
    )
    noise = Node(
        package='lampo_description',
        executable='sensor_noise.py',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('noisy')),
    )

    # slam_toolbox is a LIFECYCLE node on Kilted: started on its own it sits in
    # `unconfigured` forever -- it never declares its parameters, never
    # subscribes to the scan and never publishes a map, while looking perfectly
    # healthy in `ros2 node list`. Something has to walk it through
    # configure -> activate, and nav2_lifecycle_manager is the standard way.
    # This is also the one place in the repo where lifecycle management is
    # visible on its own, rather than buried inside the Nav2 bringup.
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox'],
            # slam_toolbox does not create a bond with the manager the way
            # Nav2's own servers do, so the manager declares the bringup failed
            # 4 s after activating it. Disable the bond check.
            'bond_timeout': 0.0,
        }],
    )

    return [slam, lifecycle, ekf, noise]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='r1_',
                              description='Robot to map with.'),
        DeclareLaunchArgument(
            'use_ekf', default_value='false',
            description='Also run a robot_localization EKF fusing wheel '
                        'odometry with the IMU. See the note in ekf.yaml.'),
        DeclareLaunchArgument(
            'noisy', default_value='false',
            description='Feed the EKF through sensor_noise.py (odom_noisy, '
                        'imu_noisy) instead of the simulator-perfect topics. '
                        'The Kalman tuning exercise of doc/control.md.'),
        OpaqueFunction(function=launch_setup),
    ])
