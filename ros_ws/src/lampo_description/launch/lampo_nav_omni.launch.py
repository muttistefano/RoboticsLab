# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    lampo_dir = get_package_share_directory('lampo_description')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    keepout_mask_yaml_file = LaunchConfiguration('keepout_mask')
    speed_mask_yaml_file = LaunchConfiguration('speed_mask')
    graph_filepath = LaunchConfiguration('graph')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_localization = LaunchConfiguration('use_localization')
    use_keepout_zones = LaunchConfiguration('use_keepout_zones')
    use_speed_zones = LaunchConfiguration('use_speed_zones')
    initial_x = LaunchConfiguration('x')
    initial_y = LaunchConfiguration('y')
    initial_yaw = LaunchConfiguration('yaw')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    # Both transform topics are namespaced. lampo_gz_mm.launch.py publishes
    # each robot's transforms under its namespace and relays them onto the
    # global topics for RViz; Nav2 reads the namespaced ones, which is also
    # what nav2_bringup's own composable-node launch files expect.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    yaml_substitutions = {
        'KEEPOUT_ZONE_ENABLED': use_keepout_zones,
        'SPEED_ZONE_ENABLED': use_speed_zones,
        'prefix/base_footprint': [namespace, 'base_footprint'],
        'prefix/odom': [namespace, 'odom']
    }

    # param_rewrites applies to every node in the file. use_sim_time was
    # previously set on amcl alone, so the other ~15 Nav2 servers ran on the
    # wall clock while TF was stamped in sim time -- which shows up as endless
    # "Lookup would require extrapolation into the future" errors.
    #
    # The AMCL seed is taken from the same x/y/yaw the robot is spawned at
    # (see lampo_gz_mm.launch.py) so the two cannot drift apart.
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'x': initial_x,
        'y': initial_y,
        'yaw': initial_yaw,
        # Where the collision monitor -- the last stage of the velocity chain --
        # publishes. Default goes straight to the gz bridge. Set it to
        # cmd_vel_nav when running twist_mux (lampo_joy.launch.py) so the
        # joystick can override navigation instead of fighting it.
        'cmd_vel_out_topic': cmd_vel_topic,
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        value_rewrites=yaml_substitutions,
        convert_types=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='r1_', description='Top-level namespace'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(lampo_dir, 'map', 'map.yaml'),
        description='Full path to map yaml file to load'
    )

    # Spawn pose of the robot this Nav2 instance drives. Keep in sync with the
    # x/y/yaw passed to lampo_gz_mm.launch.py; AMCL is seeded from these.
    declare_cmd_vel_topic_cmd = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='cmd_vel_safe',
        description='Collision monitor output topic. Use "cmd_vel_nav" when '
                    'twist_mux is arbitrating with a joystick.')

    declare_x_cmd = DeclareLaunchArgument('x', default_value='-3.5')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='2.2')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='0.3')

    declare_keepout_mask_yaml_cmd = DeclareLaunchArgument(
        'keepout_mask', default_value='',
        description='Full path to keepout mask yaml file to load'
    )

    declare_speed_mask_yaml_cmd = DeclareLaunchArgument(
        'speed_mask', default_value='',
        description='Full path to speed mask yaml file to load'
    )

    declare_graph_file_cmd = DeclareLaunchArgument(
        'graph',
        default_value='', description='Path to the graph file to load'
    )

    declare_use_localization_cmd = DeclareLaunchArgument(
        'use_localization', default_value='True',
        description='Whether to enable localization or not'
    )

    declare_use_keepout_zones_cmd = DeclareLaunchArgument(
        'use_keepout_zones', default_value='False',
        description='Whether to enable keepout zones or not'
    )

    declare_use_speed_zones_cmd = DeclareLaunchArgument(
        'use_speed_zones', default_value='False',
        description='Whether to enable speed zones or not'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(lampo_dir, 'config', 'nav2_params_omni.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushROSNamespace(namespace),
            Node(
                condition=IfCondition(use_composition),
                name='nav2_container',
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[ParameterFile(configured_params, allow_substs=True),
                            {'autostart': autostart}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                output='screen',
            ),
            # No SLAM here on purpose. nav2_bringup's slam_launch.py would be
            # started with this file as its params, and nav2_params_omni.yaml
            # has no slam_toolbox section -- slam_toolbox would come up on
            # defaults, in unprefixed frames, and quietly not work.
            # Mapping lives in lampo_slam.launch.py, which is configured for it.
            # This launch file localises against a map that already exists.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'localization_launch.py')
                ),
                condition=IfCondition(use_localization),
                launch_arguments={
                    'namespace': namespace,
                    'map': map_yaml_file,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': configured_params,
                    'use_composition': use_composition,
                    'use_respawn': use_respawn,
                    'container_name': 'nav2_container',
                }.items(),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'keepout_zone_launch.py')
                ),
                condition=IfCondition(use_keepout_zones),
                launch_arguments={
                    'namespace': namespace,
                    'keepout_mask': keepout_mask_yaml_file,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': configured_params,
                    'use_composition': use_composition,
                    'use_respawn': use_respawn,
                    'container_name': 'nav2_container',
                }.items(),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'speed_zone_launch.py')
                ),
                condition=IfCondition(use_speed_zones),
                launch_arguments={
                    'namespace': namespace,
                    'speed_mask': speed_mask_yaml_file,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': configured_params,
                    'use_composition': use_composition,
                    'use_respawn': use_respawn,
                    'container_name': 'nav2_container',
                }.items(),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'navigation_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'graph': graph_filepath,
                    'params_file': configured_params,
                    'use_composition': use_composition,
                    'use_respawn': use_respawn,
                    'container_name': 'nav2_container',
                }.items(),
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_cmd_vel_topic_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(declare_keepout_mask_yaml_cmd)
    ld.add_action(declare_speed_mask_yaml_cmd)
    ld.add_action(declare_graph_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_localization_cmd)
    ld.add_action(declare_use_keepout_zones_cmd)
    ld.add_action(declare_use_speed_zones_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
