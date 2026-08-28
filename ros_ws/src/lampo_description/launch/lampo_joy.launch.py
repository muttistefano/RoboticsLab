r"""Joystick teleoperation, arbitrated against Nav2 by twist_mux.

Without an arbiter the joystick and the navigation stack publish to the same
topic and fight each other. twist_mux subscribes to both, and forwards
whichever has the highest priority and is currently active -- so grabbing the
joystick always overrides autonomy, which is what you want on a real robot.

    joystick  --(prio 100)--\\
                             twist_mux --> <ns>/cmd_vel_safe --> gz
    Nav2      --(prio  10)--/
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)

    joy_node = Node(
        package='joy',
        executable='joy_node',
        namespace=namespace,
        name='joy_node',
        parameters=[{
            'device_id': LaunchConfiguration('joy_id'),
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
            'use_sim_time': True,
        }],
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        namespace=namespace,
        name='teleop_twist_joy_node',
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare('lampo_description'), 'config', 'joy.yaml']),
            {'use_sim_time': True},
        ],
        # Into twist_mux, not straight at the robot.
        remappings=[('/cmd_vel', 'cmd_vel_joy')],
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        namespace=namespace,
        name='twist_mux',
        output='screen',
        parameters=[
            PathJoinSubstitution([FindPackageShare('lampo_description'),
                                  'config', 'twist_mux.yaml']),
            {'use_sim_time': True},
        ],
        remappings=[('cmd_vel_out', 'cmd_vel_safe')],
    )

    return [joy_node, teleop_node, twist_mux_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='r1_',
                              description='Robot to drive. Must match the '
                                          'namespace used when spawning.'),
        DeclareLaunchArgument('joy_id', default_value='0',
                              description='Joystick index, /dev/input/js<N>.'),
        OpaqueFunction(function=launch_setup),
    ])
