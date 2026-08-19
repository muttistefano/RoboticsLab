"""Spawn one LAMPO robot into an already-running Gazebo world.

Start the world first with lampo_sandbox.launch.py, then run this once per
robot with a distinct `namespace`.

Bring-up order matters and is enforced with event handlers, not sleeps:

    robot_state_publisher  ->  publishes <ns>/robot_description
    ros_gz_sim create      ->  reads that topic, inserts the model
    (gz_ros2_control now creates the controller_manager)
    joint_state_broadcaster -> forward_position_controller -> gripper

Each step waits for the previous process to exit, so the sequence is correct
on a cold cache and on a slow laptop alike.
"""

import os
import tempfile

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
import launch_ros.descriptions
from launch_ros.substitutions import FindPackageShare
import yaml

# The sentinel used inside config/*.yaml for "this robot's namespace".
PREFIX_SENTINEL = 'PREFIX_'


def _substitute(obj, namespace):
    """Replace PREFIX_ with the namespace in every string, key or value.

    Keys matter: joint_trajectory_controller looks up per-joint tolerances as
    constraints.<actual_joint_name>, so the constraint keys have to be
    rewritten too, not just the entries of the `joints:` list.
    """
    if isinstance(obj, dict):
        return {_substitute(k, namespace): _substitute(v, namespace)
                for k, v in obj.items()}
    if isinstance(obj, list):
        return [_substitute(i, namespace) for i in obj]
    if isinstance(obj, str):
        return obj.replace(PREFIX_SENTINEL, namespace)
    return obj


def _render_config(name, namespace):
    """Read config/<name>, substitute the namespace, write it to a temp file.

    The file is per-user and per-namespace: a predictable path in a shared
    /tmp would collide between students on a lab machine.
    """
    src = os.path.join(get_package_share_directory('lampo_description'),
                       'config', name)
    with open(src) as f:
        data = yaml.safe_load(f)

    out_dir = os.path.join(tempfile.gettempdir(), f'lampo_{os.getuid()}')
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, f'{namespace}{name}')
    with open(out, 'w') as f:
        yaml.dump(_substitute(data, namespace), f)
    return out


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    mm = LaunchConfiguration('mm')

    controllers_file = _render_config('ur_controllers.yaml', namespace)

    robot_description = launch_ros.descriptions.ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            PathJoinSubstitution([FindPackageShare(
                LaunchConfiguration('description_package')), 'urdf',
                LaunchConfiguration('description_file')]),
            ' ', 'name:=', namespace, 'mm1',
            ' ', 'omni:=', LaunchConfiguration('omni'),
            ' ', 'mm:=', mm,
            ' ', 'gripper_control:=',
            LaunchConfiguration('gripper_control'),
            ' ', 'prefix:=', namespace,
            ' ', 'simulation_controllers:=', controllers_file,
        ]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        # Both transform topics are namespaced, and both are relayed onto the
        # global topics below.
        #
        # /tf_static must be namespaced: Nav2's servers are composable nodes
        # loaded by nav2_bringup's own launch files, which remap /tf_static to
        # <ns>/tf_static. Publishing statics only on the global topic leaves
        # that namespaced topic with no publisher at all, so AMCL never learns
        # base_footprint -> front_laser and silently drops every scan --
        # the stack reports "active" and the robot simply never localises.
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        parameters=[{'robot_description': robot_description},
                    {'use_sim_time': True}],
    )

    spawn = Node(
        name='spawn_robot',
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', f'/{namespace}/robot_description',
            '-name', namespace,
            '-allow_renaming', 'true',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        parameters=[{'use_sim_time': True}],
    )

    def spawner(controller):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller, '--controller-manager',
                       f'/{namespace}/controller_manager'],
            namespace=namespace,
            output='screen',
            condition=IfCondition(mm),
        )

    jsb = spawner('joint_state_broadcaster')
    arm = spawner('forward_position_controller')

    # Chain: spawn -> joint_state_broadcaster -> arm controller. Each waits for
    # the previous process to exit, so the controller_manager always exists
    # before a spawner talks to it and controllers activate in a deterministic
    # order. (The old code fired all of these in one TimerAction(5.0) and hoped.)
    #
    # No gripper controller: see the note in urdf/mm_gripper.xacro -- the
    # gripper is passive unless gripper_control:=true.
    ordering = [
        RegisterEventHandler(OnProcessExit(target_action=spawn,
                                           on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb,
                                           on_exit=[arm])),
    ]

    bridge_file = _render_config('bridge.yaml', namespace)
    # bridge.yaml uses the literal "/prefix/" marker rather than PREFIX_, so it
    # stays readable as a standalone file. One replace handles every topic --
    # adding a topic to the yaml needs no change here.
    with open(bridge_file) as f:
        bridge_data = yaml.safe_load(f)
    for item in bridge_data:
        if 'ros_topic_name' in item:
            item['ros_topic_name'] = item['ros_topic_name'].replace(
                '/prefix/', f'/{namespace}/', 1)
        if 'gz_topic_name' in item:
            item['gz_topic_name'] = item['gz_topic_name'].replace(
                '/prefix/', f'/{namespace}/', 1)
    with open(bridge_file, 'w') as f:
        yaml.dump(bridge_data, f)

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        output='screen',
        # No use_sim_time: this process is downstream of /clock, and making it
        # wait on the very clock the sandbox publishes can stall its timers.
        parameters=[{'config_file': bridge_file}],
    )

    # Everything this robot publishes is namespaced, so a global consumer --
    # RViz, or `ros2 run tf2_tools view_frames` -- would see nothing. Relay
    # both transform topics onto the global ones, where the prefixed frames of
    # any number of robots merge into a single tree.
    #
    # topic_tools/relay matches the input publisher's QoS, so the relayed
    # /tf_static keeps its TRANSIENT_LOCAL durability and late joiners still
    # receive the static transforms. (Verified on Kilted: both the source and
    # the relayed publisher report durability TRANSIENT_LOCAL.)
    relay_tf = Node(
        package='topic_tools', executable='relay',
        namespace=namespace, name='relay_tf_to_global',
        arguments=['tf', '/tf'],
        parameters=[{'use_sim_time': True}],
    )
    relay_tf_static = Node(
        package='topic_tools', executable='relay',
        namespace=namespace, name='relay_tf_static_to_global',
        arguments=['tf_static', '/tf_static'],
        parameters=[{'use_sim_time': True}],
    )
    return [robot_state_publisher, gz_bridge, relay_tf, relay_tf_static,
            spawn] + ordering


def generate_launch_description():
    args = [
        DeclareLaunchArgument('namespace', default_value='r1_',
                              description='Namespace and TF prefix. Must end '
                                          'with a separator, e.g. "r2_".'),
        DeclareLaunchArgument('description_package',
                              default_value='lampo_description'),
        DeclareLaunchArgument('description_file',
                              default_value='system.urdf.xacro'),
        DeclareLaunchArgument('mm', default_value='false',
                              description='Spawn as a mobile manipulator '
                                          '(UR arm + gripper + camera).'),
        DeclareLaunchArgument('omni', default_value='true',
                              description='true = mecanum base, '
                                          'false = differential base.'),
        # EXPERIMENTAL, and off for a reason: see urdf/mm_gripper.xacro. With
        # this true the gripper exports ros2_control interfaces, but the mimic
        # joints of the 4-bar finger linkage drift outside their URDF limits
        # and the joint limiter aborts the simulator with a fatal exception.
        DeclareLaunchArgument('gripper_control', default_value='false',
                              description='Expose the gripper through '
                                          'ros2_control. Experimental: known '
                                          'to crash the simulator.'),
        # Spawn pose. Nav2's AMCL is seeded from these same values, so the two
        # can no longer drift apart -- see lampo_nav_omni.launch.py.
        DeclareLaunchArgument('x', default_value='-3.5'),
        DeclareLaunchArgument('y', default_value='2.2'),
        DeclareLaunchArgument('z', default_value='0.2'),
        DeclareLaunchArgument('yaw', default_value='0.3'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
