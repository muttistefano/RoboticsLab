import os
from os import environ
from os import pathsep
import sys
from pathlib import Path

from launch import LaunchDescription, LaunchContext, LaunchService
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.actions import OpaqueFunction
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription,ExecuteProcess 
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.descriptions
import yaml
import tempfile




def generate_launch_description():

    declared_arguments = []



########## VISUALIZATION

    world_path = os.path.join(get_package_share_directory('lampo_description'),'worlds/warehouse.sdf')

    gazebo_server = GroupAction(
        actions=[
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments={
                'gz_args': [' -r -v 4 ' + world_path ],
                'gz_version': "9"
            }.items())
            ]
            )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("lampo_description"), "rviz", "config.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["-d", rviz_config_file],
    )



########## LAUNCHING


    nodes_to_start = [
        gazebo_server,
        rviz_node
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)


