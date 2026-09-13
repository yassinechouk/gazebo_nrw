#!/usr/bin/env python3
"""Model-only view: RViz2 + joint_state_publisher_gui, no physics.

Useful for checking the kinematics and the rebuilt meshes without Gazebo.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'assemforchouk_sim'


def generate_launch_description():
    share = FindPackageShare(PKG)
    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([share, 'urdf', 'assemforchouk.urdf.xacro']),
        ' use_gazebo:=false',
    ]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             output='screen'),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'assemforchouk.rviz'])]),
    ])
