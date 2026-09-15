#!/usr/bin/env python3
"""Full Gazebo Harmonic simulation of the NRW vertical chain-lift AS/RS cell."""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, OpaqueFunction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  FindExecutable)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'nrw_cell_sim'
WORLD_NAME = 'nrw_cell_world'


def generate_launch_description():
    share = FindPackageShare(PKG)

    args = [
        DeclareLaunchArgument('gui', default_value='true',
                              description='run the Gazebo GUI (false = headless server)'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='also open RViz2'),
        DeclareLaunchArgument('paused', default_value='false',
                              description='start the simulation paused'),
        DeclareLaunchArgument('demo', default_value='false',
                              description='run the automatic storage/retrieval work cycle'),
        DeclareLaunchArgument('payload', default_value='true',
                              description='pre-load grey packages into the comptoir slots'),
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution(
            [share, 'worlds', 'nrw_cell.sdf'])),
    ]

    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([share, 'urdf', 'nrw_cell.urdf.xacro']),
    ]), value_type=str)

    # -s = server only (headless), -r = start running instead of paused
    def _gz_args(context):
        gui = context.launch_configurations['gui'].lower() in ('true', '1')
        paused = context.launch_configurations['paused'].lower() in ('true', '1')
        world = context.launch_configurations['world']
        flags = ([] if gui else ['-s']) + ([] if paused else ['-r']) + ['-v', '3']
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
            launch_arguments={'gz_args': ' '.join(flags + [world]),
                              'on_exit_shutdown': 'true'}.items())]

    gz = OpaqueFunction(function=_gz_args)

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'nrw_cell',
                   '-world', WORLD_NAME,
                   '-x', '0', '-y', '0', '-z', '0'],
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
    )

    def spawner(name, extra=()):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '60', *extra])

    jsb = spawner('joint_state_broadcaster')
    cell = spawner('cell_controller')
    jog = spawner('jog_controller', ['--inactive'])

    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', PathJoinSubstitution([share, 'rviz', 'nrw_cell.rviz'])],
        parameters=[{'use_sim_time': True}],
    )

    payload = Node(
        package=PKG, executable='spawn_payload.py', output='screen',
        condition=IfCondition(LaunchConfiguration('payload')),
        # only the shelf: the carriage is loaded by the cycle at the infeed
        parameters=[{'use_sim_time': True, 'where': 'shelf', 'count': 5}],
    )

    demo = Node(
        package=PKG, executable='cycle_demo.py', output='screen',
        condition=IfCondition(LaunchConfiguration('demo')),
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription(args + [
        gz, rsp, bridge, spawn,
        # controllers only exist once the model is in the world
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb,   on_exit=[cell, rviz, payload])),
        RegisterEventHandler(OnProcessExit(target_action=cell,  on_exit=[jog, demo])),
    ])
