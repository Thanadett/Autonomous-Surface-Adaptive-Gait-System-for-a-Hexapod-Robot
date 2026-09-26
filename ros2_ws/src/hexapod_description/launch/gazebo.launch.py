"""Spawn the hexapod in Gazebo Sim (Harmonic) with ros2_control, IMU, depth camera and foot contacts.

    ros2 launch hexapod_description gazebo.launch.py                 # flat ground
    ros2 launch hexapod_description gazebo.launch.py world:=ramp     # 15 deg ramp at x = 0.5 m
    ros2 launch hexapod_description gazebo.launch.py rviz:=true

Command the legs (18 values, order as in config/controllers.yaml = hexapod_kinematics.LEGS x coxa/femur/tibia):
    ros2 topic pub --once /leg_position_controller/commands std_msgs/msg/Float64MultiArray \
      "{data: [0,0.3,-0.3, 0,0.3,-0.3, 0,0.3,-0.3, 0,0.3,-0.3, 0,0.3,-0.3, 0,0.3,-0.3]}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod_description')
    world = LaunchConfiguration('world')
    spawn_z = LaunchConfiguration('spawn_z')

    world_file = PathJoinSubstitution([pkg_share, 'worlds', PythonExpression(["'", world, "' + '.sdf'"])])

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg_share, 'urdf', 'hexapod.urdf.xacro'),
                 ' use_sim:=true use_sensors:=', LaunchConfiguration('use_sensors')]),
        value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v 3 ', world_file], 'on_exit_shutdown': 'true'}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'hexapod', '-z', spawn_z])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  parameters=[{'config_file': os.path.join(pkg_share, 'config', 'gz_bridge.yaml'),
                               'use_sim_time': True}])

    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])
    legs = Node(package='controller_manager', executable='spawner',
                arguments=['leg_position_controller', '--controller-manager', '/controller_manager'])
    traj = Node(package='controller_manager', executable='spawner',
                arguments=['leg_trajectory_controller', '--inactive',
                           '--controller-manager', '/controller_manager'])

    rviz = Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=['-d', os.path.join(pkg_share, 'rviz', 'display.rviz')],
                parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='flat', description='flat | ramp'),
        DeclareLaunchArgument('spawn_z', default_value='0.20', description='Spawn height of base_link [m]'),
        DeclareLaunchArgument('use_sensors', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        # let Gazebo resolve package://hexapod_description/... mesh URIs
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_share)),
        gz_sim, rsp, spawn, bridge, rviz,
        # start controllers only after the robot exists in Gazebo
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[legs, traj])),
    ])
