"""RViz visualisation of the hexapod URDF with joint sliders (no simulation).

    ros2 launch hexapod_description display.launch.py            # with joint_state_publisher_gui
    ros2 launch hexapod_description display.launch.py gui:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('hexapod_description')
    gui = LaunchConfiguration('gui')
    use_sensors = LaunchConfiguration('use_sensors')

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg, 'urdf', 'hexapod.urdf.xacro']),
                 ' use_sim:=false use_ros2_control:=false use_sensors:=', use_sensors]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show joint slider GUI'),
        DeclareLaunchArgument('use_sensors', default_value='true', description='Add IMU/camera frames'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'display.rviz'])]),
    ])
