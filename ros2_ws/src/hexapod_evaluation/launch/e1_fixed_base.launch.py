"""E1 in Gazebo: robot bolted to the world 0.30 m up, legs driven through the grid.

    ros2 launch hexapod_evaluation e1_fixed_base.launch.py out_dir:=$PWD/results/e1
    ros2 launch hexapod_evaluation e1_fixed_base.launch.py solver:=dls max_targets:=5

Differences from hexapod_bringup/sim.launch.py, all on purpose:
  * fixed_base:=true (world -> base_link joint), so posture/falling cannot
    mix into the foot-tip error;
  * use_sensors:=false (no camera rendering load, no contact sensors -
    E1 does not use them) and only /clock is bridged: /odom_tf would give
    base_link a second TF parent next to world;
  * no locomotion_node - e1_sim is the only node commanding the joints.
The launch shuts itself down when e1_sim exits (results written).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument("out_dir", default_value="results/e1", description="where e1_sim writes its CSV/summary"),
        DeclareLaunchArgument("solver", default_value="analytic", description="analytic (locomotion_node default) | dls"),
        DeclareLaunchArgument("max_targets", default_value="0", description="0 = full grid (100 poses)"),
        DeclareLaunchArgument("fixed_base_z", default_value="0.30"),
    ]

    model = PathJoinSubstitution([FindPackageShare("hexapod_description"), "urdf", "hexapod.urdf.xacro"])
    controllers = PathJoinSubstitution([FindPackageShare("hexapod_bringup"), "config", "controllers.yaml"])
    description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", model,
            " use_sim:=true use_sensors:=false fixed_base:=true",
            " fixed_base_z:=", LaunchConfiguration("fixed_base_z"),
            " controllers_file:=", controllers,
        ]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("hexapod_simulation"), "launch", "gazebo.launch.py"])),
        launch_arguments={"world": "flat_world.sdf"}.items(),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": description, "use_sim_time": True}],
    )
    clock_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="screen",
    )
    spawn = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-name", "hexapod", "-topic", "robot_description"],
    )
    controllers_spawner = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=[
            "joint_state_broadcaster", "hexapod_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30", "--activate-as-group",
        ],
    )
    e1 = Node(
        package="hexapod_evaluation", executable="e1_sim", output="screen", emulate_tty=True,
        parameters=[{
            "use_sim_time": True,
            "out_dir": LaunchConfiguration("out_dir"),
            "solver": LaunchConfiguration("solver"),
            "max_targets": ParameterValue(LaunchConfiguration("max_targets"), value_type=int),
        }],
    )
    return LaunchDescription(args + [
        gazebo, robot_state_publisher, clock_bridge, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[controllers_spawner])),
        e1,
        RegisterEventHandler(OnProcessExit(target_action=e1, on_exit=[EmitEvent(event=Shutdown(reason="E1 finished"))])),
    ])
