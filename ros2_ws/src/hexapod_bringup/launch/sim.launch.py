"""Full simulation stack: Gazebo, robot spawn, ros2_control, sensor bridge,
and locomotion_node. This is the P1 "walk in Gazebo" launch entry point --
`ros2 launch hexapod_bringup sim.launch.py`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="hexapod_training_course.sdf",
        description="World file in hexapod_simulation/worlds to load",
    )
    gait_arg = DeclareLaunchArgument(
        "initial_gait", default_value="tripod", description="tripod | ripple | wave"
    )
    camera_arg = DeclareLaunchArgument(
        "use_camera", default_value="true",
        description="false: no depth-camera rendering (faster; E2/E3 runs), also skips the point-cloud nodes",
    )
    x_arg = DeclareLaunchArgument("spawn_x", default_value="-5.45")
    y_arg = DeclareLaunchArgument("spawn_y", default_value="-1.4")
    rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Also open RViz alongside Gazebo"
    )

    model = PathJoinSubstitution(
        [FindPackageShare("hexapod_description"), "urdf", "hexapod.urdf.xacro"]
    )
    # Wrap explicitly as a string parameter: ROS 2 Jazzy's launch_ros tries to
    # YAML-parse an unwrapped Command() substitution result, which fails on
    # the XML robot_description string (this is the "Unable to parse the
    # value of parameter robot_description as yaml" error).
    # use_sim:=true selects gz_ros2_control + the Gazebo sensor/odometry plugins
    # (hexapod_description defaults to mock hardware). controllers_file points
    # gz_ros2_control at this package's controllers.yaml (hexapod_controller,
    # which locomotion_node publishes to) instead of hexapod_description's own.
    controllers = PathJoinSubstitution(
        [FindPackageShare("hexapod_bringup"), "config", "controllers.yaml"]
    )
    description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", model,
            " use_sim:=true controllers_file:=", controllers,
            " use_camera:=", LaunchConfiguration("use_camera"),
        ]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("hexapod_simulation"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": description, "use_sim_time": True}],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [FindPackageShare("hexapod_simulation"), "config", "bridge.yaml"]
                ),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    # Gazebo's point cloud uses x-forward axes; see points_frame_relabel.py.
    points_relabel = Node(
        package="hexapod_simulation",
        executable="points_frame_relabel.py",
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_camera")),
    )

    # Persistent world-fixed voxel map of everything the depth camera has seen.
    voxel_map = Node(
        package="hexapod_perception",
        executable="voxel_map_node",
        parameters=[{"use_sim_time": True}],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("use_camera")),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "hexapod",
            "-topic", "robot_description",
            # Spawned at the URDF zero pose, where the feet are 159.8 mm below
            # base_link (hexapod_description README) -> ~10 mm drop.
            "-x", LaunchConfiguration("spawn_x"), "-y", LaunchConfiguration("spawn_y"), "-z", "0.17",
        ],
        output="screen",
    )

    controllers = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "hexapod_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
            "--service-call-timeout", "10",
            "--switch-timeout", "10",
            "--activate-as-group",
        ],
        output="screen",
    )
    delayed_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[controllers])
    )

    # IMU attitude estimate (/state/attitude, complementary filter) for posture control (E4)
    state_estimator = Node(
        package="hexapod_state_estimation",
        executable="state_estimator_node",
        parameters=[{"use_sim_time": True}],
        output="screen",
        emulate_tty=True,
    )

    locomotion = Node(
        package="hexapod_locomotion",
        executable="locomotion_node",
        parameters=[
            PathJoinSubstitution([FindPackageShare("hexapod_locomotion"), "config", "gait.yaml"]),
            {"use_sim_time": True, "initial_gait": LaunchConfiguration("initial_gait")},
        ],
        output="screen",
        # Without this, Python's stdout is block-buffered when it isn't a
        # real TTY (true for every process ros2 launch starts) -- so a
        # long-running Python node's get_logger() output (including error
        # tracebacks) just sits in an internal buffer and is never flushed
        # to the log/terminal until the process exits. C++ nodes (gazebo,
        # robot_state_publisher) don't have this problem, which is why they
        # showed up fine while locomotion_node showed nothing at all, alive
        # or not. emulate_tty makes launch allocate a pty for this process
        # so Python's own isatty() check switches it to line-buffering.
        emulate_tty=True,
    )

    # Same view as `hexapod_description display.launch.py`, but pointed at
    # the live simulation's /robot_description and TF instead of running its
    # own robot_state_publisher -- so both Gazebo and RViz reflect the same
    # spawned hexapod. Disable with `use_rviz:=false`.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("hexapod_description"), "rviz", "hexapod_sim.rviz"]),
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            world_arg,
            gait_arg,
            camera_arg,
            x_arg,
            y_arg,
            rviz_arg,
            gazebo,
            robot_state_publisher,
            bridge,
            points_relabel,
            voxel_map,
            spawn,
            delayed_controllers,
            state_estimator,
            locomotion,
            rviz,
        ]
    )
