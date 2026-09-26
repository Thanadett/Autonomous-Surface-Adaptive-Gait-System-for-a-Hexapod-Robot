"""Starts Gazebo Harmonic alone (no robot, no controllers -- see
hexapod_bringup for the full stack). `world` selects which .sdf to load.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="hexapod_training_course.sdf",
        description="World file in hexapod_simulation/worlds to load "
        "(flat_world.sdf | hexapod_training_course.sdf | obstacle_course.sdf)",
    )

    # robot_state_publisher/ros_gz_sim rewrite the URDF's package:// mesh URIs
    # to model://<package_name>/..., which Gazebo can only resolve if the
    # share directory *containing* that package folder is on
    # GZ_SIM_RESOURCE_PATH. Add every package's share parent dir so meshes
    # actually render instead of failing with "Unable to find file".
    resource_dirs = {
        os.path.dirname(get_package_share_directory(pkg))
        for pkg in ("hexapod_description", "hexapod_simulation")
    }
    existing_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path_value = os.pathsep.join(
        [*resource_dirs, existing_resource_path] if existing_resource_path else resource_dirs
    )
    set_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path_value)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": [
                "-r ",
                PathJoinSubstitution(
                    [FindPackageShare("hexapod_simulation"), "worlds", LaunchConfiguration("world")]
                ),
            ]
        }.items(),
    )
    return LaunchDescription([world_arg, set_resource_path, gazebo])
