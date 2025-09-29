from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from manipulators.launch_utils import load_yaml
import os

def get_node(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type").perform(context)
    params = load_yaml(
        "manipulators",
        os.path.join("config", "drivers", f"{ur_type}.yaml"),
    )

    return [Node(
        package="manipulators",
        executable="driver_trajectory_converter",
        name="driver_trajectory_converter",
        parameters=[
            params
        ]
    )]

def generate_launch_description():
    arg = DeclareLaunchArgument(
        "ur_type",
        description="Type/series of used UR robot.",
        choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
        default_value="ur5e",
    )

    return LaunchDescription([
        arg, 
        OpaqueFunction(function=get_node)
    ])
