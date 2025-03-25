from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml
import os

def get_node(context, *args, **kwargs):
    params = load_yaml(
        "manipulators",
        os.path.join("config", "joystick", "generic.yaml"),
    )

    return [Node(
        package="manipulators",
        executable="joystick_controller_node",
        parameters=[
            params
        ]
    )]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=get_node)
    ])