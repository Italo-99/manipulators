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
        LaunchConfiguration('config').perform(context),
    )

    nodes = []
    profile = LaunchConfiguration('profile').perform(context)

    nodes.append(Node(
        package="manipulators",
        executable="joystick_controller_node",
        parameters=[
            params,
            {'profile': LaunchConfiguration('profile')}
        ],
        remappings=[('/manipulator/cmd_vel', '/cmd_vel')] # /mobile_manipulator/cmd_vel for the mobile control
    ))

    #Edit this to specify which profiles should use the game controller node, others will use the joy node
    game_controller_profiles = ['default']

    if profile.lower() in game_controller_profiles:
        nodes.append(Node(
            package="joy",
            executable="game_controller_node",
            parameters=[{
                'autorepeat_rate' : 30.0
            }]
        ))
    else:
        nodes.append(Node(
            package="joy",
            executable="joy_node",
            parameters=[{
                'autorepeat_rate' : 0.0
            }]
        ))
    
    return nodes

def generate_launch_description():

    profile_arg = DeclareLaunchArgument(
        'profile',
        default_value='default',
        description='Joystick profile to use'
    )

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution([
            'config',
            'joystick',
            'generic.yaml'
        ])
    )

    return LaunchDescription([
        profile_arg, config_arg,
        OpaqueFunction(function=get_node)
    ])