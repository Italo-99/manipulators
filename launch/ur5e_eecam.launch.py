from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml
from manipulators.launch_utils import get_ur_moveit_launch_params, get_namespace
import os

def launch_setup(context, *args, **kwargs):

    # ---------------------------------------- ACTIONS ----------------------------------------

    # LOAD MANIPULATOR PLANNER PARAMETERS
    mp_params = load_yaml(
        "manipulators",
        os.path.join(
            "config/ur5e_eecam.yaml",
        )
    )

    # LOAD UR MOVEIT PARAMETERS
    moveit_params = get_ur_moveit_launch_params(context)

    # ---------------------------------------- NODES ----------------------------------------

    nodes_to_start = []

    # MANIPULATOR PLANNER NODE
    nodes_to_start.append(
        Node(
            package="manipulators",
            executable="manipulator_planner",
            output="both",
            namespace=get_namespace(context),
            parameters=[
                mp_params
            ] + moveit_params + [
                {"prefix" : LaunchConfiguration("prefix")},
            ],
        )
    )

    # ---------------------- GRIPPER NODES ----------------------

    # ROBOTIQ 85 GRIPPER NODE
    nodes_to_start.append(
        Node(
            package="motors_trajectory",
            executable="robotiq_85_gripper_node",
            namespace=get_namespace(context),
        )
    )

    # -----------------------------------------------------------

    # LAUNCH PLANNING CONTEXT
    nodes_to_start.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                FindPackageShare("manipulators").perform(context) + "/launch/planning_context.launch.py"
            ),
            launch_arguments=[
                ("ur_type", LaunchConfiguration("ur_type")),
                ("description_package", LaunchConfiguration("description_package")),
                ("description_semantic_path", LaunchConfiguration("description_semantic_path")),
                ("description_path", LaunchConfiguration("description_path")),
                ("prefix", LaunchConfiguration("prefix")),
                ("joint_limits_file", LaunchConfiguration("joint_limits_file")),
                ("kinematics_file", LaunchConfiguration("kinematics_file")),
                ("moveit_config_package", LaunchConfiguration("moveit_config_package")),
                ("publish_joint_states", LaunchConfiguration("publish_joint_states")),
            ]
        )
    )

    return nodes_to_start

def generate_launch_description():

    # ---------------------------------------- ARGS DECLARATION ----------------------------------------

    declared_arguments = []

    # RATE
    declared_arguments.append(
        DeclareLaunchArgument(
            "rate",
            description="Rate for the joint_state_publisher (hz).",
            default_value="500",
        )
    )

    # GUI
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="False",
            choices=["True", "False"],
            description="Whether to run joint_state_publisher with gui or not.",
        )
    )

    # PUBLISH JOINT STATES
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_joint_states",
            default_value="True",
            choices=["True", "False"],
            description="Whether to run joint state publisher node or not (Disable for real control).",
        )
    )

    # RVIZ
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="True",
            choices=["True", "False"],
            description="Whether to run rviz or not.",
        )
    )

    # UR TYPE
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
            default_value="ur5e",
        )
    )

    # DESCRIPTION PKG
    declared_arguments.append( 
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument is not set, it enables use of a custom description.",
        )
    )
    
    # Declare the SRDF path
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_semantic_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("manipulators"), "models", "srdf", "ur5e_robotiq_85_gripper.srdf.xacro"
            ]),
            description="MoveIt SRDF/XACRO description file of the robot (full path).",
        )
    )

    # Declare the URDF/XACRO path
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("manipulators"), "models", "urdf", "ur5e_robotiq_85_gripper.urdf.xacro"
            ]),
            description="URDF/XACRO description file (absolute path) of the robot.",
        )
    )

    # RVIZ CONFIG
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("manipulators"), "config", "rviz", "planner.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )
    
    # PREFIX
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )

    # JOINT LIMITS AND KINEMATICS
    declared_arguments.append(
        DeclareLaunchArgument(
            "joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits filename, only needed for UR robots",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "kinematics_file",
            default_value="default_kinematics.yaml",
            description="MoveIt kinematics filename, only needed for UR robots",
        )
    )

    # MOVEIT CONFIG PACKAGE
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files and MoveIt configuration files."
        )
    )

    # GRIPPER
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value="False",
            choices=["True", "False"],
            description="Whether to run the robotiq 85 gripper node or not."
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
