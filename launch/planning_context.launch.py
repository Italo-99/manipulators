from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from manipulators.launch_utils import get_ur_moveit_launch_params

def launch_setup_ur(context, *args, **kwargs):

    # Initialize Arguments

    moveit_params = get_ur_moveit_launch_params(context)
    rate = LaunchConfiguration("rate")
    rviz_config_path = LaunchConfiguration("rviz_config_path")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_params[0]
            #robot description
        ], 
        # parameters=[{"robot_description": ParameterValue(value=robot_description_content, value_type=str)}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"rate": rate,}], 
        condition=IfCondition(PythonExpression([LaunchConfiguration("gui"), " and ", LaunchConfiguration("publish_joint_states")]))
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",        
        parameters=[{"source_list": ['/move_group/fake_controller_joint_states'],
                      "rate": rate,}],
        condition=IfCondition(PythonExpression([LaunchConfiguration("gui"), " == False", " and ", LaunchConfiguration("publish_joint_states")]))    
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=moveit_params[0:5],
            # Get only desired parameters:
            # robot_description,
            # robot_description_semantic,
            # robot_description_kinematics,
            # robot_description_planning,
            # ompl_planning_pipeline_config,
        condition=IfCondition(LaunchConfiguration("rviz"))
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=moveit_params
            # robot_description,
            # robot_description_semantic,
            # robot_description_kinematics,
            # robot_description_planning,
            # ompl_planning_pipeline_config,
            # trajectory_execution,
            # moveit_controllers,
    )

    nodes_to_start = [
            robot_state_publisher_node,
            joint_state_publisher_gui_node, 
            joint_state_publisher_node,
            move_group_node,
            rviz_node
        ]
    return nodes_to_start

def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "rate",
            description="Rate for the joint_state_publisher (hz).",
            default_value="500",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="False",
            choices=["True", "False"],
            description="Whether to run joint_state_publisher with gui or not.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_joint_states",
            default_value="True",
            choices=["True", "False"],
            description="Whether to run joint state publisher node or not (Disable for real control).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="True",
            choices=["True", "False"],
            description="Whether to run rviz or not.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
            default_value="ur5e",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_semantic_path",
            default_value=PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
            description="MoveIt SRDF/XACRO description file with the robot (full path).",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]
            ),
            description="URDF/XACRO description file (absolute path) with the robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("manipulators"), "config", "rviz", "planner.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )

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

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files and MoveIt configuration files."
        )
    )

    return LaunchDescription(
        declared_arguments + [
            OpaqueFunction(function=launch_setup_ur)
        ]
    )
