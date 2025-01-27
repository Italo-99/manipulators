from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
from ur_moveit_config.launch_common import load_yaml

import os

def launch_setup_sirio(context, *args, **kwargs):

    # Initialize Arguments
    
    manipulator_name          = LaunchConfiguration("manipulator_name")
    description_path          = LaunchConfiguration("description_path")
    tf_prefix                 = LaunchConfiguration("tf_prefix")                          # Prefix for tf, useful for multi-robot setup
    rviz_config_path          = LaunchConfiguration("rviz_config_path")                   # RViz config file
    rate                      = LaunchConfiguration("rate")                               # Publish rate for the joint_state_publisher
    description_package       = LaunchConfiguration("description_package")                # Description package 
    moveit_config_package     = LaunchConfiguration("moveit_config_package")              # Moveit config package
    description_semantic_file = LaunchConfiguration("description_semantic_file")
    prefix                    = LaunchConfiguration("prefix")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",    
            description_path
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    with open(PathJoinSubstitution([FindPackageShare(moveit_config_package), "config", description_semantic_file]).perform(context), "r") as file:
        robot_description_semantic_content = file.read()

    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}


    #KINEMATICS
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join("config", "kinematics.yaml")
        )
    }

    # PLANNING CONFIGURATION
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join("config", "joint_limits.yaml")
        )
    }

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_planning_yaml = load_yaml(
        str(moveit_config_package.perform(context)),
        os.path.join("config", "ompl_planning.yaml"),
    )

    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        # Execution time monitoring can be incompatible with the scaled JTC
        "trajectory_execution.execution_duration_monitoring": False,
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        # parameters=[{"robot_description": ParameterValue(value=robot_description_content, value_type=str)}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"rate": rate,}], 
        condition=IfCondition(LaunchConfiguration("gui"))
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",        
        parameters=[{"source_list": ['/move_group/fake_controller_joint_states'],
                      "rate": rate,}],
        condition=IfCondition(PythonExpression([LaunchConfiguration("gui"), " == False"]))    
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
        ],
        condition=IfCondition(LaunchConfiguration("rviz"))
    )

    # # Trajectory Execution Configuration
    # controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    # # # the scaled_joint_trajectory_controller does not work on fake hardware
    # # change_controllers = context.perform_substitution(use_sim_time)
    # # if change_controllers == "true":
    # controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
    # controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        #"moveit_simple_controller_manager" : controllers_yaml, 
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    }


    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
        ],
    )

    nodes_to_start = [
            robot_state_publisher_node,
            joint_state_publisher_gui_node, 
            joint_state_publisher_node,
            move_group_node,
            rviz_node
        ]
    return nodes_to_start

def launch_setup_ur(context, *args, **kwargs):

    # Initialize Arguments
    
    manipulator_name          = LaunchConfiguration("manipulator_name")
    ur_type                   = LaunchConfiguration("ur_type")
    description_path          = LaunchConfiguration("description_path")
    tf_prefix                 = LaunchConfiguration("tf_prefix")                          # Prefix for tf, useful for multi-robot setup
    rviz_config_path          = LaunchConfiguration("rviz_config_path")                   # RViz config file
    rate                      = LaunchConfiguration("rate")                               # Publish rate for the joint_state_publisher
    description_package       = LaunchConfiguration("description_package")                # Description package 
    moveit_config_package     = LaunchConfiguration("moveit_config_package")              # Moveit config package
    moveit_joint_limits_file  = LaunchConfiguration("moveit_joint_limits_file")           # Joint limits file
    moveit_kinematics_file    = LaunchConfiguration("moveit_kinematics_file")             # Kinematics file
    description_semantic_file = LaunchConfiguration("description_semantic_file")
    prefix                    = LaunchConfiguration("prefix")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, moveit_joint_limits_file]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, moveit_kinematics_file]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",    
            description_path,
            " ",
            "name:=",
            manipulator_name,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", description_semantic_file]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            manipulator_name,
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )

    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )


    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join("config", "joint_limits.yaml"),
        )
    }


    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }

    ompl_planning_yaml = load_yaml(
        str(moveit_config_package.perform(context)),
        os.path.join("config", "ompl_planning.yaml"),
    )

    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        # Execution time monitoring can be incompatible with the scaled JTC
        "trajectory_execution.execution_duration_monitoring": False,
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        # parameters=[{"robot_description": ParameterValue(value=robot_description_content, value_type=str)}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"rate": rate,}], 
        condition=IfCondition(LaunchConfiguration("gui"))
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",        
        parameters=[{"source_list": ['/move_group/fake_controller_joint_states'],
                      "rate": rate,}],
        condition=IfCondition(PythonExpression([LaunchConfiguration("gui"), " == False"]))    
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
        ],
        condition=IfCondition(LaunchConfiguration("rviz"))
    )

    # # Trajectory Execution Configuration
    # controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    # # # the scaled_joint_trajectory_controller does not work on fake hardware
    # # change_controllers = context.perform_substitution(use_sim_time)
    # # if change_controllers == "true":
    # controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
    # controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        #"moveit_simple_controller_manager" : controllers_yaml, 
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    }


    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
        ],
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
            "manipulator_type",
            description="What kind of manipulator is used, must be one of the supported types ('ur' or 'sirio').",
            default_value="ur",
            choices=["ur", "sirio"]
        )
    )

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
            "rviz",
            default_value="True",
            choices=["True", "False"],
            description="Whether to run rviz or not.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "manipulator_name",
            description="Name for the manipulator.",
            default_value="ur",
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
            "description_semantic_file",
            default_value="ur.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot (just filename, file must be inside <moveit_config_pkg>/config/ ).",
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
                [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits filename, only needed for UR robots",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_kinematics_file",
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

    manipulator_type = LaunchConfiguration("manipulator_type")

    return LaunchDescription(
        declared_arguments + [
            OpaqueFunction(function=launch_setup_ur, condition=IfCondition(PythonExpression(["'", manipulator_type, "' == 'ur'"]))),
            OpaqueFunction(function=launch_setup_sirio, condition=IfCondition(PythonExpression(["'", manipulator_type, "' == 'sirio'"]))),
        ]
    )
