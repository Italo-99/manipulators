from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml
import os

def get_ur_moveit_launch_params(context,
                                ur_type_: LaunchConfiguration | None = None,
                                description_path_: LaunchConfiguration | None = None,
                                tf_prefix_: LaunchConfiguration | None = None,
                                description_package_: LaunchConfiguration | None = None,
                                moveit_config_package_: LaunchConfiguration | None = None,
                                moveit_joint_limits_file_: LaunchConfiguration | None = None,
                                moveit_kinematics_file_: LaunchConfiguration | None = None,
                                description_semantic_file_: LaunchConfiguration | None = None,
                                prefix_: LaunchConfiguration | None = None):
    """
        This function returns a dictionary with all the parameters needed to launch move_group node for UR robots.
        It gets all the parameters from the launch arguments, passed arguments override the default values.
        TODO: Make this thing a module, for now copy-paste where needed.
        Args:
            ur_type:                   Type/series of used UR robot.
            description_path:          Path to the URDF/XACRO description file.
            tf_prefix:                 Prefix for tf, useful for multi-robot setup.
            rviz_config_path:          Path to the RViz config file.
            rate:                      Publish rate for the joint_state_publisher.
            description_package:       Name of the package containing the robot description.
            moveit_config_package:     Name of the package containing the MoveIt configuration.
            moveit_joint_limits_file:  Name of the file containing the joint limits configuration.
            moveit_kinematics_file:    Name of the file containing the kinematics configuration.
            description_semantic_file: Name of the file containing the semantic description.
            prefix:                    Prefix for the joint names, useful for multi-robot setup.
    """

    ur_type                   = ur_type_ if ur_type_ else                                     LaunchConfiguration("ur_type")
    description_path          = description_path_ if description_path_ else                   LaunchConfiguration("description_path")
    tf_prefix                 = tf_prefix_ if tf_prefix_ else                                 LaunchConfiguration("tf_prefix")
    description_package       = description_package_ if description_package_ else             LaunchConfiguration("description_package")
    moveit_config_package     = moveit_config_package_ if moveit_config_package_ else         LaunchConfiguration("moveit_config_package")
    moveit_joint_limits_file  = moveit_joint_limits_file_ if moveit_joint_limits_file_ else   LaunchConfiguration("moveit_joint_limits_file")
    moveit_kinematics_file    = moveit_kinematics_file_ if moveit_kinematics_file_ else       LaunchConfiguration("moveit_kinematics_file")
    description_semantic_file = description_semantic_file_ if description_semantic_file_ else LaunchConfiguration("description_semantic_file")
    prefix                    = prefix_ if prefix_ else                                       LaunchConfiguration("prefix")

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
            "name:=ur",
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
            "name:=ur",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
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

    moveit_controllers = {
        #"moveit_simple_controller_manager" : controllers_yaml, 
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    }

    params = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_planning_pipeline_config,
        trajectory_execution,
        moveit_controllers
    ]

    return params

def launch_setup(context, *args, **kwargs):

    # ---------------------------------------- ACTIONS ----------------------------------------

    mp_params = load_yaml(
        "manipulators",
        os.path.join(
            "config",
            LaunchConfiguration("ur_type").perform(context) + ".yaml"
        )
    )

    mp_params["ee_name"] = "tool0"

    moveit_params = get_ur_moveit_launch_params(context)

    # ---------------------------------------- NODES ----------------------------------------


    nodes_to_start = []

    nodes_to_start.append(
        Node(
            package="manipulators",
            executable="manipulator_planner",
            name="manipulator_planner",
            output="both",
            parameters=[
                mp_params
            ] + moveit_params,
        )
    )

    nodes_to_start.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                FindPackageShare("manipulators").perform(context) + "/launch/planning_context.launch.py"
            ),
            launch_arguments=[
                ("ur_type", LaunchConfiguration("ur_type")),
                ("description_package", LaunchConfiguration("description_package")),
                ("description_semantic_file", LaunchConfiguration("description_semantic_file")),
                ("prefix", LaunchConfiguration("prefix")),
                ("description_path", LaunchConfiguration("description_path")),
                ("tf_prefix", LaunchConfiguration("tf_prefix")),
                ("moveit_joint_limits_file", LaunchConfiguration("moveit_joint_limits_file")),
                ("moveit_kinematics_file", LaunchConfiguration("moveit_kinematics_file")),
                ("moveit_config_package", LaunchConfiguration("moveit_config_package")),
            ]
        )
    )

    return nodes_to_start

def generate_launch_description():

    # ---------------------------------------- ARGS DECLARATION ----------------------------------------

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

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
