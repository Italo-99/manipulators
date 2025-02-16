from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
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
        This function returns a list of dictionaries with all the parameters needed to launch move_group node for UR robots.
        A launch context must be provided by using OpaqueFunction, see manipulators/launch/planning_context.launch.py or 
        https://docs.openvins.com/gs-tutorial.html#gs-tutorial-ros2 for more information.
        Arguments must be of type LaunchConfiguration or None, if None they will be derived from the launch context, in this
        case all the necessary parameters must be declared in the launch file.

        Args:
            ur_type:                   Type/series of used UR robot.
            description_path:          Path to the URDF/XACRO description file.
            tf_prefix:                 Prefix for tf, useful for multi-robot setup.
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