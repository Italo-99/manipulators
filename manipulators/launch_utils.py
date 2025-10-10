from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def get_namespace(context):
    pre = LaunchConfiguration("prefix").perform(context)
    while(pre.endswith("_")):
        pre = pre[:-1]
    return pre

def get_ur_moveit_launch_params(context,
                                ur_type_: LaunchConfiguration | None = None,
                                description_path_: LaunchConfiguration | None = None,
                                prefix_: LaunchConfiguration | None = None,
                                description_package_: LaunchConfiguration | None = None,
                                moveit_config_package_: LaunchConfiguration | None = None,
                                joint_limits_file_: LaunchConfiguration | None = None,
                                kinematics_file_: LaunchConfiguration | None = None,
                                description_semantic_path_: LaunchConfiguration | None = None,
                                xacro_args: LaunchConfiguration | None = None):
    """
        This function returns a list of dictionaries with all the parameters needed to launch move_group node for UR robots.
        A launch context must be provided by using OpaqueFunction, see manipulators/launch/planning_context.launch.py or 
        https://docs.openvins.com/gs-tutorial.html#gs-tutorial-ros2 for more information.
        Arguments must be of type LaunchConfiguration or None, if None they will be derived from the launch context, in this
        case all the necessary parameters must be declared in the launch file.

        Args:
            ur_type:                   Type/series of used UR robot.
            description_path:          Full path to the URDF/XACRO description file.
            prefix:                 Prefix for tf, useful for multi-robot setup.
            description_package:       Name of the package containing the robot description.
            moveit_config_package:     Name of the package containing the MoveIt configuration.
            joint_limits_file:  Name of the file containing the joint limits configuration.
            kinematics_file:    Name of the file containing the kinematics configuration.
            description_semantic_path: Full path for the file containing the semantic description.
    """

    ur_type                   = ur_type_ if ur_type_ else                                     LaunchConfiguration("ur_type")
    description_path          = description_path_ if description_path_ else                   LaunchConfiguration("description_path")
    prefix                    = prefix_ if prefix_ else                                       LaunchConfiguration("prefix")
    description_package       = description_package_ if description_package_ else             LaunchConfiguration("description_package")
    moveit_config_package     = moveit_config_package_ if moveit_config_package_ else         LaunchConfiguration("moveit_config_package")
    joint_limits_file         = joint_limits_file_ if joint_limits_file_ else                 LaunchConfiguration("joint_limits_file")
    kinematics_file           = kinematics_file_ if kinematics_file_ else                     LaunchConfiguration("kinematics_file")
    description_semantic_path = description_semantic_path_ if description_semantic_path_ else LaunchConfiguration("description_semantic_path")
    xacro_args                = xacro_args if xacro_args else                                 LaunchConfiguration("xacro_args")

    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, joint_limits_file]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, kinematics_file]
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
            prefix,
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            xacro_args.perform(context)
        ]
    )

    robot_description = {"robot_description": str(robot_description_content.perform(context))}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_semantic_path,
            " ",
            "name:=ur",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            " ",
            "prefix:=",
            prefix,
            " ",
            xacro_args.perform(context)
        ]
    )

    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    robot_description_kinematics = load_yaml(
        str(moveit_config_package.perform(context)),
        os.path.join("config", "kinematics.yaml"),
    )

    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)),
            os.path.join("config", "joint_limits.yaml"),
        )
    }

    # PIPELINES
    ompl_planning_yaml = load_yaml(
        str(moveit_config_package.perform(context)),
        os.path.join("config", "ompl_planning.yaml"),
    )

    pilz_planner_config_yaml = load_yaml(
        str(moveit_config_package.perform(context)),
        os.path.join("config", "pilz_industrial_motion_planner.yaml"),
    )

    pilz_cartesian_limits_yaml = load_yaml(
        str(moveit_config_package.perform(context)),
        os.path.join("config", "pilz_cartesian_limits.yaml"),
    )

    robot_description_planning["robot_description_planning"].update(pilz_cartesian_limits_yaml)

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

    capabilities = {
        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceService"
    }

    params = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        {"ompl": ompl_planning_yaml},
        {"pilz_industrial_motion_planner": pilz_planner_config_yaml},
        moveit_controllers,
        trajectory_execution,
        {
            'planning_pipelines': ['ompl', 'pilz_industrial_motion_planner'],
            'default_planning_pipeline': 'pilz_industrial_motion_planner',
        },
        capabilities
    ]

    return params
