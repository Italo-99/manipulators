#include "manipulators/ManipulatorRoutines.h"

int main(int argc, char* argv[]) {

    
    ManipulatorMenuParams params;
    params.node_name          = "manipulator_menu_node_nouser";
    params.ros_freq           = 50;
    params.manipulator_name   = "manipulator";
    params.planning_group     = "ur_manipulator";

    params.gripper            = "toolIO";
    params.gripper_IO_cmds    = {1, 0}; // For gripper type "real_gripper", IO commands to close/open the gripper

    params.joint_names        = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    params.base_link_name     = "base_link";

    params.joint_tolerance      = 0.01;
    params.tcp_position_tolerance = 0.01;
    params.tcp_orientation_tolerance = 0.01;

    params.known_poses_path = "/home/matteo/projectred_ws/src/manipulators/config/known_poses.yaml"; 

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(params.node_name, options);

    auto routines = std::make_shared<ManipulatorRoutines>(node, params);

    routines->routinesSpinnerMenu();

    return 0;
}