#include "manipulators/ManipulatorMenu.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

int main(int argc, char* argv[]) {    
    const std::string prefix = "";
    const std::string ns = "";

    ManipulatorMenuParams params;
    params.node_name          = "manipulator_menu_node_user";
    params.ros_freq           = 10;
    params.manipulator_name   = "manipulator";
    params.planning_group     = prefix + "ur_manipulator";

    params.gripper            = "toolIO";           // Gripper type, can be "robotiq_85" for 'motor_mover' integration or "toolIO"
    params.gripper_group      = "robotiq_85_gripper";   // For gripper type "robotiq_85"
    params.gripper_IO_cmds    = {1, 0};                 // For gripper type "toolIO", IO commands to close/open the gripper

    params.joint_names        = {prefix + "shoulder_pan_joint", 
                                 prefix + "shoulder_lift_joint", 
                                 prefix + "elbow_joint",
                                 prefix + "wrist_1_joint", 
                                 prefix + "wrist_2_joint", 
                                 prefix + "wrist_3_joint"};

    params.base_link_name     = prefix + "base_link";

    // Set the path to the known poses YAML file
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("manipulators");
    params.known_poses_path = package_share_directory + "/config/known_poses.yaml";
    
    // std::string package_share_directory = ament_index_cpp::get_package_share_directory("sirio_manipulator");
    // params.known_poses_path = package_share_directory + "/config/menu/known_poses_ur.yaml";

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(params.node_name, ns, options);

    auto menu = std::make_shared<ManipulatorMenu>(params, node, false);
    menu->spinnerMenu();

    rclcpp::shutdown();

    return 0;
}
