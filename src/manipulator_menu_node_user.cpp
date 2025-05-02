#include "manipulators/ManipulatorMenu.h"

int main(int argc, char* argv[]) {    
    const std::string prefix = "";
    const std::string ns = "";

    ManipulatorMenuParams params;
    params.node_name          = "manipulator_menu_node_user";
    params.ros_freq           = 50;
    params.manipulator_name   = "manipulator";
    params.planning_group     = prefix + "ur_manipulator";

    params.gripper            = "real_gripper";
    params.gripper_group      = prefix + "robotiq_85_gripper";
    params.gripper_IO_cmds    = {1, 2}; // For gripper type "real_gripper", IO commands to close/open the gripper

    params.joint_names        = {prefix + "", 
                                 prefix + "", 
                                 prefix + "",
                                 prefix + "", 
                                 prefix + "", 
                                 prefix + ""};

    params.base_link_name     = prefix + "base_link";

    params.known_poses_path = "/home/matteo/projectred_ws/src/manipulators/config/known_poses.yaml";

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(params.node_name, ns, options);

    auto menu = std::make_shared<ManipulatorMenu>(params, node, true);
    menu->spinnerMenu();

    rclcpp::shutdown();

    return 0;
}