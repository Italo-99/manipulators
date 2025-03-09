#include "manipulators/ManipulatorMenu.h"

int main(int argc, char* argv[]) {

    
    ManipulatorMenuParams params;
    params.node_name          = "manipulator_menu_node_user";
    params.ros_freq           = 50;
    params.manipulator_name   = "manipulator";
    params.planning_group     = "ur_manipulator";

    params.gripper            = true;

    params.joint_names        = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    params.base_link_name     = "base_link";
    
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(params.node_name, options);

    auto menu = std::make_shared<ManipulatorMenu>(params, node);
    menu->spinnerMenu();

    rclcpp::shutdown();

    return 0;
}