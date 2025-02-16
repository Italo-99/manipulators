#include "manipulators/ManipulatorMenu.h"

int main(int argc, char* argv[]) {

    
    ManipulatorMenuParams params;
    params.node_name          = "manipulator_menu_node_nouser";
    params.ee_joint_name      = "";
    params.ros_freq           = 10.;
    params.manipulator_name   = "manipulator";
    params.enable_coppelia    = false;
    params.enable_sim_gripper = false;
    params.enable_real_gripper= false;
    params.gripper_topic      = "/ur_rtde/robotiq_gripper/command";
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