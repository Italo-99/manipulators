#include "manipulators/JoystickController.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

int main(int argc, char* argv[]) {    
    
    // ManipulatorMenuParams params;
    const std::string node_name = "joystick_controller_node";
    const std::string ns = "";
    // params.ros_freq           = 10;
    // params.manipulator_name   = "manipulator";
    // params.planning_group     = prefix + "ur_manipulator";

    // params.gripper            = "robotiq_85";           // Gripper type, can be "robotiq_85" for 'motor_mover' integration or "toolIO"
    // params.gripper_group      = "robotiq_85_gripper";   // For gripper type "robotiq_85"
    // params.gripper_IO_cmds    = {1, 0};                 // For gripper type "toolIO", IO commands to close/open the gripper

    // params.joint_names        = {prefix + "shoulder_pan_joint", 
    //                              prefix + "shoulder_lift_joint", 
    //                              prefix + "elbow_joint",
    //                              prefix + "wrist_1_joint", 
    //                              prefix + "wrist_2_joint", 
    //                              prefix + "wrist_3_joint"};

    // params.base_link_name     = prefix + "base_link";

    // Set the path to the known poses YAML file

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(node_name, ns, options);

    ManipulatorMenuParams params(node);
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("manipulators");
    params.known_poses_path = package_share_directory + "/config/known_poses.yaml";

    auto controller = std::make_shared<JoystickController>(params, node, false);
    controller->spinnerJoystick();

    rclcpp::shutdown();

    return 0;
}