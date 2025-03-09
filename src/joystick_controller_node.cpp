#include "manipulators/JoystickController.h"

int main(int argc, char* argv[]) {

    
    ManipulatorMenuParams params;
    params.node_name          = "manipulator_menu_node_nouser";
    params.ros_freq           = 10.;
    params.manipulator_name   = "manipulator";
    params.planning_group     = "ur_manipulator";

    params.gripper            = true;

    params.joint_names        = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    params.base_link_name     = "base_link";
    
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto controller = std::make_shared<JoystickController>(params.manipulator_name + "_joystick_controller", params);
    controller->spinner();

    rclcpp::shutdown();

    return 0;
}