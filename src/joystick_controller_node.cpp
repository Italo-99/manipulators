#include "manipulators/JoystickController.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

int main(int argc, char* argv[]) {    
    
    // ManipulatorMenuParams params;
    const std::string node_name = "joystick_controller_node";
    const std::string ns = "";

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(node_name, ns, options);

    ManipulatorMenuParams params(node);
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("manipulators");
    params.known_poses_path = package_share_directory + "/config/known_poses.yaml";

    node->declare_parameter("profile", "default");
    std::string profile = node->get_parameter("profile").as_string();

    auto controller = JoystickControllerFactory::fromProfile(
        profile, params, node, false
    );
    controller->spinnerJoystick();

    rclcpp::shutdown();

    return 0;
}