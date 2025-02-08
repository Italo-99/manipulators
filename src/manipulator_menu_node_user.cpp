#include "manipulators/ManipulatorMenu.h"

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("manipulator_menu", rclcpp::NodeOptions());

    auto menu = std::make_shared<ManipulatorMenu>(node);
    menu->spinnerMenu();

    rclcpp::shutdown();

    return 0;
}