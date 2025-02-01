#include "manipulators/ManipulatorMenu.h"

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<ManipulatorMenu>("manipulator_menu", rclcpp::NodeOptions());
    node->spinner();

    rclcpp::shutdown();

    return 0;
}