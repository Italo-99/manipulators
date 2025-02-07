#include "manipulators/DriverTrajectoryConverter.h"

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    const rclcpp::NodeOptions node_options;
    auto node = std::make_shared<DriverTrajectoryConverter>("driver_trajectory_converter_node", node_options);

    node->spinner();

    rclcpp::shutdown();

    return 0;
}