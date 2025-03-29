#include "manipulators/DriverTrajectoryConverter.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    const rclcpp::NodeOptions node_options;
    auto node = std::make_shared<DriverTrajectoryConverter>("driver_trajectory_converter_node", node_options);

    RCLCPP_INFO(node->get_logger(), "Driver Trajectory Converter initialized successfully.");

    node->spinner();

    return 0;
}