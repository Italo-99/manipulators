#include "manipulators/DriverTrajectoryConverter.h"

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    const rclcpp::NodeOptions node_options;
    auto node = std::make_shared<DriverTrajectoryConverter>("driver_trajectory_converter_node", node_options);

    // Only compute if both joint state and command maps are initialized
    rclcpp::Rate wait_rate(10);
    while(node->isReady())
    {
        rclcpp::spin_some(node);
        wait_rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Driver Trajectory Converter initialized successfully.");

    node->spinner();

    rclcpp::shutdown();

    return 0;
}