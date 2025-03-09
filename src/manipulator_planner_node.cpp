#include "manipulators/ManipulatorPlanner.h"
#include <getopt.h>


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    const rclcpp::NodeOptions node_options;
    auto node = std::make_shared<ManipulatorPlannerNode>("manipulator_planner", node_options);

    node->spinner();

    rclcpp::shutdown();

    return 0;
}