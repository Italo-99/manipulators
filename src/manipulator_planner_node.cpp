#include "manipulators/ManipulatorPlanner.h"
#include <getopt.h>


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.allow_undeclared_parameters(true);

    auto node = std::make_shared<ManipulatorPlannerNode>("manipulator_planner", node_options);

    node->spinner();

    rclcpp::shutdown();

    return 0;
}
