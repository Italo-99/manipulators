#include "manipulators/ManipulatorPlanner.h"

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    const rclcpp::NodeOptions node_options;
    //IMPORTANT: The node name must be <manipulator_name>_planner
    auto node = std::make_shared<ManipulatorPlannerNode>("manipulator_planner", node_options);


    node->spinner();

    rclcpp::shutdown();

    return 0;
}