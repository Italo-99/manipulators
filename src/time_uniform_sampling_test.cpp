#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

class TestUniformPlanner : public rclcpp::Node
{
public:
    TestUniformPlanner() : Node("time_uniform_sampling_test")
    {

        // FOLLOW THE MOVEIT TUTORIAL!!
        // https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/robot_model_and_robot_state/src/robot_model_and_robot_state_tutorial.cpp

        // Publisher to /move_group/fake_controller_joint_states
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/move_group/fake_controller_joint_states", 1);

        // Log initialization
        RCLCPP_INFO(this->get_logger(), "Initializing...");

        // Initialize the robot model loader correctly
        // robot_model_loader_ = robot_model_loader::RobotModelLoader(
        //     this->shared_from_this(), "/robot_description");

        // Load the robot model
        // robot_model = robot_model_loader_->getModel();
        robot_model_loader::RobotModelLoader robot_model_loader(Node);
        const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
        if (!robot_model)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load robot model");
            return;
        }
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    robot_model_loader::RobotModelLoader robot_model_loader_(Node);
    const moveit::core::RobotModelPtr& robot_model = nullptr;

    void planTo3DPosition()
    {
        // Define joint names for UR5e
        const std::vector<std::string> joint_names = {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

        // Define a dummy trajectory moving to a 3D position
        trajectory_msgs::msg::JointTrajectory joint_trajectory;
        joint_trajectory.joint_names = joint_names;

        // Start position (home position)
        trajectory_msgs::msg::JointTrajectoryPoint start_point;
        start_point.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        start_point.time_from_start.sec = 0;

        // Goal position for a 3D target
        trajectory_msgs::msg::JointTrajectoryPoint goal_point;
        goal_point.positions = {1.57, -1.57, 1.57, 0.0, -1.57, 0.0}; // Replace with calculated IK results
        goal_point.time_from_start.sec = 3;

        joint_trajectory.points.push_back(start_point);
        joint_trajectory.points.push_back(goal_point);

        // Process the trajectory with TimeOptimalTrajectoryGeneration
        processTrajectory(joint_trajectory);
    }

    void processTrajectory(const trajectory_msgs::msg::JointTrajectory &joint_trajectory)
    {
        auto robot_model = planning_scene_->getRobotModel();
        robot_trajectory::RobotTrajectory robot_trajectory(robot_model, "manipulator");

        // Convert ROS trajectory to RobotTrajectory
        moveit_msgs::msg::RobotTrajectory robot_trajectory_msg;
        robot_trajectory.getRobotTrajectoryMsg(robot_trajectory_msg);

        // Set the trajectory msg to RobotTrajectory
        robot_trajectory.setRobotTrajectoryMsg(planning_scene_->getCurrentState(), joint_trajectory);

        // Time-optimal trajectory generation
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        bool success = totg.computeTimeStamps(robot_trajectory);

        //READ AT: https://moveit.picknik.ai/main/api/html/classtrajectory__processing_1_1TimeOptimalTrajectoryGeneration.html

        // computeTimeStamps(robot_trajectory::RobotTrajectory &trajectory, const double max_velocity_scaling_factor=1.0,
        //                    const double max_acceleration_scaling_factor=1.0) const override

        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute time-optimal trajectory");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Time-optimal trajectory generated successfully");

        // Publish the resampled trajectory points
        rclcpp::Rate rate(10); // 0.1 seconds interval
        for (const auto &point : robot_trajectory.getTrajectory().getWayPointList())
        {
            RCLCPP_INFO(this->get_logger(), "Time: %f, Positions: [%f, %f, %f, %f, %f, %f]",
                        point.time_from_start.sec + point.time_from_start.nanosec * 1e-9,
                        point.positions[0], point.positions[1], point.positions[2],
                        point.positions[3], point.positions[4], point.positions[5]);

            // Create a JointState message
            auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();
            joint_state_msg->header.stamp = this->now();
            joint_state_msg->name = joint_trajectory.joint_names;
            joint_state_msg->position = point.positions;

            // Publish the joint state
            joint_state_pub_->publish(*joint_state_msg);

            // Sleep for the interval
            rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestUniformPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
