#ifndef DRIVERTRAJECTORYCONVERTER_H
#define DRIVERTRAJECTORYCONVERTER_H
// Import libraries
#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <unordered_map>
#include <Eigen/Geometry>

// Class implementation
class DriverTrajectoryConverter : public rclcpp::Node
{
public:
    DriverTrajectoryConverter(std::string node_name, const rclcpp::NodeOptions &options);
    ~DriverTrajectoryConverter();
    void spinner();
    bool isReady();

private:
    // ROS objects
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr   joint_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr   joint_cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  velocity_publisher_;

    void declareParameters();

    // Computation of the average computation time
    static DriverTrajectoryConverter* instance__;
    static void static_shutdown_handler(int sig);
    void shutdown_handler();

    // ROS callbacks
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state);
    void jointCmdCallback(const sensor_msgs::msg::JointState::SharedPtr joint_cmd);

    // Controller implementation
    void computeVel();

    // Joint states
    Eigen::Matrix<double,6,1> joints_values_;       // Pos status
    Eigen::Matrix<double,6,1> dq_cmd_;              // Velocity command
    Eigen::Matrix<double,6,1> qd_cmd_;              // Position command
    Eigen::Matrix<double,6,1> real_vel_;            // Velocity output of the position
    std_msgs::msg::Float64MultiArray vel_msg_;      // Final multi array vel msg to publish

    // Joints mapping
    std::unordered_map<std::string, size_t> joint_name_to_index_;  // Map for joint names to indices

    bool joint_map_initialized_;  // Flag to check if the joint state map is initialized
    bool cmd_map_initialized_;    // Flag to check if the command state map is initialized
    
    double mean_; // Average value for the duration of the driver control computation
};

#endif // DRIVERTRAJECTORYCONVERTER_H
