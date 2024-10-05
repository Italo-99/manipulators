/*
 * Software License Agreement (Apache Licence 2.0)
 *
 *  Copyright (c) [2024], [Italo Almirante]
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *   3. The name of the author may not be used to endorse or promote
 *      products derived from this software without specific prior
 *      written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: [Italo Almirante] 
 *  Created on: [2024-10-05]
 */

// Import libraries
#include "manipulators/DriverTrajectoryConverter.h"

double DriverTrajectoryConverter::mean_ = 0.0;

// Constructor
DriverTrajectoryConverter::DriverTrajectoryConverter(std::string node_name)
    : joint_map_initialized_(false), cmd_map_initialized_(false)
{
    nh_.param<std::string>(node_name+"/velocity_topic", vel_topic_, "/ur_rtde/controllers/joint_velocity_controller/command");
    nh_.param<double>(node_name+"/kp", kp_, 4.0);
    nh_.param<double>(node_name+"/spinner_rate", spinner_rate_, 500.0);
    nh_.param<double>(node_name+"/min_motor_speed", min_motor_speed_, 0.001);

    // Load joint names group (from launch file)
    if (!nh_.getParam(node_name+"/joints_names_group", joints_names_group_))
    {
        ROS_ERROR("Failed to get 'joints_names_group' parameter. Shutting down...");
        ros::shutdown();
    }

    // Setup the joint name to index map
    for (size_t i = 0; i < joints_names_group_.size(); ++i)
    {
        joint_name_to_index_[joints_names_group_[i]] = i;
        std::cout << "Found joint ready for the driver: " << joints_names_group_[i] << std::endl;
    }

    // Setup subscriber and publisher
    joint_state_sub_ = nh_.subscribe("/joint_states", 1, &DriverTrajectoryConverter::jointStateCallback, this);
    joint_cmd_sub_   = nh_.subscribe("/move_group/fake_controller_joint_states", 1, &DriverTrajectoryConverter::jointCmdCallback, this);
    velocity_publisher_ = nh_.advertise<std_msgs::Float64MultiArray>(vel_topic_, 1);

    // Initialize Eigen matrices to zero
    joints_values_.setZero();
    dq_cmd_.setZero();
    qd_cmd_.setZero();
    real_vel_.setZero();
    vel_msg_.data.resize(6);

    // Only compute if both joint state and command maps are initialized
    ros::Rate wait_rate(10);
    while(!isReady())
    {
        ros::spinOnce();
        wait_rate.sleep();
    }

    ROS_INFO("Driver Trajectory Converter initialized successfully.");
}

// Shutdown handler
void DriverTrajectoryConverter::shutdown_handler(int sig)
{
    // Show the result of the jacobian control mean duration
    ROS_INFO("Mean duration of real driver control computations: %f seconds", mean_);
    ros::Duration(1.0).sleep();

    // Shutdown ROS
    ros::shutdown();
}

// Check if both joint state and command maps are initialized
bool DriverTrajectoryConverter::isReady()
{
    return joint_map_initialized_ && cmd_map_initialized_;
}

// Callback to receive actual joint states
void DriverTrajectoryConverter::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joints_state)
{
    uint counter_group = 0;

    // Loop through the received joint states
    for (uint i = 0; i < joints_state->name.size(); i++)
    {
        const std::string& joint_name = joints_state->name[i];
        auto it = joint_name_to_index_.find(joint_name);
        // std::cout << "Iterator 1: " << it->first << std::endl;  // Joint name
        // std::cout << "Iterator 2: " << it->second << std::endl; // Joint position in the map
        if (it != joint_name_to_index_.end())
        {
            size_t index = it->second;
            joints_values_[index] = joints_state->position[i];
            counter_group++;
        }
    }

    // If all joints have been updated, mark as initialized
    if (counter_group >= joints_names_group_.size())
    {
        joint_map_initialized_ = true;
    }
}

// Callback to receive the fake controller joint states (commands)
void DriverTrajectoryConverter::jointCmdCallback(const sensor_msgs::JointState::ConstPtr& cmd_state)
{
    uint counter_group = 0;

    // Loop through the received joint commands
    for (uint i = 0; i < cmd_state->name.size(); i++)
    {        
        const std::string& joint_name = cmd_state->name[i];
        auto it = joint_name_to_index_.find(joint_name);
        // std::cout << "Iterator 1: " << it->first << std::endl;  // Joint name
        // std::cout << "Iterator 2: " << it->second << std::endl; // Joint position in the map
        if (it != joint_name_to_index_.end())
        {
            size_t index = it->second;
            qd_cmd_[index] = cmd_state->position[i];
            if (cmd_state->velocity.size() > index)
                {dq_cmd_[index] = cmd_state->velocity[i];}
            counter_group++;
        }
    }

    // If all joints have been updated, mark as initialized
    if (counter_group >= joints_names_group_.size())
    {
        cmd_map_initialized_ = true;
    }
}

// Compute the velocity command using the proportional control
void DriverTrajectoryConverter::computeVel()
{
    // Compute the velocity output: real_vel_ = dq_cmd_ + kp_ * (qd_cmd_ - joints_values_)
    real_vel_ = dq_cmd_ + kp_ * (qd_cmd_ - joints_values_);

    // Apply minimum velocity threshold and prepare velocity message
    for (size_t i = 0; i < 6; ++i)
    {
        // Apply minimum velocity threshold
        if (std::abs(real_vel_[i]) < min_motor_speed_)
        {
            real_vel_[i] = 0.0;
        }

        // Set the velocity message
        vel_msg_.data[i] = real_vel_[i];
    }

    // Publish velocity command to the robot
    velocity_publisher_.publish(vel_msg_);
}

// Spinner to continuously call callbacks and compute velocity
void DriverTrajectoryConverter::spinner()
{
    // Number of samples for mean computation
    unsigned long long int k = 0;
    signal(SIGINT, shutdown_handler);
    ros::Rate rate(spinner_rate_);

    while (ros::ok())
    {
        ros::spinOnce();                    // Process callbacks
        ros::Time start = ros::Time::now(); // Start loop time measurement
        computeVel();                       // Compute velocity after every callback cycle

        // Update mean computation
        double sample_k = (ros::Time::now() - start).toSec();
        k++;
        mean_ = (1.0 / static_cast<double>(k)) * (sample_k + mean_ * static_cast<double>(k - 1));

        // Sleep according to the defined spinner rate
        rate.sleep();
    }

    // Publish zero velocities when shutting down
    std_msgs::Float64MultiArray zero_vel;
    zero_vel.data.resize(6, 0.0);
    velocity_publisher_.publish(zero_vel);
    ros::Duration(1.0).sleep();
}