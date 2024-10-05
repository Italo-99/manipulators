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

#ifndef DRIVERTRAJECTORYCONVERTER_H
#define DRIVERTRAJECTORYCONVERTER_H

// Import libraries
#include <signal.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <unordered_map>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>

// Class implementation
class DriverTrajectoryConverter
{
public:
    DriverTrajectoryConverter(std::string node_name);
    void spinner();

private:
    // ROS objects
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber joint_cmd_sub_;
    ros::Publisher  velocity_publisher_;

    // Node params
    std::string               vel_topic_;
    double                    kp_;
    double                    spinner_rate_;
    double                    min_motor_speed_;
    std::vector<std::string>  joints_names_group_;

    // Computation of the average computation time
    static void shutdown_handler(int sig);
    static double mean_; // Average value for the duration of the driver control computation

    // ROS callbacks
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr&);
    void jointCmdCallback(const sensor_msgs::JointState::ConstPtr&);

    // Controller implementation
    void computeVel();

    // Joint states
    Eigen::Matrix<double,6,1> joints_values_; // Pos status
    Eigen::Matrix<double,6,1> dq_cmd_;        // Velocity command
    Eigen::Matrix<double,6,1> qd_cmd_;        // Position command
    Eigen::Matrix<double,6,1> real_vel_;      // Velocity output of the position
    std_msgs::Float64MultiArray vel_msg_;      // Final multi array vel msg to publish

    // Joints mapping
    std::unordered_map<std::string, size_t> joint_name_to_index_;  // Map for joint names to indices

    bool joint_map_initialized_;  // Flag to check if the joint state map is initialized
    bool cmd_map_initialized_;    // Flag to check if the command state map is initialized

    bool isReady();
};

#endif // DRIVERTRAJECTORYCONVERTER_H
