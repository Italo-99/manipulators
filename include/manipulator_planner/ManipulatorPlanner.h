/*
 * Software License Agreement (Apache Licence 2.0)
 *
 *  Copyright (c) [2024], [Andrea Pupa] [italo Almirante]
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
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: [Andrea Pupa] [Italo Almirante]
 *  Created on: [2024-01-17]
 */

// CLASS HEADER IMLEMENTATION OF MANIPULATOR PLANNER -> CHILD OF DYNAMIC PLANNER 

#ifndef MANIPULATOR_PLANNER_H
#define MANIPULATOR_PLANNER_H

// IMPORT LIBRARIES
#include "dynamic_planner/dynamic_planner.h"

class ManipulatorPlanner
{
 public:
  // ---------------------  PUBLIC CONSTRUCTOR ---------------------
      ManipulatorPlanner();     // Constructor
      ~ManipulatorPlanner();    // Destructor

  // ---------------------  PUBLIC FUNCTIONS ---------------------

    void spinner(void);       // Asynchronous spinner for ROS routines

    // Create a collision object from a selected primitive
    moveit_msgs::CollisionObject createObj(std::string& name, 
                                         uint8        obj_type, 
                                         float64      obj_dims, 
                                         float64      obj_pos, 
                                         bool         rot_90);

 private:

  // --------------------- PRIVATE FUNCTIONS ---------------------
    // --------------------- UTILS FUNCTIONS ---------------------

      void check_param(std::string manipulator_name,
                          double vel_factor, 
                          double acc_factor,
                            bool sim);

    // --------------------- MOVE FUNCTIONS ---------------------

      // Callback function for goals in the 3D cartesian space for the robot TCP
      void tcpGoalCallback(const geometry_msgs::Pose::ConstPtr& p);

      // Callback function for goals in the joint space
      void jointsGoalCallback(const sensor_msgs::JointState::ConstPtr& js);

      // Callback function for goals in the 3D cartesian space for the robot TCP
      void tcpGoalSeqCallback(std::vector<const geometry_msgs::Pose::ConstPtr>& p_seq);

      // Callback function for goals in the joint space
      void jointsGoalSeqCallback(std::vector<const sensor_msgs::JointState::ConstPtr>& js_seq);

  // ---------------------  PRIVATE VARIABLES ---------------------

    ros::NodeHandle nh_;                    // Node object
    ros::Subscriber tcp_goal_sub_;          // Subscriber to TCP goal
    ros::Subscriber joint_goal_sub_;        // Publisher to joint goal

    std::vector<std::string> joint_names_;  // Joints' names
    std::string ee_name_;                   // End-effector's name
    std::string base_name_;                 // Robot base's name
    DynamicPlanner* planner_;               // Dynamic planner object 
};

#endif /* MANIPULATOR_PLANNER_H */
