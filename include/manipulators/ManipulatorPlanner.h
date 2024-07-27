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
#include <tf2_ros/transform_listener.h>

class ManipulatorPlanner
{
 public:
  // ---------------------  PUBLIC CONSTRUCTOR ---------------------
      ManipulatorPlanner();                   // Constructor
      ~ManipulatorPlanner();                  // Destructor

  // ---------------------  PUBLIC FUNCTIONS ---------------------

    void spinner(void);       // Asynchronous spinner for ROS routines

 private:

  // --------------------- PRIVATE FUNCTIONS ---------------------
    // --------------------- UTILS FUNCTIONS ---------------------

      // Check parameters passed to the manipulator
      void check_param();

      // Create a collision object from a selected primitive
      void createObj(const std::string& name,
                     const int            obj_type, 
                     std::vector<double>  obj_dims, 
                     double               obj_pos[], 
                     double               rot_pos[],
                     uint                 opoeration);

      // Callback function for goals in the 3D cartesian space for the robot TCP
      void addCollObjCallback(const moveit_msgs::CollisionObject& obj);

      // Get manipulator ee pose thourgh FKINE of current joints pose
      const geometry_msgs::PoseStamped get_manip_FKine(void);

      // Get manipulator Jacobian
      const Eigen::MatrixXd get_manip_Jacobian(void);

    // --------------------- MOVE FUNCTIONS ---------------------

      // Callback function for goals in the 3D cartesian space for the robot TCP
      void tcpGoalCallback(const geometry_msgs::Pose::ConstPtr& p);
      // Callback function for goals in the 3D cartesian space for the robot TCP computed through Inverse Kinematics
      void tcpGoalIKCallback(const geometry_msgs::Pose::ConstPtr& p);
      // Callback function for moveit fake controller of the TCP computed through Inverse Kinematics
      void tcpGoalIK_NoPlanner_Callback(const geometry_msgs::Pose::ConstPtr& p);
      // Callback function for goals in the joint space
      void jointsGoalCallback(const sensor_msgs::JointState::ConstPtr& js);
      // Callback function for goals as carthesian move
      void cartesianMoveCallback(const geometry_msgs::PoseArray::ConstPtr& p_seq);

      // // Callback function for goals in the 3D cartesian space for the robot TCP
      // void tcpGoalSeqCallback(const std::vector<geometry_msgs::Pose>& p_seq);

      // // Callback function for goals in the joint space
      // void jointsGoalSeqCallback(const std::vector<std::vector<double>>& js_seq);

  // ---------------------  PRIVATE VARIABLES ---------------------

    ros::NodeHandle nh_;                    // Node object
    ros::Subscriber tcp_goal_sub_;          // Subscriber to TCP goal
    ros::Subscriber joint_goal_sub_;        // Subscriber to joint goal
    ros::Subscriber tcp_goalIK_sub_;        // Subscriber to TCP goal with InvKine
    ros::Subscriber tcp_goalIK_noplan_sub_; // Subscriber to TCP goal with InvKine fake controller
    ros::Subscriber carthesian_move_sub_;   // Subscriber to carthesian move
    // ros::Subscriber tcp_goalSeq_sub_;    // Subscriber to the sequence of TCP goal
    // ros::Subscriber joint_goalSeq_sub_;  // Subscriber to the sequence of joint goal
    ros::Subscriber add_coll_obj_sub_;      // Subscriber to add a collision object

    ros::Publisher jointGoalFake_pub_;      // Fake joint state publisher

    // Planner args
    std::string manipulator_name_;          // Manipulator name
    double vel_factor_, acc_factor_;        // Scale factor for joint velocities and accelerations
    bool sim_;

    std::vector<std::string> joint_names_;  // Joints' names
    std::string ee_name_;                   // End-effector's name
    std::string base_name_;                 // Robot base's name
    DynamicPlanner* planner_;               // Dynamic planner object 
    bool tcp_pub_;                          // True if this instance has to publish ee_pos, else false 
};

#endif /* MANIPULATOR_PLANNER_H */
