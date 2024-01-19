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

// CLASS HEADER IMLEMENTATION OF MANIPULATOR MENU

#ifndef MANIPULATOR_MENU_H
#define MANIPULATOR_MENU_H

// IMPORT LIBRARIES
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "moveit_msgs/CollisionObject.h"
#include <tf2/LinearMath/Quaternion.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>

class ManipulatorMenu
{
 public:
  // ---------------------  PUBLIC CONSTRUCTOR ---------------------
      ManipulatorMenu();     // Constructor

  // ---------------------  PUBLIC FUNCTIONS ---------------------

    void spinner(void);                                       // Asynchronous spinner for ROS routines

    void publishJointGoal(const std::vector<double> joints);  // publish a joint goal to the manipulator planner
    void publishTcpGoal(const std::vector<double> position);  // publish a tcp   goal to the manipulator planner


 private:

  // --------------------- PRIVATE FUNCTIONS ---------------------

    // ---------------------  PRIVATE PUBS/SUBS ---------------------

      void publishCollisionObject();    // add a collision object to the manipulator scene
      void jointStateVisualizer();      // listen to joint state publisher

    // --------------------- MOVE FUNCTIONS ---------------------

      void testJointGoal(void);   // to test a joint goal
      void userJointGoal(void);   // to perform a joint goal set by the user 

      void testTcpGoal(void);     // to test a tcp goal
      void userTcpGoal(void);     // to perform a tcp goal set by the user 

      // Joint state callback function
      void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    
    // --------------------- UTILS FUNCTIONS ---------------------

      // Create a collision object from a selected primitive
      void addObj(const std::string&   name, 
                  const int            obj_type, 
                  std::vector<double>  obj_dims, 
                  double               obj_pos[], 
                  double               rot_pos[]);

      // Function to add a collision object
      void addCollObj(const moveit_msgs::CollisionObject& obj);

      void printMenu();

      int getUserChoice();

      void processChoice(int choice);

  // ---------------------  PRIVATE VARIABLES ---------------------

    // ---------------------  ROS HANDLING ---------------------
      ros::NodeHandle nh_;
      ros::Publisher  jointStatePublisher_;
      ros::Publisher  tcpPosePublisher_;
      ros::Publisher  collisionObjectPublisher_;
      ros::Subscriber jointStateSubscriber_;
      
      ros::Publisher display_goal_pub_;

    // ---------------------  USEFUL TOOLS ---------------------

      bool counterJg_;
      bool counterCg_; 
};

#endif /* MANIPULATOR_MENU_H */
