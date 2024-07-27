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
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "moveit_msgs/CollisionObject.h"
#include <tf2/LinearMath/Quaternion.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/robot_state/conversions.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "manipulators/CoppeliaMenu.h"
#include "gripper/RobotiQGripperControl.h"
#include "std_srvs/SetBool.h"

class ManipulatorMenu
{
 public:
  // ---------------------  PUBLIC CONSTRUCTOR ---------------------
    ManipulatorMenu(std::string gripper_joint_name,
                    std::string last_robot_link); // Constructor
    // ~ManipulatorMenu();                    // Destructor

  // ---------------------  PUBLIC FUNCTIONS ---------------------

    // Spinner
      void spinnerMenu(void);         // Asynchronous spinner for ROS routines with user menu
      void spinner(void);             // Update current robot joints state

    // Coppelia
      void startCoppeliaSim(void);        // Start simulation on CoppeliaSim
      void stopCoppeliaSim(void);         // Stop  simulation on CoppeliaSim
      void saveCoppeliaScene(void);       // Save  scene      on CoppeliaSim
      void changeCoppeliaCablePose(void); // Changle randomly cable pose on CoppeliaSim

    // Joint and TCP moves
      sensor_msgs::JointState publishJointGoal(const std::vector<double> joints);  // publish a joint goal to the manipulator planner
      sensor_msgs::JointState publishJointGoal(const sensor_msgs::JointState jointStateMsg);
      geometry_msgs::Pose publishTcpGoal(const std::vector<double> position);  // publish a tcp   goal to the manipulator planner
      geometry_msgs::Pose publishTcpGoal(const geometry_msgs::Pose tcpPoseMsg);
      geometry_msgs::Pose publishTcpIKGoal(const std::vector<double> position);// publish a tcpIK goal to the manipulator planne
      geometry_msgs::Pose publishTcpIKGoal(const geometry_msgs::Pose tcpPoseMsg); 
      geometry_msgs::Pose publishCartesianMove(const uint   axis1,  // publish a carthesian move command
                                              const uint   axis2,
                                              const double pos1,
                                              const double pos2,
                                              const uint   steps);
      sensor_msgs::JointState oneJointMove(const int num, const double joint_rot); // to define a rotation around a single joint
      sensor_msgs::JointState goHome(const bool);               // to setup home position

    // Get the position and orientation of the end effector (they contain a ros spin once)
      geometry_msgs::PoseStamped getEEpose();
      std::vector<double> getEEpos_rpy();

    // Get the transform between two frames
      geometry_msgs::PoseStamped getTf(const std::string& source_frame, const std::string& target_frame);

    // Move along axes
      geometry_msgs::Pose move_along_x(const double x_step);
      geometry_msgs::Pose move_along_y(const double y_step);
      geometry_msgs::Pose move_along_z(const double z_step);

    // Tcp orientation handling
      geometry_msgs::Pose make_tcp_rot(const std::vector<double> rot_vec);
      geometry_msgs::Pose rotate_around_x(const double x_rot_step);
      geometry_msgs::Pose rotate_around_y(const double y_rot_step);
      geometry_msgs::Pose rotate_around_z(const double z_rot_step);
      geometry_msgs::Pose change_tcp_orient(const std::vector<double> rot_vec);

    // Add collision objects
      void publishCollisionObject(const moveit_msgs::CollisionObject collisionObjectMsg);
      void addObj(const std::string& name,
                const int            obj_type, 
                std::vector<double>  obj_dims, 
                double               obj_pos[], 
                double               rot_pos[],
                uint                 operation);

    // Gripper control
      void openGripper(void);
      void closeGripper(void);
      void moveGripper(const double);
      void grabObjGripper(void);
      void detachObjGripper(void);
      void openRealGripper(void);
      void closeRealGripper(void);
      void moveRealGripper(const float);
      
    // Quaternions handling
      geometry_msgs::Quaternion quaternion_from_euler(double roll, double pitch, double yaw);
      std::vector<double> euler_from_quaternion(const geometry_msgs::Quaternion quat);

    // Degrees and radians conversions
      std::vector<double> deg_from_rad(const std::vector<double>);
      std::vector<double> rad_from_deg(const std::vector<double>);      

    // Joy handlers
      
 
 private:

  // --------------------- PRIVATE FUNCTIONS ---------------------

    // ---------------  PRIVATE COPPELIA METHODS ---------------------
      void wait_for_response(void);     // Send the request and show the response

    // --------------------- PRIVATE PUBS/SUBS ---------------------

      void jointStateVisualizer();      // listen to joint state publisher

    // --------------------- MOVE FUNCTIONS ---------------------

      void testJointGoal(void);   // to test a joint goal
      void userJointGoal(void);   // to perform a joint goal set by the user 
      void oneJointMove_user();   // to move only a single joint

      void testTcpGoal(void);       // to test a tcp goal
      void userTcpGoal(void);       // to perform a tcp goal set by the user 
      void userTcpIKGoal(void);     // to perform a tcpIK goal set by the user 
      void userCartesianMove(void); // to perform a cartesian move set by the user

      // Joint state callback function
      void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

      void userGripperMove(void);          // to perform a gripper move set by the user
      void callGripperSrv(const bool);     // to call open/close gripper srv
      void callGrabbingSrv(const bool);    // to call grab/detach gripper srv
      void callRealGripperSrv(const float);// to call real gripper open close

    // --------------------- UTILS FUNCTIONS ---------------------

      // Function to add a collision object by the user
      void addCollObj(void);
      // Function to delete a given collision object from the user menu
      void deleteCollObj(void);

      //Menu handling
      void printMenu();
      int getUserChoice();
      void processChoice(int choice);

  // --------------------- PRIVATE VARIABLES ---------------------

    // ---------------------  GRIPPER HANDLER -------------
      std::string gripper_joint_name_;
      std::string last_robot_link_;
      double ee_offset_;

    // ---------------------  ROS HANDLING ---------------------
      ros::NodeHandle nh_;
      ros::Publisher  jointGoalPublisher_;
      ros::Publisher  tcpPosePublisher_;
      ros::Publisher  tcpPoseIKPublisher_;
      ros::Publisher  carthesianMovePublisher_;
      ros::Publisher  display_goal_pub_;
      ros::Publisher  eepose_pub_;
      ros::Publisher  collisionObjectPublisher_;
      ros::Publisher  moveGripperPublisher_;
      ros::Subscriber jointStateSubscriber_;

      geometry_msgs::PoseStamped current_tcp_pose_;
      sensor_msgs::JointState current_joint_pose_;

      // ---------------------  COPPELIA HANDLING ---------------------
        ros::ServiceClient client_;
        manipulators::CoppeliaMenu coppelia_srv_;

      // ---------------------  GRIPPER HANDLING ---------------------
        ros::ServiceClient gripper_client_;
        ros::ServiceClient grab_client_;
        ros::ServiceClient real_gripper_client_;

    // ---------------------  USEFUL TOOLS ---------------------

      bool counterJg_;
      bool counterCg_; 
};

#endif /* MANIPULATOR_MENU_H */
