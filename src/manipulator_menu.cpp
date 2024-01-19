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

// CLASS SOURCE IMLEMENTATION OF MANIPULATOR MENU
// This file is useful to publish some msgs on the topics of a manipulator planner instance

// TODO: together with node launch, UR10 should go immediatly to a preconfigured configuration
// initial position {0,-90,+90,-90,-90,0}

// IMPORT LIBRARIES
#include "manipulators/ManipulatorMenu.h"

// --------------------- PUBLIC CONSTRUCTOR ---------------------

ManipulatorMenu::ManipulatorMenu()
{
    // --------------------- PUBS & SUBS DELCARATIONS ---------------------

    jointStatePublisher_      = nh_.advertise<sensor_msgs::JointState>("/desired_joint_pose", 1);
    tcpPosePublisher_         = nh_.advertise<geometry_msgs::Pose>("/desired_tcp_pose", 1);
    collisionObjectPublisher_ = nh_.advertise<moveit_msgs::CollisionObject>("/add_collision_object", 1);
    jointStateSubscriber_     = nh_.subscribe("/joint_states", 1, &ManipulatorMenu::jointStateCallback, this);

    // --------------------- Global class variables init ---------------------

    counterJg_ = false;       // choice of test joint goal
    counterCg_ = false;       // choice of test tcp3D goal
}

// --------------------- PUBLIC FUNCTIONS ---------------------

// Asynchronous spinner for ROS routines
void ManipulatorMenu::spinner()
{
  int userChoice = 0;

  while (ros::ok())
  {
    printMenu();
    userChoice = getUserChoice();
    processChoice(userChoice);
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  ROS_WARN_THROTTLE(3, "Closing the menu! \n");
  ros::shutdown();
}

// --------------------- PRIVATE FUNCTIONS ---------------------
  
// --------------------- PUBS HANDLER ---------------------
  
// --------------------- JOINT GOALS HANDLER ---------------------

// void ManipulatorMenu::testJointGoal()
// {

// }

// void ManipulatorMenu::userJointGoal()
// {

// }

void ManipulatorMenu::publishJointGoal() 
{
  // Declare the empty vector of joints goals
  std::vector<double> joints = {0.,0.,0.,0.,0.,0.};

  // Alternate a different joint goal when launching this function
  counterJg_ = !counterJg_;
  if (counterJg_) {joints = {0.0,-1.57,+1.57,0.0,+1.57,0.0};}
  else            {joints = {0.0,-1.57,+1.57,0.0,-1.57,0.0};}   

  // Fill the joint msg
  sensor_msgs::JointState jointStateMsg;
  jointStateMsg.header.stamp = ros::Time::now();
  jointStateMsg.name.push_back("shoulder_pan_joint");
  jointStateMsg.name.push_back("shoulder_lift_joint");
  jointStateMsg.name.push_back("elbow_joint");
  jointStateMsg.name.push_back("wrist_1_joint");
  jointStateMsg.name.push_back("wrist_2_joint");
  jointStateMsg.name.push_back("wrist_3_joint");
  for (unsigned int k = 0; k < joints.size(); k++) {jointStateMsg.position.push_back(joints[k]); }

  // Publish the JointState message
  jointStatePublisher_.publish(jointStateMsg);
}

// --------------------- TCP GOALS HANDLER ---------------------

void ManipulatorMenu::publishTcpGoal() 
{
    geometry_msgs::Pose tcpPoseMsg;
    // Fill TCP pose message
    tcpPosePublisher_.publish(tcpPoseMsg);
}

void ManipulatorMenu::publishCollisionObject() 
{
    moveit_msgs::CollisionObject collisionObjectMsg;
    // Fill collision object message
    collisionObjectPublisher_.publish(collisionObjectMsg);
}

// --------------------- SUBS HANDLER ---------------------

void ManipulatorMenu::jointStateVisualizer() 
{
    // This method is called internally by the constructor
}

void ManipulatorMenu::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) 
{
    // Process incoming joint state message
}

// --------------------- COLLISION OBJECTS HANDLER ---------------------
// Create a collision object from a selected primitive
void ManipulatorMenu::addObj(const std::string&   name, 
            const int            obj_type, 
            std::vector<double>  obj_dims, 
            double               obj_pos[], 
            double               rot_pos[])
{
  
}

// Function to add a collision object
void ManipulatorMenu::addCollObj(const moveit_msgs::CollisionObject& obj)
{

}

// --------------------- MENU HANDLER ---------------------

void ManipulatorMenu::printMenu()
{
  std::cout << "======= Manipulator Menu =======\n";
  std::cout << "1. Test a joint goal\n";
  std::cout << "2. Test a TCP goal\n";
  std::cout << "3. Give a joint goal\n";
  std::cout << "4. Give a TCP goal\n";
  std::cout << "5. Give a TCP goal through InvKine\n";
  std::cout << "6. Move a defined joint\n";
  std::cout << "7. Move the robot along x\n";
  std::cout << "8. Move the robot along y\n";
  std::cout << "9. Move the robot along z\n";
  std::cout << "10.Change TCP orientation\n";
  std::cout << "11.Add an object to the scene\n";
  std::cout << "12.Clear goal markers\n";
  std::cout << "13.Shutdown the menu\n";
  std::cout << "=====================\n";
}

int ManipulatorMenu::getUserChoice()
{
  int choice;
  std::cout << "Enter your choice: ";
  std::cin >> choice;
  return choice;
}

void ManipulatorMenu::processChoice(int choice)
{
  switch (choice)
  {
  case 1:
    ROS_INFO("You selected Option 1");
    publishJointGoal();
    break;

  case 2:
    ROS_INFO("You selected Option 2");
    break;

  case 3:
    ROS_INFO("You selected Option 3");
    break;

  case 4:
    ROS_INFO("You selected Option 4");
    break;

  case 5:
    ROS_INFO("You selected Option 5");
    break;

  case 6:
    ROS_INFO("You selected Option 6\n");
    break;

  case 7:
    ROS_INFO("You selected Option 7\n");
    break;

  case 8:
    ROS_INFO("You selected Option 8\n");

  case 9:
    ROS_INFO("You selected Option 9\n");
    break;

  case 10:
    ROS_INFO("You selected Option 10\n");
    break;

  case 11:
    ROS_INFO("You selected Option 11\n");
    break;

  case 12:
    ROS_INFO("You selected Option 12\n");
    break;

  case 13:
    ROS_INFO("Exiting...\n");
    ros::shutdown();
    break;

  default:
    ROS_WARN("Invalid choice. Please choose a valid option.");
    break;
  }
}