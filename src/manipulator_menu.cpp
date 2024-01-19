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
    display_goal_pub_         = nh_.advertise<geometry_msgs::PoseStamped>("/display_robot_goal", 1, true);

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

// Publish a joint goal by passing a vector
void ManipulatorMenu::publishJointGoal(const std::vector<double> joints) 
{
  // Fill the joint msg
  sensor_msgs::JointState jointStateMsg;
  jointStateMsg.header.stamp = ros::Time::now();
  for (unsigned int k = 0; k < joints.size(); k++) {jointStateMsg.position.push_back(joints[k]); }

  // Publish the JointState message
  jointStatePublisher_.publish(jointStateMsg);
}

// Publish a Tcp goal by passing a vector (rotations must be expressed in deg)
void ManipulatorMenu::publishTcpGoal(const std::vector<double> position) 
{
  geometry_msgs::Pose tcpPoseMsg;

  tcpPoseMsg.position.x = position[0];
  tcpPoseMsg.position.y = position[1];
  tcpPoseMsg.position.z = position[2];

  // Conversion from euler rotation to pose quaternion
  tf2::Quaternion quat; quat.setRPY(position[3]*M_PI/180,position[4]*M_PI/180,position[5]*M_PI/180); quat.normalize();
  tcpPoseMsg.orientation.x = quat.getX();
  tcpPoseMsg.orientation.y = quat.getY();
  tcpPoseMsg.orientation.z = quat.getZ();
  tcpPoseMsg.orientation.w = quat.getW();

  tcpPosePublisher_.publish(tcpPoseMsg);

  // Display the goal on RViz
  geometry_msgs::PoseStamped robot_goal_msg;
  robot_goal_msg.header.frame_id = "base_link";
  robot_goal_msg.header.stamp = ros::Time::now();
  robot_goal_msg.pose = tcpPoseMsg,
  
  display_goal_pub_.publish(robot_goal_msg);
}

// --------------------- PRIVATE FUNCTIONS ---------------------
  
// --------------------- PUBS HANDLER ---------------------
  
// --------------------- JOINT GOALS HANDLER ---------------------

void ManipulatorMenu::testJointGoal()
{
  // Declare the empty vector of joints goals
  std::vector<double> joints = {0.,0.,0.,0.,0.,0.};

  // Alternate a different joint goal when launching this function
  counterCg_ = !counterCg_;
  if (counterCg_) {joints = {0.0,-1.57,+1.57,0.0,+1.57,0.0};}
  else            {joints = {0.0,-1.57,+1.57,0.0,-1.57,0.0};}   
  
  publishJointGoal(joints);
}

void ManipulatorMenu::userJointGoal()
{
  // Declare the empty vector of joints goals
  std::vector<double> joints = {0.,0.,0.,0.,0.,0.};
  
  // Take user degree angle for each joint
  std::cout << "Enter the values of the joint goal in degrees: \n";

  for (unsigned int k = 0; k < 6; k++)
  {
    std::cout << "Joint " << k+1 << " : ";
    std::cin >> joints[k];
    joints[k] = joints[k]/180.00*M_PI;
  }

  publishJointGoal(joints);
}

// --------------------- TCP GOALS HANDLER ---------------------

void ManipulatorMenu::testTcpGoal()
{
  // Declare the empty vector of joints goals
  std::vector<double> position = {0.,0.,0.,0.,0.,0.};

  // Alternate a different joint goal when launching this function
  counterCg_ = !counterCg_;
  if (counterCg_) {position = {0.60,0.20,0.35,0.0,0.0,90.0};}
  else            {position = {0.60,-0.20,0.35,0.0,0.0,90.0};}   
  
  publishTcpGoal(position);
}

void ManipulatorMenu::userTcpGoal()
{
  // Declare the empty vector of joints goals
  std::vector<double> position = {0.,0.,0.,0.,0.,0.};
  
  // Take user degree angle for each joint
  std::cout << "Enter the values of the tcp goal, with rotation angles in degrees:";

  // X position input
  std::cout << "X position:  ";
  std::cin >> position[0];
  // Y position input
  std::cout << "Y position:  ";
  std::cin >> position[1];
  // Z position input
  std::cout << "Z position:  ";
  std::cin >> position[2];

  // Deg RPY angles input
  std::cout << "Rx: ";
  std::cin >> position[3];
  std::cout << "Ry: ";
  std::cin >> position[4];
  std::cout << "Rz: ";
  std::cin >> position[5];

  publishTcpGoal(position);
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

void ManipulatorMenu::publishCollisionObject() 
{
    moveit_msgs::CollisionObject collisionObjectMsg;
    // Fill collision object message
    collisionObjectPublisher_.publish(collisionObjectMsg);
}


// --------------------- MENU HANDLER ---------------------

void ManipulatorMenu::printMenu()
{
  std::cout << "\n======= Manipulator Menu =======\n";
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
    testJointGoal();
    break;

  case 2:
    ROS_INFO("You selected Option 2");
    testTcpGoal();
    break;

  case 3:
    ROS_INFO("You selected Option 3");
    userJointGoal();
    break;

  case 4:
    ROS_INFO("You selected Option 4");
    userTcpGoal();
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