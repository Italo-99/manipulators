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

ManipulatorMenu::ManipulatorMenu(const bool tcp_pub):tcp_pub_(tcp_pub)
{
    // --------------------- PUBS & SUBS DELCARATIONS ---------------------

    jointStatePublisher_      = nh_.advertise<sensor_msgs::JointState>("/desired_joint_pose", 1);
    tcpPosePublisher_         = nh_.advertise<geometry_msgs::Pose>("/desired_tcp_pose", 1);
    tcpPoseIKPublisher_       = nh_.advertise<geometry_msgs::Pose>("/desired_tcpIK_pose", 1);
    collisionObjectPublisher_ = nh_.advertise<moveit_msgs::CollisionObject>("/add_collision_object", 1);
    jointStateSubscriber_     = nh_.subscribe("/joint_states", 1, &ManipulatorMenu::jointStateCallback, this);
    display_goal_pub_         = nh_.advertise<geometry_msgs::PoseStamped>("/display_robot_goal", 1, true);

    if (tcp_pub_) {eepose_sub_= nh_.subscribe("/display_eepose", 1, &ManipulatorMenu::eePoseCallback, this);}
    else          {eepose_pub_= nh_.advertise<geometry_msgs::PoseStamped>("/display_robot_goal", 1, true);}

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
    getEEpos_rpy();
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
  tcpPoseMsg.orientation = quaternion_from_euler(position[3],position[4],position[5]);

  tcpPosePublisher_.publish(tcpPoseMsg);

  // Display the goal on RViz
  geometry_msgs::PoseStamped robot_goal_msg;
  robot_goal_msg.header.frame_id = "base_link";
  robot_goal_msg.header.stamp = ros::Time::now();
  robot_goal_msg.pose = tcpPoseMsg,
  
  display_goal_pub_.publish(robot_goal_msg);
}

// Publish a Tcp goal by passing a vector (rotations must be expressed in deg)
void ManipulatorMenu::publishTcpIKGoal(const std::vector<double> position) 
{
  geometry_msgs::Pose tcpPoseMsg;

  tcpPoseMsg.position.x = position[0];
  tcpPoseMsg.position.y = position[1];
  tcpPoseMsg.position.z = position[2];

  // Conversion from euler rotation to pose quaternion
  tcpPoseMsg.orientation = quaternion_from_euler(position[3],position[4],position[5]);

  tcpPoseIKPublisher_.publish(tcpPoseMsg);

  // Display the goal on RViz
  geometry_msgs::PoseStamped robot_goal_msg;
  robot_goal_msg.header.frame_id = "base_link";
  robot_goal_msg.header.stamp = ros::Time::now();
  robot_goal_msg.pose = tcpPoseMsg,
  
  display_goal_pub_.publish(robot_goal_msg);
}

void ManipulatorMenu::oneJointMove(const int num, const double joint_rot)
{
  std::vector<double> joint_target = {0.,0.,0.,0.,0.,0.};
  for (unsigned int k = 0; k < 6; k++) 
  {
    joint_target[k] = current_joint_pose_.position[k];
  }
  joint_target[num] = joint_target[num] + joint_rot*M_PI/180;
  publishJointGoal(joint_target);
}

// Get current EE pose
geometry_msgs::PoseStamped ManipulatorMenu::getEEpose()
{
  if(tcp_pub_) {current_tcp_pose_ = getTf("base_link","tool0");}
  eepose_pub_.publish(current_tcp_pose_);
  return current_tcp_pose_;
}

// Get EE pos as vector with RPY euler angles
std::vector<double> ManipulatorMenu::getEEpos_rpy()
{
  // Read current EE pose by TF
  getEEpose();

  // Declaration of pose vector
  std::vector<double> tcp_pose_rpy = {0.,0.,0.,0.,0.,0.};

  // Fill the position
  tcp_pose_rpy[0] = current_tcp_pose_.pose.position.x;
  tcp_pose_rpy[1] = current_tcp_pose_.pose.position.y;
  tcp_pose_rpy[2] = current_tcp_pose_.pose.position.z;

  // Fill the rotation
  std::vector<double> tcp_rpy = euler_from_quaternion(current_tcp_pose_.pose.orientation);
  tcp_pose_rpy[3] = tcp_rpy[0];
  tcp_pose_rpy[4] = tcp_rpy[1];
  tcp_pose_rpy[5] = tcp_rpy[2];

  return tcp_rpy;
}

// Listen a TF between two given frames
geometry_msgs::PoseStamped ManipulatorMenu::getTf(const std::string& source_frame, const std::string& target_frame)
{
  // Create a TF2 buffer and listener
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // Wait for the transformation to be available
  try {tf_buffer.canTransform(source_frame, target_frame, ros::Time(0), ros::Duration(0.5));} 
  catch (tf2::TransformException& ex) {ROS_WARN("%s", ex.what());}

  // Get the transformation
  geometry_msgs::TransformStamped transformStamped;
  try                                 {transformStamped = tf_buffer.lookupTransform(source_frame, target_frame,ros::Time(0));}
  catch (tf2::TransformException &ex) {ROS_WARN("%s",ex.what()); ros::Duration(1.0).sleep();}

  // Convert the tf msg into a PoseStamped
  geometry_msgs::PoseStamped target_pose;
  target_pose.header.frame_id  = source_frame;
  target_pose.header.stamp     = ros::Time::now();
  target_pose.pose.position.x  = transformStamped.transform.translation.x;
  target_pose.pose.position.y  = transformStamped.transform.translation.y;
  target_pose.pose.position.z  = transformStamped.transform.translation.z;
  target_pose.pose.orientation = transformStamped.transform.rotation;

  return target_pose;
}

void ManipulatorMenu::move_along_x(const double x_step)
{
  std::vector<double> goal_pose = getEEpos_rpy();
  goal_pose[0] = goal_pose[0] + x_step;
  publishTcpIKGoal(goal_pose);
}

void ManipulatorMenu::move_along_y(const double y_step)
{
    std::vector<double> goal_pose = getEEpos_rpy();
    goal_pose[1] = goal_pose[1] + y_step;
    publishTcpIKGoal(goal_pose);
}

void ManipulatorMenu::move_along_z(const double z_step)
{
    std::vector<double> goal_pose = getEEpos_rpy();
    goal_pose[2] = goal_pose[2] + z_step;
    publishTcpIKGoal(goal_pose);
}

void ManipulatorMenu::make_tcp_rot(const std::vector<double> rot_vec)
{
  std::vector<double> goal_pose = getEEpos_rpy();
  goal_pose[3] = goal_pose[3] + rot_vec[0];
  goal_pose[4] = goal_pose[4] + rot_vec[1];
  goal_pose[5] = goal_pose[5] + rot_vec[2];
  publishTcpIKGoal(goal_pose);
}

void ManipulatorMenu::change_tcp_orient(const std::vector<double> rot_vec)
{
  std::vector<double> goal_pose = getEEpos_rpy();
  goal_pose[3] = rot_vec[0];
  goal_pose[4] = rot_vec[1];
  goal_pose[5] = rot_vec[2];
  publishTcpIKGoal(goal_pose);
}

void ManipulatorMenu::rotate_around_x(const double x_rot_step)
{
  std::vector<double> goal_pose = getEEpos_rpy();
  goal_pose[3] = goal_pose[3] + x_rot_step;
  publishTcpIKGoal(goal_pose);
}

void ManipulatorMenu::rotate_around_y(const double y_rot_step)
{
  std::vector<double> goal_pose = getEEpos_rpy();
  goal_pose[4] = goal_pose[4] + y_rot_step;
  publishTcpIKGoal(goal_pose);
}

void ManipulatorMenu::rotate_around_z(const double z_rot_step)
{
  std::vector<double> goal_pose = getEEpos_rpy();
  goal_pose[5] = goal_pose[5] + z_rot_step;
  publishTcpIKGoal(goal_pose);
}

// --------------------- PRIVATE FUNCTIONS ---------------------
  
// --------------------- PUBS HANDLER ---------------------
  
// --------------------- JOINT GOALS HANDLER ---------------------

void ManipulatorMenu::testJointGoal()
{
  // Declare the empty vector of joints goals
  std::vector<double> joints = {0.,0.,0.,0.,0.,0.};

  // Alternate a different joint goal when launching this function
  counterJg_ = !counterJg_;
  if (counterJg_) {joints = {0.0,-1.57,+1.57,0.0,+1.57,0.0};}
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

void ManipulatorMenu::oneJointMove_user()
{
  int num = 0;
  double joint_rot = 0.0;
  std::cout << "Enter the values of the joint to move in [0,5]: \n";
  std::cin >> num;
  std::cout << "Enter the rotation of the joint in deg: \n";
  std::cin >> joint_rot;
  oneJointMove(num,joint_rot);
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

void ManipulatorMenu::userTcpIKGoal()
{
  // Declare the empty vector of joints goals
  std::vector<double> position = {0.,0.,0.,0.,0.,0.};
  
  // Take user degree angle for each joint
  std::cout << "Enter the values of the tcp goal through InvKine, with rotation angles in degrees:";

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

  publishTcpIKGoal(position);
}

// --------------------- SUBS HANDLER ---------------------

void ManipulatorMenu::jointStateVisualizer() 
{
    for (unsigned int k = 0; k < 6; k++)
    {
      std::cout << "Joint 0: " << current_joint_pose_.position[0];
      std::cout << "Joint 1: " << current_joint_pose_.position[1];
      std::cout << "Joint 2: " << current_joint_pose_.position[2];
      std::cout << "Joint 3: " << current_joint_pose_.position[3];
      std::cout << "Joint 4: " << current_joint_pose_.position[4];
      std::cout << "Joint 5: " << current_joint_pose_.position[5];
    }
}

void ManipulatorMenu::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) 
{
  // Update joint current pose
  current_joint_pose_ = *msg;
}

void ManipulatorMenu::eePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // Update current ee pose
  current_tcp_pose_ = *msg;
}

// --------------------- COLLISION OBJECTS HANDLER ---------------------
// Create a collision object from a selected primitive
void ManipulatorMenu::addObj(const std::string&   name,
                             const int            obj_type, 
                             std::vector<double>  obj_dims, 
                             double               obj_pos[], 
                             double               rot_pos[])
{
  // Creation of the obj
  moveit_msgs::CollisionObject obj;

  obj.header.frame_id = "base_link";
  obj.id              = name;
  obj.primitives.resize(1);
  obj.primitives[0].type = obj_type;
  int size_obj_dims = obj_dims.size();
  obj.primitives[0].dimensions.resize(size_obj_dims);

  // Set primitive type
  switch(obj_type)
  {
    case 1:   // BOX: Rectangular shape setting
      if (size_obj_dims != 3)  {ROS_WARN_THROTTLE(3, "obj_dims array is not compatible with obj_type");}
      else                     {// Set the three dimensions of the parallelepiped
                                obj.primitives[0].dimensions[0] = obj_dims[0];
                                obj.primitives[0].dimensions[1] = obj_dims[1];
                                obj.primitives[0].dimensions[2] = obj_dims[2];}
      break;

    case 2:   // SPHERE
      if (size_obj_dims != 1)  {ROS_WARN_THROTTLE(3, "obj_dims array is not compatible with obj_type");}
      else                     {// Set the sphere radius
                                obj.primitives[0].dimensions[0] = obj_dims[0];}
      break;

    default:   // CYLINDER OR CONE
      if (size_obj_dims != 2)  {ROS_WARN_THROTTLE(3, "obj_dims array is not compatible with obj_type");}
      else                     {// Set height and radius of the cylinder/cone
                                obj.primitives[0].dimensions[0] = obj_dims[0];
                                obj.primitives[0].dimensions[1] = obj_dims[1];}
      break;
  }

  // Set static obj
  obj.operation = 0;

  // Set obj position
  obj.primitive_poses.resize(1);
  obj.primitive_poses[0].position.x = obj_pos[0];
  obj.primitive_poses[0].position.y = obj_pos[1];
  obj.primitive_poses[0].position.z = obj_pos[2];

  // Set obj orientation
  obj.primitive_poses[0].orientation.x = rot_pos[0];
  obj.primitive_poses[0].orientation.y = rot_pos[1];
  obj.primitive_poses[0].orientation.z = rot_pos[2];
  obj.primitive_poses[0].orientation.w = rot_pos[3];

  collisionObjectPublisher_.publish(obj);
}
// Function to add a collision object
void ManipulatorMenu::addCollObj()
{
  std::string name;
  int obj_type;
  std::vector<double>  obj_dims;
  double               obj_pos[] = {0.,0.,0.};
  double               rot_pos[] = {0.,0.,0.};
  std::cout << "Insert following infomation about the obj.";
  std::cout << "Name: "; std::cin >> name;
  std::cout << "Object type: 1 for BOX, 2 for SPHERE, 3 for CYLINDER, 4 for CONE."; std::cin >> obj_type;
  //If box chosen
  if (obj_type == 1)       {obj_dims = {0.,0.,0.,};
                            std::cout << "X dim: "; std::cin >> obj_dims[0];
                            std::cout << "Y dim: "; std::cin >> obj_dims[1];
                            std::cout << "Z dim: "; std::cin >> obj_dims[2];}
  // If sphere chosen
  else if (obj_type == 2)  {obj_dims = {0.};
                            std::cout << "X dim: "; std::cin >> obj_dims[0];}
  // Else
  else                     {obj_dims = {0.,0.};
                            std::cout << "X dim: "; std::cin >> obj_dims[0];
                            std::cout << "Y dim: "; std::cin >> obj_dims[1];}

  std::cout << "Insert position";
  std::cout << "X position: "; std::cin >> obj_pos[0];
  std::cout << "Y position: "; std::cin >> obj_pos[1];
  std::cout << "Z position: "; std::cin >> obj_pos[2];
  std::cout << "Insert orientation";
  std::cout << "RX rotation: "; std::cin >> obj_pos[3];
  std::cout << "RY rotation: "; std::cin >> obj_pos[4];
  std::cout << "RZ rotation: "; std::cin >> obj_pos[5];

  addObj(name,obj_type,obj_dims,obj_pos,rot_pos);
}

void ManipulatorMenu::publishCollisionObject() 
{
    moveit_msgs::CollisionObject collisionObjectMsg;
    // Fill collision object message
    collisionObjectPublisher_.publish(collisionObjectMsg);
}

// --------------------- QUATERNIONS HANDLER -------------------
// Conversion from degrees euler angles to quaternion
geometry_msgs::Quaternion ManipulatorMenu::quaternion_from_euler(double roll, double pitch, double yaw)
{
  // Declaration of empty quaternion
  geometry_msgs::Quaternion quaternion;

  // Conversion from euler rotation to pose quaternion
  tf2::Quaternion quat; quat.setRPY(roll*M_PI/180,pitch*M_PI/180,yaw*M_PI/180); quat.normalize();
  quaternion.x = quat.getX();
  quaternion.y = quat.getY();
  quaternion.z = quat.getZ();
  quaternion.w = quat.getW();

  return quaternion;
}

std::vector<double> ManipulatorMenu::euler_from_quaternion(const geometry_msgs::Quaternion quaternion)
{
  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(quaternion, tf_quaternion);

  // Get Euler angles
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

  // Store the angles in a vector
  std::vector<double> euler_angles = {roll,pitch,yaw};

  return euler_angles; 
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
  std::cout << "11.Rotate the TCP around x\n";
  std::cout << "12.Rotate the TCP around y\n";
  std::cout << "13.Rotate the TCP around z\n";
  std::cout << "14.Get a fixed TCP orientation\n";
  std::cout << "15.Add an object to the scene\n";
  std::cout << "16.Visualize joints state\n";
  std::cout << "17.Visualize current tcp pose\n";
  std::cout << "18.Shutdown the menu\n";
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
  double step;
  std::vector<double> rot;
  std::vector<double> ee_pos;
  geometry_msgs::PoseStamped ee_pose;
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
    userTcpIKGoal();
    break;

  case 6:
    ROS_INFO("You selected Option 6\n");
    oneJointMove_user();
    break;

  case 7:
    ROS_INFO("You selected Option 7\n");
    
    std::cout << "Insert how many metres you want to move along x";
    std::cin >> step;
    move_along_x(step);
    break;

  case 8:
    ROS_INFO("You selected Option 8\n");
    
    std::cout << "Insert how many metres you want to move along y";
    std::cin >> step;
    move_along_y(step);

  case 9:
    ROS_INFO("You selected Option 9\n");
    
    std::cout << "Insert how many metres you want to move along z";
    std::cin >> step;
    move_along_z(step);
    break;

  case 10:
    ROS_INFO("You selected Option 10\n");
    std::cout << "Insert the rotation around the axis you want to do.";
    
    rot = {0.,0.,0.}; 
    std::cout << " X rotation: "; std::cin >> rot[0];
    std::cout << " Y rotation: "; std::cin >> rot[1];
    std::cout << " Z rotation: "; std::cin >> rot[2];
    make_tcp_rot(rot);
    break;

  case 11:
    ROS_INFO("You selected Option 11\n");
    std::cout << "Insert the rotation around X axis you want to do.";
    double x_rot; 
    std::cout << " X rotation: "; std::cin >> x_rot;
    rotate_around_x(x_rot);
    break;

  case 12:
    ROS_INFO("You selected Option 12\n");
    std::cout << "Insert the rotation around Y axis you want to do.";
    double y_rot; 
    std::cout << " Y rotation: "; std::cin >> y_rot;
    rotate_around_y(y_rot);
    break;

  case 13:
    ROS_INFO("You selected Option 13\n");
    std::cout << "Insert the rotation around Z axis you want to do.";
    double z_rot; 
    std::cout << " Z rotation: "; std::cin >> z_rot;
    rotate_around_z(z_rot);
    break;

  case 14:
    ROS_INFO("You selected Option 14\n");
    std::cout << "Insert the FIXED orientation of the EE you want to have.";
    
    rot = {0.,0.,0.}; 
    std::cout << " X rotation: "; std::cin >> rot[0];
    std::cout << " Y rotation: "; std::cin >> rot[1];
    std::cout << " Z rotation: "; std::cin >> rot[2];
    change_tcp_orient(rot);
    break;

  case 15:
    ROS_INFO("You selected Option 15\n");
    addCollObj();
    break;

  case 16:
    ROS_INFO("You selected Option 14\n");
    for (unsigned int k = 0; k < 6; k++) 
    {
      std::cout << "Joint " << k << " : " << current_joint_pose_.position[k];
    }    
    break;

  case 17:
    ROS_INFO("You selected Option 15\n");
    ee_pose = getEEpose();
    ee_pos = getEEpos_rpy();
    std::cout << " EE - X position: " << ee_pos[0];
    std::cout << " EE - Y position: " << ee_pos[1];
    std::cout << " EE - Z position: " << ee_pos[2];
    std::cout << " EE - X rotation: " << ee_pos[3];
    std::cout << " EE - Y rotation: " << ee_pos[4];
    std::cout << " EE - Z rotation: " << ee_pos[5];
    break;

  case 18:
    ROS_INFO("Exiting...\n");
    ros::shutdown();
    break;

  default:
    ROS_WARN("Invalid choice. Please choose a valid option.");
    break;
  }
}