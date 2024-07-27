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

// IMPORT LIBRARIES
#include "manipulators/ManipulatorMenu.h"

// --------------------- PUBLIC CONSTRUCTOR ---------------------

ManipulatorMenu::ManipulatorMenu(std::string gripper_joint_name,
                                 std::string last_robot_link)
{
  // Gripper attribute setting
  gripper_joint_name_ = gripper_joint_name;
  last_robot_link_    = last_robot_link;

  // Listen static tf from tool0 and tcp_gripper
  geometry_msgs::PoseStamped ee_offset_pose = getTf(last_robot_link_,"tcp_gripper");
  if      (last_robot_link_ == "tool0") 
  {
    ee_offset_ = ee_offset_pose.pose.position.z;
  }
  else if (last_robot_link_ == "link_6") 
  {
    ee_offset_ = ee_offset_pose.pose.position.x;
    ROS_INFO("ProjectRED robot-gripper setup");
  }
  else
  {
    ee_offset_ = ee_offset_pose.pose.position.z;
  }  

  // --------------------- PUBS & SUBS DELCARATIONS ---------------------

  jointGoalPublisher_       = nh_.advertise<sensor_msgs::JointState>("/desired_joint_pose", 1);
  tcpPosePublisher_         = nh_.advertise<geometry_msgs::Pose>("/desired_tcp_pose", 1);
  tcpPoseIKPublisher_       = nh_.advertise<geometry_msgs::Pose>("/desired_tcpIK_pose", 1);
  tcpPoseIK_noplannerPub_   = nh_.advertise<geometry_msgs::Pose>("/noplan_tcpIK_pose", 1);
  carthesianMovePublisher_  = nh_.advertise<geometry_msgs::PoseArray>("/desired_cartesian_move", 1, true);
  display_goal_pub_         = nh_.advertise<geometry_msgs::PoseStamped>("/display_robot_goal", 1, true);
  eepose_pub_               = nh_.advertise<geometry_msgs::PoseStamped>("/display_ee_pose", 1, true);
  collisionObjectPublisher_ = nh_.advertise<moveit_msgs::CollisionObject>("/add_collision_object", 1);
  moveGripperPublisher_     = nh_.advertise<std_msgs::Float64>(gripper_joint_name_+"/gripper_control", 1);
  jointStateSubscriber_     = nh_.subscribe("/joint_states", 1, &ManipulatorMenu::jointStateCallback, this);
  joy_sub_                  = nh_.subscribe("/joy", 1, &ManipulatorMenu::joyCallback, this);

  // --------------------- Global class variables init ---------------------

  joy_step_   = 0.01;       // Vel speed from joy
  counterJg_ = false;       // choice of test joint goal
  counterCg_ = false;       // choice of test tcp3D goal

  // --------------------- CoppeliaSim client init ---------------------
  client_ = nh_.serviceClient<manipulators::CoppeliaMenu>("coppelia_menu");

  // --------------------- Gripper client init ---------------------
  gripper_client_ = nh_.serviceClient<std_srvs::SetBool>(gripper_joint_name_+"/move_gripper");
  grab_client_    = nh_.serviceClient<std_srvs::SetBool>(gripper_joint_name_+"/grabbing_gripper");

  // Real gripper client init
  real_gripper_client_ = nh_.serviceClient<gripper::RobotiQGripperControl>("/ur_rtde/robotiq_gripper/command");
}

// --------------------- PUBLIC FUNCTIONS ---------------------

// --------------------- ROS HANDLER ---------------------

// Asynchronous spinner for ROS routines without user menu
void ManipulatorMenu::spinner()
{
  ros::spinOnce();  // The loop rate is set in the child class node
}

// Asynchronous spinner for ROS routines with user menu
void ManipulatorMenu::spinnerMenu()
{
  // Initialize user choice variable
  int userChoice = 0;

  while (ros::ok())
  {
    spinner();                      // ROS Once spinner
    getEEpos_rpy();                 // Update current robot pose
    printMenu();                    // Print choice menu
    userChoice = getUserChoice();   // Get user choice from the terminal
    processChoice(userChoice);      // Execute the command
    ros::Duration(1.0).sleep();     // Wait 1s until next command
  }

  ROS_INFO("Closing the menu!\n");
  ros::shutdown();
}

// --------------------- COPPELIASIM HANDLER ---------------------

// Open Coppelia simulation
void ManipulatorMenu::startCoppeliaSim()
{
  coppelia_srv_.request.command = 0;
  wait_for_response();
}

// Close Coppelia simulation
void ManipulatorMenu::stopCoppeliaSim()
{
  coppelia_srv_.request.command = 1;
  wait_for_response();
}

// Save Coppelia scene
void ManipulatorMenu::saveCoppeliaScene()
{
  coppelia_srv_.request.command = 2;
  wait_for_response();
}

// Change Coppelia cable pose
void ManipulatorMenu::changeCoppeliaCablePose()
{
  coppelia_srv_.request.command = 3;
  wait_for_response();
}

// --------------------- MOVEMENTS HANDLER ---------------------

// Publish a joint goal by passing a vector of joints in deg
sensor_msgs::JointState ManipulatorMenu::publishJointGoal(const std::vector<double> joints) 
{
  // Fill the joint msg with degToRad conversion
  sensor_msgs::JointState jointStateMsg;
  jointStateMsg.header.stamp = ros::Time::now();
  for (unsigned int k = 0; k < joints.size(); k++)
  {
    jointStateMsg.position.push_back(joints[k]*M_PI/180);
  }
  
  return publishJointGoal(jointStateMsg);
}

// Publish a joint goal by passing a JointState msg
sensor_msgs::JointState ManipulatorMenu::publishJointGoal(const sensor_msgs::JointState jointStateMsg)
{
  // Publish the JointState message
  jointGoalPublisher_.publish(jointStateMsg);
  return jointStateMsg;
}

// Publish a Tcp goal by passing a vector (rotations must be expressed in deg)
geometry_msgs::Pose ManipulatorMenu::publishTcpGoal(const std::vector<double> position) 
{
  geometry_msgs::Pose tcpPoseMsg;

  tcpPoseMsg.position.x = position[0];
  tcpPoseMsg.position.y = position[1];
  tcpPoseMsg.position.z = position[2];

  // Conversion from euler rotation to pose quaternion
  tcpPoseMsg.orientation = quaternion_from_euler(position[3],position[4],position[5]);

  return publishTcpGoal(tcpPoseMsg);
}

// Publish a Tcp goal by passing a geometry_msgs::Pose
geometry_msgs::Pose ManipulatorMenu::publishTcpGoal(const geometry_msgs::Pose tcpPoseMsg)
{
  tcpPosePublisher_.publish(tcpPoseMsg);

  // Display the goal on RViz
  geometry_msgs::PoseStamped robot_goal_msg;
  robot_goal_msg.header.frame_id = "base_link";
  robot_goal_msg.header.stamp = ros::Time::now();
  robot_goal_msg.pose = tcpPoseMsg,
  
  display_goal_pub_.publish(robot_goal_msg);

  return tcpPoseMsg;
}

// Publish a TcpIK goal by passing a vector (rotations must be expressed in deg)
geometry_msgs::Pose ManipulatorMenu::publishTcpIKGoal(const std::vector<double> position) 
{
  // TCP pose
  geometry_msgs::Pose tcpPoseMsg;
  tcpPoseMsg.position.x = position[0];
  tcpPoseMsg.position.y = position[1];
  tcpPoseMsg.position.z = position[2];
  tcpPoseMsg.orientation = quaternion_from_euler(position[3],position[4],position[5]);

  return publishTcpIKGoal(tcpPoseMsg);
}

// Publish a TcpIK goal by passing a geometry_msgs::Pose
geometry_msgs::Pose ManipulatorMenu::publishTcpIKGoal(const geometry_msgs::Pose tcpPoseMsg) 
{
  // The pose to pass as goal to invKine planner must be referred to tool0
  geometry_msgs::Pose tool0_PoseMsg;
  tool0_PoseMsg.orientation = tcpPoseMsg.orientation;
  Eigen::Quaterniond  q(tcpPoseMsg.orientation.w,
                        tcpPoseMsg.orientation.x,
                        tcpPoseMsg.orientation.y,
                        tcpPoseMsg.orientation.z);
  Eigen::Affine3d transform = Eigen::Translation3d(
                              tcpPoseMsg.position.x,
                              tcpPoseMsg.position.y,
                              tcpPoseMsg.position.z)*q;
  Eigen::Vector3d vec_offset(0, 0, -ee_offset_);
  // Correction if link_6 has x offset, while standard tool0 frame has z offset
  if (last_robot_link_ == "link_6")
  {
    vec_offset[0] = -ee_offset_;
    vec_offset[1] = 0.0;
    vec_offset[2] = 0.0;
  }
  Eigen::Vector3d tool0_pos = transform * vec_offset;

  tool0_PoseMsg.position.x = tool0_pos.x();
  tool0_PoseMsg.position.y = tool0_pos.y();
  tool0_PoseMsg.position.z = tool0_pos.z();

  // Plan trajectory through inverse kinematics
  tcpPoseIKPublisher_.publish(tool0_PoseMsg);

  // Display the goal on RViz
  geometry_msgs::PoseStamped robot_goal_msg;
  robot_goal_msg.header.frame_id = "base_link";
  robot_goal_msg.header.stamp = ros::Time::now();
  robot_goal_msg.pose = tcpPoseMsg;
  
  display_goal_pub_.publish(robot_goal_msg);

  return tcpPoseMsg;
}

// Publish a TcpIK goal, with fake moveit controller, by passing a vector (rotations must be expressed in deg)
geometry_msgs::Pose ManipulatorMenu::publishTcpIK_noplanner_Goal(const std::vector<double> position) 
{
  // TCP pose
  geometry_msgs::Pose tcpPoseMsg;
  tcpPoseMsg.position.x = position[0];
  tcpPoseMsg.position.y = position[1];
  tcpPoseMsg.position.z = position[2];
  tcpPoseMsg.orientation = quaternion_from_euler(position[3],position[4],position[5]);

  return publishTcpIK_noplanner_Goal(tcpPoseMsg);
}

// Publish a TcpIK goal, with fake moveit controller, by passing a geometry_msgs::Pose
geometry_msgs::Pose ManipulatorMenu::publishTcpIK_noplanner_Goal(const geometry_msgs::Pose tcpPoseMsg) 
{
  // The pose to pass as goal to invKine planner must be referred to tool0
  geometry_msgs::Pose tool0_PoseMsg;
  tool0_PoseMsg.orientation = tcpPoseMsg.orientation;
  Eigen::Quaterniond  q(tcpPoseMsg.orientation.w,
                        tcpPoseMsg.orientation.x,
                        tcpPoseMsg.orientation.y,
                        tcpPoseMsg.orientation.z);
  Eigen::Affine3d transform = Eigen::Translation3d(
                              tcpPoseMsg.position.x,
                              tcpPoseMsg.position.y,
                              tcpPoseMsg.position.z)*q;
  Eigen::Vector3d vec_offset(0, 0, -ee_offset_);
  // Correction if link_6 has x offset, while standard tool0 frame has z offset
  if (last_robot_link_ == "link_6")
  {
    vec_offset[0] = -ee_offset_;
    vec_offset[1] = 0.0;
    vec_offset[2] = 0.0;
  }
  Eigen::Vector3d tool0_pos = transform * vec_offset;

  tool0_PoseMsg.position.x = tool0_pos.x();
  tool0_PoseMsg.position.y = tool0_pos.y();
  tool0_PoseMsg.position.z = tool0_pos.z();

  // Plan trajectory through inverse kinematics
  tcpPoseIK_noplannerPub_.publish(tool0_PoseMsg);

  // Display the goal on RViz
  geometry_msgs::PoseStamped robot_goal_msg;
  robot_goal_msg.header.frame_id = "base_link";
  robot_goal_msg.header.stamp = ros::Time::now();
  robot_goal_msg.pose = tcpPoseMsg;
  
  display_goal_pub_.publish(robot_goal_msg);

  return tcpPoseMsg;
}

// Publish a cartesian goal of poses sequence along the same line
// Specify axis1 and axis2 as 0 for x, 1 for y and 2 for z
// Define pos1 and pos2 the final poses along those axis (the 3rd won't change)
// steps value will define how many waypoints to put in 
geometry_msgs::Pose ManipulatorMenu::publishCartesianMove(const uint   axis1,
                                                          const uint   axis2,
                                                          const double pos1,
                                                          const double pos2,
                                                          const uint   steps) 
{
  // Initialize starting and waypoints variables
  geometry_msgs::PoseStamped current_pose = getTf("base_link",last_robot_link_);
  geometry_msgs::PoseArray   waypoints;
  geometry_msgs::Pose        final_pose;
  waypoints.header.frame_id = "base_link"; 
  double step_axisX = 0.;
  double step_axisY = 0.;
  double step_axisZ = 0.;
  // Compute axis step
  if  (axis1 == axis2) {ROS_WARN("Error in axis input!"); return final_pose;}
  else 
  {
    if      (axis1 == 0) {step_axisX = pos1 - current_pose.pose.position.x;}
    else if (axis1 == 1) {step_axisY = pos1 - current_pose.pose.position.y;}
    else if (axis1 == 2) {step_axisZ = pos1 - current_pose.pose.position.z;}
    if      (axis2 == 0) {step_axisX = pos2 - current_pose.pose.position.x;}
    else if (axis2 == 1) {step_axisY = pos2 - current_pose.pose.position.y;}
    else if (axis2 == 2) {step_axisZ = pos2 - current_pose.pose.position.z;}
  }
  // Compute common step
  step_axisX = step_axisX/static_cast<double>(steps);
  step_axisY = step_axisY/static_cast<double>(steps);
  step_axisZ = step_axisZ/static_cast<double>(steps);
  // Fill waypoints msgs
  for(unsigned int k = 0; k < steps; k++)
  {
    geometry_msgs::Pose wp;
    // Same orientation for every waypoint
    wp.orientation  = current_pose.pose.orientation;
    // Fill the position
    wp.position.x   = current_pose.pose.position.x + (k+1)*step_axisX; 
    wp.position.y   = current_pose.pose.position.y + (k+1)*step_axisY; 
    wp.position.z   = current_pose.pose.position.z + (k+1)*step_axisZ; 
    // Last position should be accurate
    if (k == steps-1)
    {
      if      (axis1 == 0) {wp.position.x = pos1;}
      else if (axis2 == 0) {wp.position.x = pos2;}
      if      (axis1 == 1) {wp.position.y = pos1;}
      else if (axis2 == 1) {wp.position.y = pos2;}
      if      (axis1 == 2) {wp.position.z = pos1;}
      else if (axis2 == 2) {wp.position.z = pos2;}
      final_pose = wp;
    }
    // Add current computed waypoints to the vector
    waypoints.poses.push_back(wp);
  }
  // Publish the msg
  carthesianMovePublisher_.publish(waypoints);

  return final_pose;
}

// Move a single joint, joint rotation must be in deg
sensor_msgs::JointState ManipulatorMenu::oneJointMove(const int num, const double joint_rot)
{
  // Read from subscribers the current joints state
  spinner();
  // Fill current joints pose as target
  std::vector<double> joint_target = {0.,0.,0.,0.,0.,0.};
  for (unsigned int k = 0; k < 6; k++) 
  {
    joint_target[k] = current_joint_pose_.position[k]*180/M_PI;
  }
  // Change the joint target position
  joint_target[num] = joint_target[num] + joint_rot;
  return publishJointGoal(joint_target);
}

// Go to pre configured home position
sensor_msgs::JointState ManipulatorMenu::goHome(const bool ee_orient)
{
  std::vector<double> start_joint_pose = {0.,0.,0.,0.,0.,0};
  if (!ee_orient) // gripper down
  {start_joint_pose = {0.,-90.,+90.,-90.,-90.,180};}
  else // gripper at the front
  {start_joint_pose = {0.,-90.,+90.,  0.,+90.,0};}
  // Publishe home joint goal 
  return publishJointGoal(start_joint_pose);
}

// -------------------- TF END EFFECTOR LISTENER -----------------------//

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

// Get current EE pose
geometry_msgs::PoseStamped ManipulatorMenu::getEEpose()
{
  // Compute the tf between base_link and end-effector
  current_tcp_pose_ = getTf("base_link","tcp_gripper");
  eepose_pub_.publish(current_tcp_pose_);
  return current_tcp_pose_;
}

// Get EE pose as vector with RPY euler angles
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

  return tcp_pose_rpy;
}

// -------------------- SIMPLE MOVES ALONG CARTHESIAN AXES -----------------------//

// Set a carthesian move along x axis in metres
geometry_msgs::Pose ManipulatorMenu::move_along_x(const double x_step)
{
  // Get current EE pose
  std::vector<double> goal_pose = getEEpos_rpy();
  // Update position along X
  goal_pose[0] = goal_pose[0] + x_step;
  return publishTcpIKGoal(goal_pose);
}

// Set a carthesian move along x axis in metres
geometry_msgs::Pose ManipulatorMenu::move_along_y(const double y_step)
{
  // Get current EE pose
  std::vector<double> goal_pose = getEEpos_rpy();
  // Update position along Y
  goal_pose[1] = goal_pose[1] + y_step;
  return publishTcpIKGoal(goal_pose);
}

// Set a carthesian move along x axis in metres
geometry_msgs::Pose ManipulatorMenu::move_along_z(const double z_step)
{
  // Get current EE pose
  std::vector<double> goal_pose = getEEpos_rpy();
  goal_pose[2] = goal_pose[2] + z_step;
  // Update position along Z
  return publishTcpIKGoal(goal_pose);
}

// -------------------- SIMPLE ROTATIONS AROUND CARTHESIAN AXES -----------------------//

// Set a RELATIVE ee rotation around the 3 carthesian axis (in degrees)
geometry_msgs::Pose ManipulatorMenu::make_tcp_rot(const std::vector<double> rot_vec)
{
  // Get current EE pose
  std::vector<double> goal_pose = getEEpos_rpy();
  // Update tcp orient goal
  goal_pose[3] = goal_pose[3] + rot_vec[0];
  goal_pose[4] = goal_pose[4] + rot_vec[1];
  goal_pose[5] = goal_pose[5] + rot_vec[2];
  return publishTcpIKGoal(goal_pose);
}

// Set an ABSOLUTE orientation ee position around the 3 carthesian axis (in degrees)
geometry_msgs::Pose ManipulatorMenu::change_tcp_orient(const std::vector<double> rot_vec)
{
  // Get current EE pose
  std::vector<double> goal_pose = getEEpos_rpy();
  // Update tcp orient goal
  goal_pose[3] = rot_vec[0];
  goal_pose[4] = rot_vec[1];
  goal_pose[5] = rot_vec[2];
  return publishTcpIKGoal(goal_pose);
}

// Set a relative rotation around x axis (in degrees)
geometry_msgs::Pose ManipulatorMenu::rotate_around_x(const double x_rot_step)
{
  // Get current EE pose
  std::vector<double> goal_pose = getEEpos_rpy();
  // Update tcp orient goal
  goal_pose[3] = goal_pose[3] + x_rot_step;
  return publishTcpIKGoal(goal_pose);
}

// Set a relative rotation around y axis (in degrees)
geometry_msgs::Pose ManipulatorMenu::rotate_around_y(const double y_rot_step)
{
  // Get current EE pose
  std::vector<double> goal_pose = getEEpos_rpy();
  // Update tcp orient goal
  goal_pose[4] = goal_pose[4] + y_rot_step;
  return publishTcpIKGoal(goal_pose);
}

// Set a relative rotation around z axis (in degrees)
geometry_msgs::Pose ManipulatorMenu::rotate_around_z(const double z_rot_step)
{
  // Get current EE pose
  std::vector<double> goal_pose = getEEpos_rpy();
  // Update tcp orient goal
  goal_pose[5] = goal_pose[5] + z_rot_step;
  return publishTcpIKGoal(goal_pose);
}

// --------------------- COLLISION OBJECTS HANDLER ---------------------

// Create a collision object from a selected primitive
void ManipulatorMenu::addObj(const std::string&   name,
                             const int            obj_type, 
                             std::vector<double>  obj_dims, 
                             double               obj_pos[], 
                             double               rot_pos[],
                             uint                 operation)
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

  // Set obj operation: ADD=0, REMOVE=1, APPEND=2, MOVE=3
  obj.operation = operation;

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

  publishCollisionObject(obj);
}

// Collision object publisher
void ManipulatorMenu::publishCollisionObject(const moveit_msgs::CollisionObject collisionObjectMsg) 
{
  collisionObjectPublisher_.publish(collisionObjectMsg);
}

// --------------------- GRIPPER CONTROL ---------------------

// Open the gripper
void ManipulatorMenu::openGripper()
{
  callGripperSrv(false);
}

// Close the gripper
void ManipulatorMenu::closeGripper()
{
  callGripperSrv(true);
}

// Move the gripper
void ManipulatorMenu::moveGripper(const double gripper_position)
{
  std_msgs::Float64 gripper_pos_msg;
  gripper_pos_msg.data = gripper_position;
  moveGripperPublisher_.publish(gripper_pos_msg);
}

// Grab an object at the gripper
void ManipulatorMenu::grabObjGripper()
{
  callGrabbingSrv(true);
}
// Detach an object from the gripper
void ManipulatorMenu::detachObjGripper()
{
  callGrabbingSrv(false); 
}

// Open real gripper
void ManipulatorMenu::openRealGripper()
{
  callRealGripperSrv(100.);
}

// Close real gripper
void ManipulatorMenu::closeRealGripper()
{
  callRealGripperSrv(0.);
}

// Move real gripper (input is in range [0,100])
void ManipulatorMenu::moveRealGripper(const float command)
{
  callRealGripperSrv(command);
}

// --------------------- PRIVATE FUNCTIONS ---------------------

// --------------------- COPPELIA HANDLER ---------------------

// Send the request and show the response
void ManipulatorMenu::wait_for_response()
{
  if (client_.call(coppelia_srv_))
  {
    ROS_INFO("Simulation status: %d", coppelia_srv_.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service coppelia_menu");
  }
}
  
// --------------------- JOINT GOALS HANDLER ---------------------

void ManipulatorMenu::testJointGoal()
{
  // Declare the empty vector of joints goals
  std::vector<double> joints = {0.,0.,0.,0.,0.,0.};

  // Alternate a different joint goal when launching this function
  counterJg_ = !counterJg_;
  if (counterJg_) {joints = {0.0,-90,+90,0.0,+90,0.0};}
  else            {joints = {0.0,-90,+90,0.0,-90,0.0};}   
  
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
  }

  publishJointGoal(joints);
}

void ManipulatorMenu::oneJointMove_user()
{
  int num = 0;
  double joint_rot = 0.0;
  std::cout << "Enter the joint to move in [0,5]: \n";
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
  std::cout << "Enter the values of the tcp goal, with rotation angles in degrees:\n";

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
} 

void ManipulatorMenu::userTcpIKGoal()
{
  // Declare the empty vector of joints goals
  std::vector<double> position = {0.,0.,0.,0.,0.,0.};
  
  // Take user degree angle for each joint
  std::cout << "Enter the values of the tcp goal through InvKine, with rotation angles in degrees:\n";

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

void ManipulatorMenu::userTcpIK_no_planner_Goal()
{
  // Declare the empty vector of joints goals
  std::vector<double> position = {0.,0.,0.,0.,0.,0.};
  
  // Take user degree angle for each joint
  std::cout << "Enter the values of the tcp goal through InvKine, with fake moveit controller, with rotation angles in degrees:\n";

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

  publishTcpIK_noplanner_Goal(position);
}

// --------------------- USER CARTESIAN MOVES HANDLER ---------------------
void ManipulatorMenu::userCartesianMove()
{
  ROS_INFO("Setup your cartesian move:");
  uint   axis1;
  uint   axis2;
  double pos1;
  double pos2;
  uint   steps;
  std::cout << "Insert the first axis  (0:x, 1:y, 2:z): "; std::cin >> axis1; 
  std::cout << "Insert the second axis (0:x, 1:y, 2:z): "; std::cin >> axis2;
  std::cout << "Insert the final position on axis1    : "; std::cin >> pos1; 
  std::cout << "Insert the final position on axis2    : "; std::cin >> pos2; 
  std::cout << "Set the number of waypoints passed    : "; std::cin >> steps;
  publishCartesianMove(axis1,axis2,pos1,pos2,steps);
}

// --------------------- SUBS HANDLER ---------------------

void ManipulatorMenu::jointStateVisualizer() 
{
  spinner();
  for (unsigned int k = 0; k < 6; k++)
  {
    std::cout << "Joint " << k << " : " << current_joint_pose_.position[k]*180/M_PI << std::endl;
  }
}

void ManipulatorMenu::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) 
{
  // Update joint current pose
  current_joint_pose_ = *msg;
}

// --------------------- COLLISION OBJECTS PRIVATE MENU HANDLER ---------------------

// Function to add a collision object from the user menu
void ManipulatorMenu::addCollObj()
{
  std::string name;
  int obj_type;
  std::vector<double>  obj_dims;
  double               obj_pos[] = {0.,0.,0.};
  double               rot_pos[] = {0.,0.,0.};
  std::cout << "Insert following infomation about the obj.\n";
  std::cout << "Name: "; std::cin >> name;
  std::cout << "Object type: 1 for BOX, 2 for SPHERE, 3 for CYLINDER, 4 for CONE.\n"; std::cin >> obj_type;
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

  std::cout << "Insert position\n";
  std::cout << "X position: "; std::cin >> obj_pos[0];
  std::cout << "Y position: "; std::cin >> obj_pos[1];
  std::cout << "Z position: "; std::cin >> obj_pos[2];
  std::cout << "Insert orientation\n";
  std::cout << "RX rotation: "; std::cin >> rot_pos[0];
  std::cout << "RY rotation: "; std::cin >> rot_pos[1];
  std::cout << "RZ rotation: "; std::cin >> rot_pos[2];

  geometry_msgs::Quaternion rot_quat = quaternion_from_euler(rot_pos[0],rot_pos[1],rot_pos[2]);
  double rot_pos_quat[4] = {rot_quat.x,rot_quat.y,rot_quat.z,rot_quat.w};

  addObj(name,obj_type,obj_dims,obj_pos,rot_pos_quat,0);
}

// Function to delete a given collision object from the user menu
void ManipulatorMenu::deleteCollObj()
{
    std::string obj_name_loc;
    std::cout << "Insert the name of the object you want to delete:" << std::endl;
    std::cin >> obj_name_loc;
    std::vector<double>   obj_dim_loc        = {0.,0.,0.};
    double                obj_pos_loc[]      = {0.,0.,0.};
    double                rot_pos_quat_loc[] = {0.,0.,0.,1.};
    addObj(obj_name_loc,1,obj_dim_loc,obj_pos_loc,rot_pos_quat_loc,1);
}

// --------------------- COLLISION OBJECTS PRIVATE MENU HANDLER ---------------------
// Gripper Moving command from the user
void ManipulatorMenu::userGripperMove()
{
  double gripper_position = 0.;

  // Take user gripper position
  std::cout << "Enter the value (in %) of gripper move :\n";

  // Gripper position input
  std::cout << "Gripper opening position: ";
  std::cin >> gripper_position;

  moveGripper(gripper_position);
}

// --------------------- GRIPPER SERVICES ---------------------
// Call the service for open/close gripper
void ManipulatorMenu::callGripperSrv(const bool command)
{
    // Create a request
    std_srvs::SetBool srv;
    srv.request.data = command;

    // Call the service
    if (gripper_client_.call(srv)) 
    {
        if (srv.response.success) 
        {
            ROS_INFO("Gripper move request succeeded");
        }
        else
        {
            ROS_ERROR("Gripper move request failed");
        }
    }
    else
    {
        ROS_ERROR("Failed to call service to move the gripper");
    }
}

// Call the service for grab/detach an object at the gripper
void ManipulatorMenu::callGrabbingSrv(const bool command)
{
    // Create a request
    std_srvs::SetBool srv;
    srv.request.data = command;

    // Call the service
    if (grab_client_.call(srv)) 
    {
        if (srv.response.success) 
        {
            ROS_INFO("Gripper grabbing request succeeded");
        }
        else
        {
            ROS_ERROR("Gripper grabbing request failed");
        }
    }
    else
    {
        ROS_ERROR("Failed to call service for gripper grabbing");
    }
}

// Call the service to open/close the real gripper
void ManipulatorMenu::callRealGripperSrv(const float command)
{
    // Create a request
    gripper::RobotiQGripperControl srv;
    srv.request.position = command;
    srv.request.speed    = 50;
    srv.request.force    = 50;

    // Call the service
    if (real_gripper_client_.call(srv)) 
    {
        if (srv.response.success) 
        {
            ROS_INFO("Gripper move request succeeded");
        }
        else
        {
            ROS_ERROR("Gripper move request failed");
        }
    }
    else
    {
        ROS_ERROR("Failed to call service to move the gripper");
    }
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
// Conversion from quaternion to degrees euler angles
std::vector<double> ManipulatorMenu::euler_from_quaternion(const geometry_msgs::Quaternion quaternion)
{
  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(quaternion, tf_quaternion);

  // Get Euler angles
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

  // Store the angles in a vector
  std::vector<double> euler_angles = {roll*180.0/M_PI,pitch*180.0/M_PI,yaw*180.0/M_PI};

  // Check if angles are in the interval (-180,180]
  for (unsigned int k = 0; k < 3; k++)
  {
    if      (euler_angles[k] < -179.9999999999) {euler_angles[k] += 360.;}
    else if (euler_angles[k] > +180.          ) {euler_angles[k] -= 360.;}
  }

  return euler_angles; 
}

// --------------------- DEG-RADIANS HANDLER -------------------
std::vector<double> ManipulatorMenu::deg_from_rad(const std::vector<double> joint_rad)
{
  std::vector<double> joint_deg(joint_rad.size(),0);
  // Iterate over input vector
  for (unsigned int k; k < joint_deg.size(); k++)
      {joint_deg[k] = joint_rad[k]*180/M_PI;}
  // Return result
  return joint_deg;
}

std::vector<double> ManipulatorMenu::rad_from_deg(const std::vector<double> joint_deg)
{
  std::vector<double> joint_rad(joint_deg.size(),0);
  // Iterate over input vector
  for (unsigned int k; k < joint_rad.size(); k++)
      {joint_rad[k] = joint_deg[k]/180*M_PI;}
  // Return result
  return joint_rad;
}

// --------------------- JOYSTICK HANDLER -------------------

// Joy callback
void ManipulatorMenu::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
  std::vector<double> dx_des = {0.,0.,0.,0.,0.,0.};
  dx_des[0]=  joy->axes[1]*joy_step_;       //  asse x
  dx_des[1] = joy->axes[0]*joy_step_;       //  asse y
  // dx_des(2) = -joy->axes[2]*joy_step_;       //  asse z
  // dx_des(3) = -joy->axes[3]*joy_step_;       //  asse rx
  // dx_des(4) = -joy->axes[4]*joy_step_;       //  asse ry
  // dx_des(5) = -joy->axes[5]*joy_step_;       //  asse rz
  move_Joystick(dx_des);
}

// Joystick move
geometry_msgs::Pose ManipulatorMenu::move_Joystick(const std::vector<double> dx)
{ 
  // ROS_INFO("dx_des(0): %f, dx_des(1): %f, dx_des(2): %f, dx_des(3): %f, dx_des(4): %f, dx_des(5): %f",
  //               dx[0],         dx[1],         dx[2],         dx[3],         dx[4],         dx[5]);
  
  // Get current EE pose
  std::vector<double> goal_pose = getEEpos_rpy();
  goal_pose[0] = goal_pose[0] + dx[0];
  goal_pose[1] = goal_pose[1] + dx[1];
  // goal_pose[2] = goal_pose[2] + dx[2];
  // goal_pose[3] = goal_pose[3] + dx[3];
  // goal_pose[4] = goal_pose[4] + dx[4];
  // goal_pose[5] = goal_pose[5] + dx[5];

  // return publishTcpIKGoal(goal_pose);
  return publishTcpIK_noplanner_Goal(goal_pose);
}


// --------------------- MENU HANDLER ---------------------

void ManipulatorMenu::printMenu()
{
  std::cout << "\n======= MANIPULATOR MENU =======\n";
  std::cout << "\n======= Joint/tcp moving test options =======\n";
  std::cout << "0. Give a TCP goal through InvKine, with fake controller\n";
  std::cout << "1. Test a joint goal\n";
  std::cout << "2. Test a TCP goal\n";
  std::cout << "3. Give a joint goal\n";
  std::cout << "4. Give a TCP goal\n";
  std::cout << "5. Give a TCP goal through InvKine\n";
  std::cout << "6. Move a defined joint\n";
  std::cout << "\n======= Carthesian moving test options =======\n";
  std::cout << "7. Move the robot along x\n";
  std::cout << "8. Move the robot along y\n";
  std::cout << "9. Move the robot along z\n";
  std::cout << "\n======= Tcp orientation options =======\n";
  std::cout << "10.Change TCP orientation\n";
  std::cout << "11.Rotate the TCP around x\n";
  std::cout << "12.Rotate the TCP around y\n";
  std::cout << "13.Rotate the TCP around z\n";
  std::cout << "14.Get a fixed TCP orientation\n";
  std::cout << "\n======= Handle objects in the planning scene =======\n";
  std::cout << "15.Add an object to the scene\n";
  std::cout << "16.Delete an object from the scene\n";
  std::cout << "\n======= Visualize current robot state =======\n";
  std::cout << "17.Visualize joints state\n";
  std::cout << "18.Visualize current tcp pose\n";
  std::cout << "\n======= Home positions setting =======\n";
  std::cout << "19.Go to home position (gripper down)\n";
  std::cout << "20.Go to home position (gripper at the front)\n";
  std::cout << "\n======= CoppeliaSim handling =======\n";
  std::cout << "21. To start twin Coppelia simulation\n";
  std::cout << "22. To stop  twin Coppelia simulation\n";
  std::cout << "23. To save  twin CoppeliaSim scene\n";
  std::cout << "24. To change cable pose in the CoppeliaSim scene\n";
  std::cout << "\n======= Cartesian move =======\n";
  std::cout << "25.Make a cartesian move\n";
  std::cout << "\n======= Fake gripper control =======\n";
  std::cout << "26.Open the gripper\n";
  std::cout << "27.Close the gripper\n";
  std::cout << "28.Set the position of the gripper\n";
  std::cout << "29.Grab an object at the gripper\n";
  std::cout << "30.Detach an object from the gripper\n";
  std::cout << "\n======= Real gripper control =======\n";
  std::cout << "31.Open  real gripper\n";
  std::cout << "32.Close real gripper\n";
  std::cout << "33.Move  real gripper to a given position \n";
  std::cout << "\n======= Closing ROS menu =======\n";
  std::cout << "34.Shutdown the menu\n";

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
  double step;                // Linear move length along axis
  std::vector<double> rot;    // End effector rotation
  std::vector<double> ee_pos; // End effector position
  switch (choice)
  {
  case 0:
    ROS_INFO("You selected Option 0");
    userTcpIK_no_planner_Goal();
    break;
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
    ROS_INFO("You selected Option 6");
    oneJointMove_user();
    break;

  case 7:
    ROS_INFO("You selected Option 7");
    
    std::cout << "Insert how many metres you want to move along x: \n";
    std::cin >> step;
    move_along_x(step);
    break;

  case 8:
    ROS_INFO("You selected Option 8");
    
    std::cout << "Insert how many metres you want to move along y:\n";
    std::cin >> step;
    move_along_y(step);
    break;
  case 9:
    ROS_INFO("You selected Option 9");
    
    std::cout << "Insert how many metres you want to move along z:\n";
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
    ROS_INFO("You selected Option 11");
    std::cout << "Insert the rotation around X axis you want to do.\n";
    double x_rot; 
    std::cout << " X rotation: "; std::cin >> x_rot;
    rotate_around_x(x_rot);
    break;

  case 12:
    ROS_INFO("You selected Option 12");
    std::cout << "Insert the rotation around Y axis you want to do.\n";
    double y_rot; 
    std::cout << " Y rotation: "; std::cin >> y_rot;
    rotate_around_y(y_rot);
    break;

  case 13:
    ROS_INFO("You selected Option 13");
    std::cout << "Insert the rotation around Z axis you want to do.\n";
    double z_rot; 
    std::cout << " Z rotation: "; std::cin >> z_rot;
    rotate_around_z(z_rot);
    break;

  case 14:
    ROS_INFO("You selected Option 14");
    std::cout << "Insert the FIXED orientation of the EE you want to have.\n";    
    rot = {0.,0.,0.}; 
    std::cout << " X rotation: "; std::cin >> rot[0];
    std::cout << " Y rotation: "; std::cin >> rot[1];
    std::cout << " Z rotation: "; std::cin >> rot[2];
    change_tcp_orient(rot);
    break;

  case 15:
    ROS_INFO("You selected Option 15");
    addCollObj();
    break;
  case 16:
    ROS_INFO("You selected Option 16");
    deleteCollObj();
    break;
  case 17:
    ROS_INFO("You selected Option 17");
    jointStateVisualizer();    
    break;

  case 18:
    ROS_INFO("You selected Option 18");
    ee_pos = getEEpos_rpy();
    std::cout << " EE - X position: " << ee_pos[0] << std::endl;
    std::cout << " EE - Y position: " << ee_pos[1] << std::endl;
    std::cout << " EE - Z position: " << ee_pos[2] << std::endl;
    std::cout << " EE - X rotation: " << ee_pos[3] << std::endl;
    std::cout << " EE - Y rotation: " << ee_pos[4] << std::endl;
    std::cout << " EE - Z rotation: " << ee_pos[5] << std::endl;
    break;

  
  case 19:
    ROS_INFO("You selected Option 19");
    ROS_INFO("Go to home position, gripper down ...");
    goHome(0);
    break;
  case 20:
    ROS_INFO("You selected Option 20");
    ROS_INFO("Go to home position, gripper at the front ...");
    goHome(1);
    break;
  case 21:
    ROS_INFO("You selected Option 21");
    ROS_INFO("Start Coppelia simulation");
    startCoppeliaSim();
    break;
  case 22:
    ROS_INFO("You selected Option 22");
    ROS_INFO("Stop Coppelia simulation");
    stopCoppeliaSim();
    break;
  case 23:
    ROS_INFO("You selected Option 23");
    ROS_INFO("Save current Coppelia scene");
    saveCoppeliaScene();
    break;
  case 24:
    ROS_INFO("You selected Option 24");
    ROS_INFO("Change cable position in the CoppeliaSim scene");
    changeCoppeliaCablePose();
    break;
  case 25:
    ROS_INFO("You selected Option 25");
    userCartesianMove();
    break;
  case 26:
    ROS_INFO("You selected Option 26");
    ROS_INFO("Opening the gripper ...");
    openGripper();
    break;
  case 27:
    ROS_INFO("You selected Option 27");
    ROS_INFO("Closing the gripper ...");
    closeGripper();
    break;
  case 28:
    ROS_INFO("You selected Option 28");
    ROS_INFO("Gripper moving setting");
    userGripperMove();
    break;  
  case 29:
    ROS_INFO("You selected Option 29");
    ROS_INFO("Grab an object to the gripper");
    grabObjGripper();
    break;
  case 30:
    ROS_INFO("You selected Option 30");
    ROS_INFO("Detach an object from the gripper");
    detachObjGripper();
    break;
  case 31:
    ROS_INFO("You selected Option 31");
    ROS_INFO("Opening real gripper");
    openRealGripper();
    break;
  case 32:
    ROS_INFO("You selected Option 32");
    ROS_INFO("Closing real gripper");
    closeRealGripper();
    break;
  case 33:
    ROS_INFO("You selected Option 33");
    ROS_INFO("Set a real gripper position");
    float gripper_pos;
    std::cin >> gripper_pos;
    moveRealGripper(gripper_pos);
    break;
  case 34:
    ROS_INFO("Exiting...\n");
    ros::shutdown();
    break;
  default:
    ROS_WARN("Invalid choice. Please choose a valid option.");
    break;
  }
}