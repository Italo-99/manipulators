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

ManipulatorMenu::ManipulatorMenu(ManipulatorMenuParams& params)
{
  // Check for parameters
  params_ = params;
  if (!nh_.getParam(params_.node_name+"/ee_joint_name", params_.ee_joint_name))
  {
    ROS_WARN("EE joint name param not defined! Assuming EE passed as object arg or as default.");
    // ee_joint_name  = "robotiq85_gripper/finger_joint"; EXAMPLE
    params_.ee_joint_name  = params.ee_joint_name;
  }
  if (!nh_.getParam(params_.node_name+"/ros_freq", params_.ros_freq))
  {
    ROS_WARN("ROS loop frequency param not defined! Assuming default value passed as object arg or as default.");
    params_.ros_freq = params.ros_freq;
  }
  if (params_.ros_freq < 0.1) 
  {
    ROS_WARN("ROS loop frequency param too low or not passed! Assuming minimum value of 10 Hz.");
    params_.ros_freq = 10.;
  }
  if (!nh_.getParam(params_.node_name+"/manipulator_name", params_.manipulator_name))
  {
    ROS_WARN("Manipulator name param not defined! Assuming default value passed as object arg or as default.");
    params_.manipulator_name = params.manipulator_name;
  }
  if (!nh_.getParam(params_.node_name+"/enable_coppelia", params_.enable_coppelia))
  {
    ROS_WARN("Coppelia enable param not defined! Assuming default value passed as object arg or as default.");
    params_.enable_coppelia = params.enable_coppelia;
  }
  if (!nh_.getParam(params_.node_name+"/enable_sim_gripper", params_.enable_sim_gripper))
  {
    ROS_WARN("Gripper enable param not defined! Assuming default value passed as object arg or as default.");
    params_.enable_sim_gripper = params.enable_sim_gripper;
  }
  if (!nh_.getParam(params_.node_name+"/enable_real_gripper", params_.enable_real_gripper))
  {
    ROS_WARN("Real gripper enable param not defined! Assuming default value passed as object arg or as default.");
    params_.enable_real_gripper = params.enable_real_gripper;
  }
  if (!nh_.getParam(params_.node_name+"/gripper_topic", params_.gripper_topic))
  {
    ROS_WARN("Real gripper command topic param not defined! Assuming default value passed as object arg or as default.");
    params_.gripper_topic = params.gripper_topic;
  }
  if (!nh_.getParam(params_.node_name+"/joint_names", params_.joint_names))
  {
    ROS_WARN("Joint names param not defined! Assuming default value passed as object arg or as default.");
    params_.joint_names = params.joint_names;
  }
  if (!nh_.getParam(params_.node_name+"/base_link_name", params_.base_link_name))
  {
    ROS_WARN("Base link name param not defined! Assuming default value passed as object arg or as default.");
    params_.base_link_name = params.base_link_name;
  }

  // Display Manipulator setup
  ROS_INFO("Manipulator menu initialized with the following setup:");
  ROS_INFO("Manipulator name: %s", params_.manipulator_name.c_str());

  for (int k = 0; k< params_.joint_names.size(); k++)
  {
    ROS_INFO("Joint %d name: %s", k, params_.joint_names[k].c_str());
  }


  // Init arrays
  for (const std::string& name : params_.joint_names) {joints_map_group_[name] = 0.;}
  joints_values_group_.resize(params_.joint_names.size());
  current_joint_pose_.name      = params_.joint_names;

  // --------------------- PUBS & SUBS DELCARATIONS ---------------------
  jointGoalPublisher_           = nh_.advertise<sensor_msgs::JointState>(params_.manipulator_name+"/desired_joint_pose", 1);
  jointGoalPublisherNoPlanner_  = nh_.advertise<sensor_msgs::JointState>(params_.manipulator_name+"/noplan_joint_pose", 1);
  tcpPosePublisher_             = nh_.advertise<geometry_msgs::Pose>(params_.manipulator_name+"/desired_tcp_pose", 1);
  tcpPoseIKPublisher_           = nh_.advertise<geometry_msgs::Pose>(params_.manipulator_name+"/desired_tcpIK_pose", 1);
  tcpPoseIK_noplannerPub_       = nh_.advertise<geometry_msgs::Pose>(params_.manipulator_name+"/noplan_tcpIK_pose", 1);
  carthesianMovePublisher_      = nh_.advertise<geometry_msgs::PoseArray>(params_.manipulator_name+"/desired_cartesian_move", 1);
  display_goal_pub_             = nh_.advertise<geometry_msgs::PoseStamped>(params_.manipulator_name+"/display_robot_goal", 1);
  eepose_pub_                   = nh_.advertise<geometry_msgs::PoseStamped>(params_.manipulator_name+"/display_ee_pose", 1);
  collisionObjectPublisher_     = nh_.advertise<moveit_msgs::CollisionObject>(params_.manipulator_name+"/add_collision_object", 1);
  moveGripperPublisher_         = nh_.advertise<std_msgs::Float64>(params_.ee_joint_name+"/motor_control", 1);
  jointStateSubscriber_         = nh_.subscribe("/joint_states", 1, &ManipulatorMenu::jointStateCallback, this);

  // --------------------- Kinematics client init ---------------------
  invKineClient_                = nh_.serviceClient<manipulators::InvKine>(params_.manipulator_name+"/invKine");
  pseudoInvClient_              = nh_.serviceClient<manipulators::PseudoInverse>(params_.manipulator_name+"/pseudoInverse");
  fKineClient_                  = nh_.serviceClient<manipulators::FKine>(params_.manipulator_name+"/FKine");
  jacobianClient_               = nh_.serviceClient<manipulators::Jacobian>(params_.manipulator_name+"/Jacobian");

  // --------------------- CoppeliaSim client init ---------------------
  if (params_.enable_coppelia)
  {coppeliaClient_ = nh_.serviceClient<manipulators::CoppeliaMenu>("coppelia_menu");}

  // --------------------- Gripper client init ---------------------
  if (params_.ee_joint_name != "" && params.enable_sim_gripper == true)
  {
    grab_client_    = nh_.serviceClient<std_srvs::SetBool>(params_.ee_joint_name+"/grabbing_gripper");
    gripper_client_ = nh_.serviceClient<std_srvs::SetBool>(params_.ee_joint_name+"/move_gripper");

    if (params_.enable_real_gripper)
    {real_gripper_client_ = nh_.serviceClient<gripper::RobotiQGripperControl>(params_.gripper_topic);}
  }
}

// --------------------- PUBLIC FUNCTIONS ---------------------

std::vector<double> ManipulatorMenu::invKineClient(const geometry_msgs::Pose pose)
{
    std::vector<double> joint_values;

    // Set target pose
    invKine_srv_.request.target_pose = pose;

    // Call the srv
    if (invKineClient_.call(invKine_srv_))
    {
        // ROS_INFO("Inverse kinematics joint values received:");
        // for (auto &joint_value : srv.response.joint_values) {ROS_INFO_STREAM(joint_value);}
        for (unsigned int k = 0; k < invKine_srv_.response.joint_values.layout.dim[0].size; k++)
        {
          joint_values.push_back(invKine_srv_.response.joint_values.data[k]);
        }
    }
    else {ROS_ERROR("Failed to call service invKine");}

    return joint_values;
}

Eigen::MatrixXd ManipulatorMenu::pseudoInverseClient()
{
  Eigen::MatrixXd matrix(params_.joint_names.size(), 6);

  if (pseudoInvClient_.call(pseudoInv_srv_))
  {
    // ROS_INFO("Pseudoinverse matrix received:");

    // Assign data from Float64MultiArray to Eigen::MatrixXd
    for (int i = 0; i < params_.joint_names.size(); ++i)
    {
      for (int j = 0; j < 6; ++j)
      {
          matrix(i, j) = pseudoInv_srv_.response.pseudo_inverse.data[i * params_.joint_names.size() + j];
      }
    }
  } 
  else {ROS_ERROR("Failed to call service pseudoInverse");}

  return matrix;
}

geometry_msgs::Pose ManipulatorMenu::getCurrentFKineClient()
{
  geometry_msgs::Pose pose;
  if (fKineClient_.call(fKine_srv_))
  {
    // ROS_INFO("Forward Kinematics Pose received:");
    // ROS_INFO_STREAM(fKine_srv_.response.tcp_pose);
    pose.position.x     = fKine_srv_.response.tcp_pose.position.x;
    pose.position.y     = fKine_srv_.response.tcp_pose.position.y;
    pose.position.z     = fKine_srv_.response.tcp_pose.position.z;
    pose.orientation.x  = fKine_srv_.response.tcp_pose.orientation.x;
    pose.orientation.y  = fKine_srv_.response.tcp_pose.orientation.y;
    pose.orientation.z  = fKine_srv_.response.tcp_pose.orientation.z;
    pose.orientation.w  = fKine_srv_.response.tcp_pose.orientation.w;
  }
  else {ROS_ERROR("Failed to call service getCurrentFKine");}
  return pose;
}

Eigen::MatrixXd ManipulatorMenu::getJacobianClient()
{
  Eigen::MatrixXd matrix(6, params_.joint_names.size());
      
  if (jacobianClient_.call(jacobian_srv_))
  {
    // ROS_INFO("Jacobian matrix received successfully:");
    // Assign data from Float64MultiArray to Eigen::MatrixXd
    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < params_.joint_names.size(); ++j)
      {
          matrix(i, j) = jacobian_srv_.response.jacobian.data[i * 6 + j];
      }
    }
  }
  else {ROS_ERROR("Failed to call service getJacobian");}
    
  return matrix;
}

// --------------------- ROS HANDLER ---------------------

// Asynchronous spinner for ROS routines without user menu
void ManipulatorMenu::spinner()
{
    // Setup a rate for ROS loop execution
    ros::Rate r(params_.ros_freq);
    
    // ROS loop
    while (ros::ok())
    {
        // ROS spinner        
        ros::spinOnce();
        getEEpose();

        // Test funtion for InvKine computations time measurement: about 10-20 ms, not acceptable
        // ros::Time start = ros::Time::now();
        // invKineClient(getCurrentFKineClient());
        // getCurrentFKineClient();
        // pseudoInverseClient();
        // ROS_INFO("Total duration of the computations: %f", ros::Time::now().toSec()-start.toSec());

        // Wait for next loop time
        r.sleep();
    }

    // Shutdown ROS if Ctrl+C or Ctrl+D are pressed
    ros::shutdown();

}

// Asynchronous spinner for ROS routines with user menu
void ManipulatorMenu::spinnerMenu()
{
  // Initialize user choice variable
  int userChoice = 0;

  while (ros::ok())
  {
    ros::spinOnce();                // ROS Once spinner
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

// Publish a joint goal (with no planner) by passing a vector of joints in deg
sensor_msgs::JointState ManipulatorMenu::publishJointGoal_NoPlanner(const std::vector<double> joints) 
{
  // Fill the joint msg with degToRad conversion
  sensor_msgs::JointState jointStateMsg;
  jointStateMsg.header.stamp = ros::Time::now();
  for (unsigned int k = 0; k < joints.size(); k++)
  {
    jointStateMsg.position.push_back(joints[k]*M_PI/180);
  }
  
  return publishJointGoal_NoPlanner(jointStateMsg);
}

// Publish a joint goal by passing a JointState msg, without calling the planner
sensor_msgs::JointState ManipulatorMenu::publishJointGoal_NoPlanner(const sensor_msgs::JointState jointStateMsg)
{
  // Publish the JointState message
  jointGoalPublisherNoPlanner_.publish(jointStateMsg);
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
  robot_goal_msg.header.frame_id = params_.base_link_name;
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
  // Plan trajectory through inverse kinematics
  tcpPoseIKPublisher_.publish(tcpPoseMsg);

  // Display the goal on RViz
  geometry_msgs::PoseStamped robot_goal_msg;
  robot_goal_msg.header.frame_id = params_.base_link_name;
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
  // Plan trajectory through inverse kinematics
  tcpPoseIK_noplannerPub_.publish(tcpPoseMsg);

  // Display the goal on RViz
  geometry_msgs::PoseStamped robot_goal_msg;
  robot_goal_msg.header.frame_id = params_.base_link_name;
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
  geometry_msgs::Pose       current_pose = getCurrentFKineClient();
  geometry_msgs::PoseArray  waypoints;
  geometry_msgs::Pose       final_pose;
  waypoints.header.frame_id = params_.base_link_name;
  double step_axisX = 0.;
  double step_axisY = 0.;
  double step_axisZ = 0.;
  // Compute axis step
  if  (axis1 == axis2) {ROS_WARN("Error in axis input!"); return final_pose;}
  else 
  {
    if      (axis1 == 0) {step_axisX = pos1 - current_pose.position.x;}
    else if (axis1 == 1) {step_axisY = pos1 - current_pose.position.y;}
    else if (axis1 == 2) {step_axisZ = pos1 - current_pose.position.z;}
    if      (axis2 == 0) {step_axisX = pos2 - current_pose.position.x;}
    else if (axis2 == 1) {step_axisY = pos2 - current_pose.position.y;}
    else if (axis2 == 2) {step_axisZ = pos2 - current_pose.position.z;}
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
    wp.orientation  = current_pose.orientation;
    // Fill the position
    wp.position.x   = current_pose.position.x + (k+1)*step_axisX; 
    wp.position.y   = current_pose.position.y + (k+1)*step_axisY; 
    wp.position.z   = current_pose.position.z + (k+1)*step_axisZ; 
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
  ros::spinOnce();
  // Fill current joints pose as target
  std::vector<double> joint_target;
  for (unsigned int k = 0; k < params_.joint_names.size(); k++) 
  {
    joint_target.push_back(current_joint_pose_.position[k]*180/M_PI);
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
  {start_joint_pose = {0.,-90.,-90.,-90.,+90.,+60.};}
  else // gripper at the front
  {start_joint_pose = {0.,-90.,-90.,  0.,+90.,+60.};}
  if (params_.joint_names.size() != 6) 
  {
    for (unsigned int k = 0; k < params_.joint_names.size() - 6; k++)
    {
      start_joint_pose.push_back(0.);
    }
  }
  // Publish home joint goal 
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
  try {tf_buffer.canTransform(source_frame, target_frame, ros::Time(0), ros::Duration(0.2));} 
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
geometry_msgs::Pose ManipulatorMenu::getEEpose()
{
  // Compute the FKine between base_link and end-effector
  current_tcp_pose_.header.frame_id = params_.base_link_name;
  current_tcp_pose_.pose = getCurrentFKineClient();
  eepose_pub_.publish(current_tcp_pose_);
  return current_tcp_pose_.pose;
}

// Get EE pose as vector with RPY euler angles
std::vector<double> ManipulatorMenu::getEEpos_rpy()
{
  // Read current EE pose by FKine
  getEEpose();

  // Fill the rotation vector
  std::vector<double> tcp_rpy = euler_from_quaternion(current_tcp_pose_.pose.orientation);

  // Declaration of the pose vector
  std::vector<double> tcp_pose_rpy = {current_tcp_pose_.pose.position.x,current_tcp_pose_.pose.position.y,current_tcp_pose_.pose.position.z,tcp_rpy[0],tcp_rpy[1],tcp_rpy[2]};
  return tcp_pose_rpy;
}

// -------------------- SIMPLE MOVES ALONG CARTHESIAN AXES -----------------------//

// Set a carthesian move along x axis in metres
geometry_msgs::Pose ManipulatorMenu::move_along_x(const double x_step, bool cartesian)
{
  // Get current EE pose
  std::vector<double> goal_pose = getEEpos_rpy();
  // Update position along X
  goal_pose[0] = goal_pose[0] + x_step;
  if (cartesian)
  {
    uint8_t n_steps = std::max(int(x_step/0.1),1);
    return publishCartesianMove(0,1,goal_pose[0],goal_pose[1],n_steps);
  }
  else {return publishTcpGoal(goal_pose);}
}

// Set a carthesian move along x axis in metres
geometry_msgs::Pose ManipulatorMenu::move_along_y(const double y_step, bool cartesian)
{
  // Get current EE pose
  std::vector<double> goal_pose = getEEpos_rpy();
  // Update position along Y
  goal_pose[1] = goal_pose[1] + y_step;
  if (cartesian)
  {
    uint8_t n_steps = std::max(int(y_step/0.1),1);
    return publishCartesianMove(0,1,goal_pose[0],goal_pose[1],n_steps);
  }
  else {return publishTcpGoal(goal_pose);}
}

// Set a carthesian move along x axis in metres
geometry_msgs::Pose ManipulatorMenu::move_along_z(const double z_step, bool cartesian)
{
  // Get current EE pose
  std::vector<double> goal_pose = getEEpos_rpy();
  goal_pose[2] = goal_pose[2] + z_step;
  // Update position along Z
  if (cartesian)
  {
    uint8_t n_steps = std::max(int(z_step/0.1),1);
    return publishCartesianMove(0,2,goal_pose[0],goal_pose[2],n_steps);
  }
  else {return publishTcpGoal(goal_pose);}
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
  return publishTcpGoal(goal_pose);
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
  return publishTcpGoal(goal_pose);
}

// Set a relative rotation around x axis (in degrees)
geometry_msgs::Pose ManipulatorMenu::rotate_around_x(const double x_rot_step)
{
  // Get current EE pose
  std::vector<double> goal_pose = getEEpos_rpy();
  // Update tcp orient goal
  goal_pose[3] = goal_pose[3] + x_rot_step;
  return publishTcpGoal(goal_pose);
}

// Set a relative rotation around y axis (in degrees)
geometry_msgs::Pose ManipulatorMenu::rotate_around_y(const double y_rot_step)
{
  // Get current EE pose
  std::vector<double> goal_pose = getEEpos_rpy();
  // Update tcp orient goal
  goal_pose[4] = goal_pose[4] + y_rot_step;
  return publishTcpGoal(goal_pose);
}

// Set a relative rotation around z axis (in degrees)
geometry_msgs::Pose ManipulatorMenu::rotate_around_z(const double z_rot_step)
{
  // Get current EE pose
  std::vector<double> goal_pose = getEEpos_rpy();
  // Update tcp orient goal
  goal_pose[5] = goal_pose[5] + z_rot_step;
  return publishTcpGoal(goal_pose);
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

  obj.header.frame_id = params_.base_link_name;
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

    default:  // CYLINDER OR CONE
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

// ------------------- KINEMATICS PARAMS SETTERS ---------------------- //

// Set Jacobian-based speed control
void ManipulatorMenu::setJacobianSpeedControl(bool set)
{
  ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>(params_.manipulator_name+"/enable_jac_vel");
  std_srvs::SetBool srv;
  srv.request.data = set;
  client.call(srv);
  std::cout << srv.response.message << std::endl;
}

// Set Instantaneous kinematics mode
void ManipulatorMenu::setInstantKineMode(bool set)
{
  ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>(params_.manipulator_name+"/instKine_setter");
  std_srvs::SetBool srv;
  srv.request.data = set;
  client.call(srv);
  std::cout << srv.response.message << std::endl;
}

// Set new dynamic planners vel/acc params
void ManipulatorMenu::setNewPlannerParams(float new_vel,float new_acc)
{
  ros::ServiceClient client = nh_.serviceClient<manipulators::ChangePlannerParams>(params_.manipulator_name+"/change_planner_params");
  manipulators::ChangePlannerParams srv;
  srv.request.new_vel_factor = new_vel;
  srv.request.new_acc_factor = new_acc;
  client.call(srv);
  std::cout << srv.response.message << std::endl;
}

// Set Joints real time speed control
void ManipulatorMenu::setJsRealTimeControl(bool set)
{
  ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>(params_.manipulator_name+"/enable_js_rt_vel");
  std_srvs::SetBool srv;
  srv.request.data = set;
  client.call(srv);
  std::cout << srv.response.message << std::endl;
}

// --------------------- PRIVATE FUNCTIONS ---------------------

// --------------------- COPPELIA HANDLER ---------------------

// Send the request and show the response
void ManipulatorMenu::wait_for_response()
{
  if (coppeliaClient_.call(coppelia_srv_))
  {
    ROS_INFO("Simulation status: %d", coppelia_srv_.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service coppelia_menu");
  }
}
  
// --------------------- JOINT GOALS HANDLER ---------------------

void ManipulatorMenu::userJointGoal()
{
  // Declare the empty vector of joints goals
  std::vector<double> joints;
  
  // Take user degree angle for each joint
  std::cout << "Enter the values of the joint goal in degrees: \n";

  for (unsigned int k = 0; k < params_.joint_names.size(); k++)
  {
    double new_joint_value = 0.;
    std::cout << "Joint " << k+1 << " : ";
    std::cin >> new_joint_value;
    joints.push_back(new_joint_value);
  }

  publishJointGoal(joints);
}

void ManipulatorMenu::oneJointMove_user()
{
  int num = 0;
  double joint_rot = 0.0;
  std::cout << "Enter the joint to move in [0, "<< params_.joint_names.size() - 1 << "]: \n";
  std::cin >> num;
  std::cout << "Enter the rotation of the joint in deg: \n";
  std::cin >> joint_rot;
  oneJointMove(num,joint_rot);
}

// --------------------- TCP GOALS HANDLER ---------------------

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

  publishTcpGoal(position);
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

void ManipulatorMenu::userJoint_no_planner_Goal()
{
  // Declare the empty vector of joints goals
  std::vector<double> joints;
  
  // Take user degree angle for each joint
  std::cout << "Enter the values of the joint goal in degrees: \n";

  for (unsigned int k = 0; k < params_.joint_names.size(); k++)
  {
    double new_joint_value = 0.;
    std::cout << "Joint " << k+1 << " : ";
    std::cin >> new_joint_value;
    joints.push_back(new_joint_value);
  }

  publishJointGoal_NoPlanner(joints);
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
  ros::spinOnce();
  for (unsigned int k = 0; k < params_.joint_names.size(); k++)
  {
    std::cout << "Joint " << k << " : " << current_joint_pose_.position[k]*180/M_PI << std::endl;
  }
}

void ManipulatorMenu::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joints_state) 
{
  
  // Map to store couples joint name - joint values
  static std::unordered_map<std::string, double>::iterator it;
  uint counter_group  = 0;

  for (uint i = 0; i < joints_state->name.size(); i++)
  {
    // Look for joints group names within joints current state
    it = joints_map_group_.find(joints_state->name[i]);
    // Exclude last link (gripper) from the search
    if (it != joints_map_group_.end())
    {
      // At the second position of the iteration, insert current joint position and velocity
      it->second = joints_state->position[i];
  
      // Increment the number of joints received from the joints state subscriber
      counter_group++;

      // If we have reached the last joint of the group
      if (counter_group == params_.joint_names.size())
      {
        // Iterate over the joints
        for (uint k = 0; k < params_.joint_names.size(); k++)
        {
          // Store the joints values from the joints map
          joints_values_group_[k] = joints_map_group_[params_.joint_names[k]];
        }

        // Log gripper planning group
        ROS_INFO_ONCE("%s joints values received by the menu interface.", params_.manipulator_name.c_str());
      }
    }
  }

  // Store the value into the global (public) class variable
  current_joint_pose_.position  = joints_values_group_;
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

// --------------------- MENU HANDLER ---------------------

void ManipulatorMenu::printMenu()
{
  std::cout << "\n======= MANIPULATOR MENU =======\n";
  std::cout << "\n======= Joint/tcp moving test options =======\n";
  std::cout << "1. Give a TCP goal through InvKine, with fake controller\n";
  std::cout << "2. Give a joint goal, with fake controller\n";
  std::cout << "3. Give a joint goal to MoveIt\n";
  std::cout << "4. Give a TCP goal to MoveIt\n";
  std::cout << "5. Give a TCP goal through InvKine to MoveIt\n";
  std::cout << "6. Move a defined joint\n";
  std::cout << "\n======= Carthesian moves test options =======\n";
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
  std::cout << "\n======= Cartesian move =======\n";
  std::cout << "21.Make a cartesian move\n";
  if (params_.enable_coppelia)
  {
    std::cout << "\n======= CoppeliaSim handling =======\n";
    std::cout << "22. To start twin Coppelia simulation\n";
    std::cout << "23. To stop  twin Coppelia simulation\n";
    std::cout << "24. To save  twin CoppeliaSim scene\n";
  }
  if (params_.enable_sim_gripper)
  {
    std::cout << "\n======= Fake gripper control =======\n";
    std::cout << "25.Open the gripper\n";
    std::cout << "26.Close the gripper\n";
    std::cout << "27.Set the position of the gripper\n";
    std::cout << "28.Grab an object at the gripper\n";
    std::cout << "29.Detach an object from the gripper\n";
  }
  if (params_.enable_real_gripper)
  {
    std::cout << "\n======= Real gripper control =======\n";
    std::cout << "30.Open  real gripper\n";
    std::cout << "31.Close real gripper\n";
    std::cout << "32.Move  real gripper to a given position \n";
  }
  std::cout << "\n======= Kinematics srvs =======\n";
  std::cout << "33. Get joint values through inverse kinematics of a given pose\n";
  std::cout << "34. Get current Jacobian\n";
  std::cout << "35. Get current Inverse Jacobian\n";
  std::cout << "36. Change velocity and acceleration as planner's parameters\n";
  std::cout << "\n======= Kinematics mode setter =======\n";
  std::cout << "37. Enable  the instantaneous kinematics mode for motors\n";
  std::cout << "38. Disable the instantaneous kinematics mode for motors\n";
  std::cout << "39. Enable  the jacobian speed control mode for robot joints\n";
  std::cout << "40. Disable the jacobian speed control mode for robot joints\n";
  std::cout << "41. Enable  the joints real time speed control mode\n";
  std::cout << "42. Disable the joints real time speed control mode\n";
  std::cout << "\n======= Closing ROS menu =======\n";
  std::cout << "43.Shutdown the menu\n";
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
  case 1:
    ROS_INFO("You selected Option 1");
    userTcpIK_no_planner_Goal();
    break;
  case 2:
    ROS_INFO("You selected Option 2");
    userJoint_no_planner_Goal();
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
    {
      ROS_INFO("You selected Option 18");
      std::vector<double> ee_pose = getEEpos_rpy();
      std::cout << " EE - X position: " << ee_pose[0] << std::endl;
      std::cout << " EE - Y position: " << ee_pose[1] << std::endl;
      std::cout << " EE - Z position: " << ee_pose[2] << std::endl;
      std::cout << " EE - X rotation: " << ee_pose[3] << std::endl;
      std::cout << " EE - Y rotation: " << ee_pose[4] << std::endl;
      std::cout << " EE - Z rotation: " << ee_pose[5] << std::endl;
    }
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
    userCartesianMove();
    break;
  case 22:
    ROS_INFO("You selected Option 22");
    ROS_INFO("Start Coppelia simulation");
    startCoppeliaSim();
    break;
  case 23:
    ROS_INFO("You selected Option 23");
    ROS_INFO("Stop Coppelia simulation");
    stopCoppeliaSim();
    break;
  case 24:
    ROS_INFO("You selected Option 24");
    ROS_INFO("Save current Coppelia scene");
    saveCoppeliaScene();
    break;
  case 25:
    ROS_INFO("You selected Option 25");
    ROS_INFO("Opening the gripper ...");
    openGripper();
    break;
  case 26:
    ROS_INFO("You selected Option 26");
    ROS_INFO("Closing the gripper ...");
    closeGripper();
    break;
  case 27:
    ROS_INFO("You selected Option 27");
    ROS_INFO("Gripper moving setting");
    userGripperMove();
    break;  
  case 28:
    ROS_INFO("You selected Option 28");
    ROS_INFO("Grab an object to the gripper");
    grabObjGripper();
    break;
  case 29:
    ROS_INFO("You selected Option 29");
    ROS_INFO("Detach an object from the gripper");
    detachObjGripper();
    break;
  case 30:
    ROS_INFO("You selected Option 30");
    ROS_INFO("Opening real gripper");
    openRealGripper();
    break;
  case 31:
    ROS_INFO("You selected Option 31");
    ROS_INFO("Closing real gripper");
    closeRealGripper();
    break;
  case 32:
    ROS_INFO("You selected Option 32");
    ROS_INFO("Set a real gripper position");
    float gripper_pos;
    std::cin >> gripper_pos;
    moveRealGripper(gripper_pos);
    break;
  case 33:
    {
      ROS_INFO("You selected Option 33");
      ROS_INFO("Set the pose you want to compute inverse kinematics.");
      geometry_msgs::Pose pose;
      float rx,ry,rz;
      std::cout << "X position: ";
      std::cin >> pose.position.x;
      std::cout << "Y position: ";
      std::cin >> pose.position.y;
      std::cout << "Z position: ";
      std::cin >> pose.position.z;
      std::cout << "X rotation (in degrees): ";
      std::cin >> rx;
      std::cout << "Y rotation (in degrees): ";
      std::cin >> ry;
      std::cout << "Z rotation (in degrees): ";
      std::cin >> rz;
      pose.orientation = quaternion_from_euler(rx,ry,rz);
      std::vector<double> joints = invKineClient(pose);
      for (unsigned int k = 0; k < joints.size(); k++)
      {
        ROS_INFO("Joint %d: %f", k, joints[k]);
      }
    }
    break;
  case 34:
    {
      ROS_INFO("You selected Option 34");
      Eigen::MatrixXd jac = getJacobianClient();
      ROS_INFO("Jacobian computed:\n");
      std::cout << jac << std::endl;
    }
    break;
  case 35:
    {
      ROS_INFO("You selected Option 35");
      Eigen::MatrixXd inv_jac = pseudoInverseClient();
      ROS_INFO("Inverse Jacobian computed:\n");
      std::cout << inv_jac << std::endl;
    }
    break;
  case 36:
    {
      ROS_INFO("You selected Option 36");
      float vel,acc;
      std::cout << "Insert new vel factor: ";
      std::cin  >> vel;
      std::cout << "Insert new acc factor: ";
      std::cin  >> acc;
      setNewPlannerParams(vel,acc);
    }
    break;
  case 37:
    {
      ROS_INFO("You selected Option 37");
      setInstantKineMode(true);
    }
    break;
  case 38:
    {
      ROS_INFO("You selected Option 38");
      setInstantKineMode(false);
    }
    break;
  case 39:
    {
      ROS_INFO("You selected Option 39");
      setJacobianSpeedControl(true);
    }
    break;
  case 40:
    {
      ROS_INFO("You selected Option 40");
      setJacobianSpeedControl(false);
    }
    break;
  case 41:
    {
      ROS_INFO("You selected Option 41");
      setJsRealTimeControl(true);
    }
    break;
  case 42:
    {
      ROS_INFO("You selected Option 42");
      setJsRealTimeControl(false);
    }
    break;
  case 43:
    ROS_INFO("You selected Option 41");
    ROS_INFO("Exiting...\n");
    ros::shutdown();
    break;
  default:
    ROS_WARN("Invalid choice. Please choose a valid option.");
    break;
  }
}