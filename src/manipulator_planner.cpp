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

// CLASS SOURCE IMLEMENTATION OF MANIPULATOR PLANNER -> CALLS AN OBJECT OF DYNAMIC PLANNER 

// The following code introduces the class of manipulator planner, an instance of the
// dynamic planner, plus a table object in RViz to test planning

// IMPORT LIBRARIES
#include "manipulators/ManipulatorPlanner.h"

// --------------------- PUBLIC CONSTRUCTOR ---------------------

ManipulatorPlanner::ManipulatorPlanner(std::string node_name)
{
  // ---------------------  TCP AND JOINT GOALS SUBSCRIBERS ---------------------

  // Planning callbacks
  tcp_goal_sub_           = nh_.subscribe("/desired_tcp_pose",   1, &ManipulatorPlanner::tcpGoalCallback,               this);
  joint_goal_sub_         = nh_.subscribe("/desired_joint_pose", 1, &ManipulatorPlanner::jointsGoalCallback,            this);
  tcp_goalIK_sub_         = nh_.subscribe("/desired_tcpIK_pose", 1, &ManipulatorPlanner::tcpGoalIKCallback,             this);

  // No planning callbacks
  joint_goal_noplan_sub_  = nh_.subscribe("/noplan_joint_pose",  1, &ManipulatorPlanner::jointsGoal_NoPlanner_Callback, this);
  tcp_goalIK_noplan_sub_  = nh_.subscribe("/noplan_tcpIK_pose",  1, &ManipulatorPlanner::tcpGoalIK_NoPlanner_Callback,  this);

  // tcp_goalSeq_sub_   = nh_.subscribe("/desired_tcpSeq_poses",   1, &ManipulatorPlanner::tcpGoalSeqCallback,    this);
  // tcpIK_goalSeq_sub_ = nh_.subscribe("/desired_tcpIKSeq_poses", 1, &ManipulatorPlanner::tcpIKGoalSeqCallback,  this);
  // joint_goalSeq_sub_ = nh_.subscribe("/desired_jointSeq_poses", 1, &ManipulatorPlanner::jointsGoalSeqCallback, this);

  carthesian_move_sub_ = nh_.subscribe("/desired_cartesian_move", 1, &ManipulatorPlanner::cartesianMoveCallback, this);

  // ---------------------  ADD COLLISION OBJECT SUBSCRIBER  ---------------------

  add_coll_obj_sub_ = nh_.subscribe("/add_collision_object", 1, &ManipulatorPlanner::addCollObjCallback, this);

  // ---------------------  PRIVATE VARIABLES SETUP  ---------------------------

  node_name_ = node_name;
  check_param();
  dynamic_behaviour_ = false;   // No replanning

  // Jacobian speed based control
  enaJacControl_sub_   = nh_.subscribe(manipulator_name_+"/enable_jacobian_speed_control",  1, &ManipulatorPlanner::jacobianControlSetterCallback, this);
  velJacSetpoint_sub_  = nh_.subscribe(manipulator_name_+"/cmd_vel",                        1, &ManipulatorPlanner::updateVelJacSetpoint,          this);

  // ---------------------  MOTOR CONTROLLERS FOR INVKINE  ---------------------------
  instKine_setter_sub_ = nh_.subscribe(manipulator_name_+"/instKine_setter", 1, &ManipulatorPlanner::instantKineSetterCallback, this);
  instKine_setter_pub_ = nh_.advertise<std_msgs::Bool>(manipulator_name_+"/instKine_setter", 1);

  j0_pub_ = nh_.advertise<std_msgs::Float64>(manipulator_name_+"/"+joint_names_[0]+"/motor_control", 1);
  j1_pub_ = nh_.advertise<std_msgs::Float64>(manipulator_name_+"/"+joint_names_[1]+"/motor_control", 1);
  j2_pub_ = nh_.advertise<std_msgs::Float64>(manipulator_name_+"/"+joint_names_[2]+"/motor_control", 1);
  j3_pub_ = nh_.advertise<std_msgs::Float64>(manipulator_name_+"/"+joint_names_[3]+"/motor_control", 1);
  j4_pub_ = nh_.advertise<std_msgs::Float64>(manipulator_name_+"/"+joint_names_[4]+"/motor_control", 1);
  j5_pub_ = nh_.advertise<std_msgs::Float64>(manipulator_name_+"/"+joint_names_[5]+"/motor_control", 1);

  // ---------------------- KINEMATICS SERVICES ---------------------------------------- //
  inv_kine_service_       = nh_.advertiseService(manipulator_name_ + "/invKine",       &ManipulatorPlanner::invKineCallback,        this);
  pseudo_inverse_service_ = nh_.advertiseService(manipulator_name_ + "/pseudoInverse", &ManipulatorPlanner::pseudoInverseCallback,  this);
  get_fkine_service_      = nh_.advertiseService(manipulator_name_ + "/FKine",         &ManipulatorPlanner::getCurrentFKineCallback,this);
  get_jacobian_service_   = nh_.advertiseService(manipulator_name_ + "/Jacobian",      &ManipulatorPlanner::getJacobianCallback,    this);

  // ---------------------- DYNAMIC PLANNER OBJECT ------------------------------------- //

  // Call to the dynamic planner constructor
  planner_ = new DynamicPlanner(manipulator_name_, joint_names_, vel_factor_, acc_factor_,
                                dynamic_behaviour_, sample_time_, max_velocity_);  

  // Set the sim mode for the dynamic planner
  planner_->setSimMode(sim_);

  // --------------------- EXAMPLE ENVIRONMENT SETUP --------------------- //

  // // How to add a table to the scene
  // std::vector<double> dim_obj = {4.,4.,0.079};
  // double pos_obj[]            = {0.,0.,-0.04};
  // double rot_obj[]            = {0.,0.,0.,1.};
  // createObj("table", 1, dim_obj, pos_obj,rot_obj,0);

  // // How to delete from the scene
  // createObj("table", 1, dim_obj, pos_obj,rot_obj,1);  
}

// Destructor of the object manipulator planner's class
ManipulatorPlanner::~ManipulatorPlanner() {delete planner_;}

// ---------------------  PUBLIC FUNCTIONS --------------------- //

// Manipulator planner spin function
void ManipulatorPlanner::spinner()
{

  // Setup a rate for ROS loop execution
  ros::Rate r(ros_freq_);

  // ROS loop
  while (ros::ok())
  {
    // Call the spinner for object related fuctions
    planner_->spinner();

    // Test funtion for InvKine computations
    // ros::Time start = ros::Time::now();
    // planner_->invKine(get_manip_FKine());
    // planner_->pseudoInverse(planner_->getJacobian());
    // ROS_INFO("Total duration of the computations: %f", ros::Time::now().toSec()-start.toSec());

    // Jacobian speed control
    ros::Time start = ros::Time::now();
    if (jac_control_){jacobianControl();}
    ROS_INFO("Total duration of the computations: %f", ros::Time::now().toSec()-start.toSec());

    // Wait for next loop time
    r.sleep();
  }
}

// -------------------- JACOBIAN SPEED CONTROL ----------------- //

// Set the jacobian speed based control
void ManipulatorPlanner::jacobianControlSetterCallback(const std_msgs::Bool::ConstPtr& msg)
{
  jac_control_ = msg->data;
  arm_vel_cmd_[0] = 0.;
  arm_vel_cmd_[1] = 0.;
  arm_vel_cmd_[2] = 0.;
  arm_vel_cmd_[3] = 0.;
  arm_vel_cmd_[4] = 0.;
  arm_vel_cmd_[5] = 0.;
}

// Execute the jacobian based control
void ManipulatorPlanner::jacobianControl()
{
  // Compute the speed
  Eigen::Matrix<double,6,1> dq = get_manip_InvJacobian()*arm_vel_cmd_;

  // Convert joints state into Eigen::MatrixXd
  Eigen::Matrix<double,6,1> q;
  q[0] = planner_->joints_values_group_[0];
  q[1] = planner_->joints_values_group_[1];
  q[2] = planner_->joints_values_group_[2];
  q[3] = planner_->joints_values_group_[3];
  q[4] = planner_->joints_values_group_[4];
  q[5] = planner_->joints_values_group_[5];

  // Update joint position setpoint
  Eigen::Matrix<double,6,1> qd = q + dq / ros_freq_;

  // Build the msg for the joints setpoint
  sensor_msgs::JointState js;
  js.name     = joint_names_;
  js.position.resize(qd.size());
  js.velocity.resize(dq.size());

  // Insert positions setpoint
  js.position[0] = qd[0];
  js.position[1] = qd[1];
  js.position[2] = qd[2];
  js.position[3] = qd[3];
  js.position[4] = qd[4];
  js.position[5] = qd[5];

  // Insert velocity  setpoint
  js.velocity[0] = dq[0];
  js.velocity[1] = dq[1];
  js.velocity[2] = dq[2];
  js.velocity[3] = dq[3];
  js.velocity[4] = dq[4];
  js.velocity[5] = dq[5];

  // Send the goal to the dynamic planner V1
  planner_->moveRobot(js);
}

// Update the velocity setpoint of the arm for the jacobian speed based control
void ManipulatorPlanner::updateVelJacSetpoint(const geometry_msgs::Twist::ConstPtr& msg)
{    
    // Map the linear velocity components from the Twist message
    arm_vel_cmd_[0] = msg->linear.x; // X component of linear velocity
    arm_vel_cmd_[1] = msg->linear.y; // Y component of linear velocity
    arm_vel_cmd_[2] = msg->linear.z; // Z component of linear velocity
    
    // Map the angular velocity components from the Twist message
    arm_vel_cmd_[3] = msg->angular.x; // X component of angular velocity
    arm_vel_cmd_[4] = msg->angular.y; // Y component of angular velocity
    arm_vel_cmd_[5] = msg->angular.z; // Z component of angular velocity
}

// ---------------------- SERVER FUNCTIONS ---------------------- //

// Function to handle inverse kinematics service
bool ManipulatorPlanner::invKineCallback(manipulators::InvKine::Request  &req,
                                         manipulators::InvKine::Response &res)              // TODO: test
{
  std::vector<double> joint_values = planner_->invKine(req.target_pose);
  // Convert vector to MultiArray for response
  std_msgs::Float64MultiArray output;
  output.layout.dim.push_back(std_msgs::MultiArrayDimension());
  output.layout.dim[0].size = joint_values.size();
  output.data.assign(joint_values.data(), joint_values.data() + joint_values.size());
  res.joint_values = output;
  // res.message = "Inverse Kinematics computed successfully";
  return true;

  // std::vector<double> joint_values = planner_->invKine(req.target_pose);
  // res.joint_values = joint_values;
  // return true;
}

// Function to handle pseudoinverse service
bool ManipulatorPlanner::pseudoInverseCallback(manipulators::PseudoInverse::Request  &req,
                                               manipulators::PseudoInverse::Response &res)  // TODO: test
{
  Eigen::MatrixXd pseudo_inv = get_manip_InvJacobian();

  // Convert Eigen matrix to MultiArray for response
  std_msgs::Float64MultiArray output;
  output.layout.dim.push_back(std_msgs::MultiArrayDimension());
  output.layout.dim.push_back(std_msgs::MultiArrayDimension());
  output.layout.dim[0].size = pseudo_inv.rows();
  output.layout.dim[1].size = pseudo_inv.cols();
  output.data.assign(pseudo_inv.data(), pseudo_inv.data() + pseudo_inv.size());
  res.pseudo_inverse = output;
  // res.message = "Inverse Jacobian computed successfully";
  return true;
}

// Service for forward kinematics (no input needed from client)
bool ManipulatorPlanner::getCurrentFKineCallback(manipulators::FKine::Request  &req,
                                                 manipulators::FKine::Response &res)          // TODO: test
{
    geometry_msgs::Pose pose = get_manip_FKine();
    res.tcp_pose = pose;
    // res.message = "Forward Kinematics Pose computed successfully";
    return true;
}

// Service for Jacobian (no input needed from client)
bool ManipulatorPlanner::getJacobianCallback(manipulators::Jacobian::Request  &req,
                                             manipulators::Jacobian::Response &res)           // TODO: test
{
    Eigen::MatrixXd jacobian = planner_->getJacobian();
    
    std_msgs::Float64MultiArray output;
    output.layout.dim.push_back(std_msgs::MultiArrayDimension());
    output.layout.dim.push_back(std_msgs::MultiArrayDimension());
    output.layout.dim[0].size = jacobian.rows();
    output.layout.dim[1].size = jacobian.cols();
    output.data.assign(jacobian.data(), jacobian.data() + jacobian.size());
    res.jacobian = output;
    // res.message = "Jacobian computed successfully";
    return true;
}

// ---------------------- PRIVATE FUNCTIONS --------------------- //

// ----------------------- UTILS FUNCTIONS ---------------------- //

// External command to enable instantaneous kinematics
void ManipulatorPlanner::set_instKine(bool set)
{
  std_msgs::Bool msg;
  msg.data = set;
  instKine_setter_pub_.publish(msg);
}

// Check manipulators parameters passed to the node
void ManipulatorPlanner::check_param()
{
  // If one of the following parameters has not been defined, shutdwon ROS
  // The names of the params are passed with the prefix of the name of the node

  // Check for manipulator name parameter
  if (!nh_.getParam(node_name_+"/manipulator_name", manipulator_name_))
  {
    ROS_ERROR("Manipulator name not defined");
    ros::shutdown();
    return;
  }

  // Check for joint names parameter
  if (!nh_.getParam(node_name_+"/joint_names", joint_names_))
  {
    ROS_ERROR("Joint names not defined");
    ros::shutdown();
    return;
  }

  // Check for ee name parameter
  if (!nh_.getParam(node_name_+"/ee_name", ee_name_))
  {
    ROS_ERROR("End-effector name not defined");
    ros::shutdown();
    return;
  }

  // Check for robot base name parameter
  if (!nh_.getParam(node_name_+"/base_name", base_name_))
  {
    ROS_ERROR("Base name not defined");
    ros::shutdown();
    return;
  }

  // The following parameters can also be unset, but it's better to show a warning to the console to make the user aware
  // Check for velocity factor parameter
  if (!nh_.getParam(node_name_+"/vel_factor", vel_factor_))
  {
    ROS_WARN("Velocity factor not defined, assuming 0.5");
    // If the parameter has not been set by the user, it is set here
    vel_factor_ = 0.5;
  }

  // Check for acceleration factor parameter
  if (!nh_.getParam(node_name_+"/acc_factor", acc_factor_))
  {
    ROS_WARN("Acceleration factor not defined, assuming 0.5");

    // If the parameter has not been set by the user, it is set here
    acc_factor_ = 0.5;
  }

  // Check for ROS node loop frequency parameter
  if (!nh_.getParam(node_name_+"/ros_freq", ros_freq_))
  {
    ROS_WARN("ROS node loop frequency not defined, assuming 500 Hz.");

    // If the parameter has not been set by the user, it is set here
    ros_freq_ = 500;
  }

  // Check for instantaneous move parameter
  if (!nh_.getParam(node_name_+"/inst_kine", inst_kine_))
  {
    ROS_WARN("Instantaneous kinematics param not defined, assuming true.");

    // If the parameter has not been set by the user, it is set here
    inst_kine_ = true;
  }

  // Check sampling time parameters for cartesian planner
  if (!nh_.getParam(node_name_+"/sample_time", sample_time_))
  {
    ROS_WARN("Sample time param not defined! Assuming default value as 0.002.");
    sample_time_ = 0.002;
  }
  if (!nh_.getParam(node_name_+"/max_velocity", max_velocity_))
  {
    ROS_WARN("Sample time param not defined! Assuming default value as 0.5.");
    max_velocity_ = 0.5;
  }

  // Get simulation status from the user (simulation or debug)
  nh_.getParam(node_name_+"/sim", sim_);
}

// Creation of a collision object
void ManipulatorPlanner::createObj( const std::string&  name,
                                    const int           obj_type, 
                                    std::vector<double> obj_dims, 
                                    double              obj_pos[], 
                                    double              rot_pos[],
                                    uint                operation)
{
  // Creation of the obj
  moveit_msgs::CollisionObject obj;
  // Set header msg
  obj.header.seq = 1;
  obj.header.stamp = ros::Time::now();
  // Set frames
  obj.header.frame_id = base_name_;
  obj.id              = name;
  // Set pose
  obj.pose.orientation.w = 1.;
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

  // Set obj operation: ADD=0, REMOVE=1, APPEND=2, MOVE=3
  obj.operation = operation;

  // THIS IS THE WAY TO HANDLE OBSTACLES. Push back in the vector if you want to add, 
  // remove from the vector if you want to remove. Be sure to also process and apply!
  planner_->getCollisionObjects().push_back(obj);                       // add the obj object as obstacle
  planner_->getPlanningScenePtr()->processCollisionObjectMsg(obj);      // map the collision object into the joint space
  planner_->getPlanningSceneInterface().applyCollisionObjects(planner_->getCollisionObjects());
}

// Callback function for goals in the 3D cartesian space for the robot TCP
void ManipulatorPlanner::addCollObjCallback(const moveit_msgs::CollisionObject& obj)
{
  std::vector<double> dim_array = {obj.primitives[0].dimensions[0],      obj.primitives[0].dimensions[1],      obj.primitives[0].dimensions[2]  };
  double pos_array[]            = {obj.primitive_poses[0].position.x,    obj.primitive_poses[0].position.y,    obj.primitive_poses[0].position.z};
  double rot_array[]            = {obj.primitive_poses[0].orientation.x, obj.primitive_poses[0].orientation.y, obj.primitive_poses[0].orientation.z, obj.primitive_poses[0].orientation.w};
  uint operation                = obj.operation;
  createObj(obj.id,obj.primitives[0].type,dim_array,pos_array,rot_array,operation);
}

// --------------------- JACOBIAN-FKINE-INVKINE FUNCTIONS -------------------- //

// Get the tcp pose through FKine of a given joint pose
const geometry_msgs::Pose ManipulatorPlanner::get_manip_FKine()
{
  // Get current tcp pose through FKine
  return planner_->get_currentFKine(ee_name_);
}

// Get manipulator Jacobian
const Eigen::MatrixXd ManipulatorPlanner::get_manip_Jacobian()
{
  return planner_->getJacobian();
}

// Get manipulator inverse Jacobian
const Eigen::MatrixXd ManipulatorPlanner::get_manip_InvJacobian()
{
  return planner_->pseudoInverse(planner_->getJacobian());
}

// --------------------- MOVE CALLBACK FUNCTIONS --------------------- //

// Callback function to handle a tcp 3D goal
void ManipulatorPlanner::tcpGoalCallback(const geometry_msgs::Pose::ConstPtr& p)
{
  // V1: tcp goal
  // Declaration of the goal variabl as PS
  geometry_msgs::PoseStamped goal;

  // Fill the fields of the goal variable
  goal.header.frame_id = base_name_;
  // Set the pose as passed from the publisher
  goal.pose            = *p;                              

  // Check if the quaternion has unit norm, if not return an error
  tf2::Quaternion quat_tf;
  tf2::convert(goal.pose.orientation, quat_tf);
  if (quat_tf.length() >= 1.1 || quat_tf.length() <= 0.9)
  {
    ROS_ERROR("Quaternion must have unit norm.");
    return;
    ROS_ERROR("Quaternion must have unit norm.");
    return;
  }
  // If the norm is not so far from the unit, normalize the orientation quaternion
  quat_tf.normalize();
  goal.pose.orientation = tf2::toMsg(quat_tf);

  // Send the goal to the dynamic planner V6 
  planner_->plan(goal, ee_name_);
}

// Callback function to handle a joint goal
void ManipulatorPlanner::jointsGoalCallback(const sensor_msgs::JointState::ConstPtr& js)
{
  // Verify if the joint name vector size is the same as the joint goal passed from the publisher
  if (js->position.size() != joint_names_.size())
  {
    ROS_ERROR("Joint goal size is not the same as joint names.");
    return;
  }

  // Send the goal to the dynamic planner V1
  planner_->plan(js->position);
}

// Callback function to handle a tcp 3D goal with the inverse kinematics
void ManipulatorPlanner::tcpGoalIKCallback(const geometry_msgs::Pose::ConstPtr& p)
{
  // Make the inverse kinematics
  std::vector<double> joint_values = planner_->invKine(*p);

  if (joint_values.size() < joint_names_.size())
  {
    ROS_WARN("InvKine failed: goal not sent to MoveIt!");
    return;
  }

  sensor_msgs::JointState js;
  js.name = joint_names_;
  for (unsigned int k = 0; k < 6; k++) {js.position.push_back(joint_values[k]);}

  // Send to joint goal dynamic planner V1
  planner_->plan(js.position);
}

// Callback function to handle a joint goal without planner
void ManipulatorPlanner::jointsGoal_NoPlanner_Callback(const sensor_msgs::JointState::ConstPtr& js)
{
  // Verify if the joint name vector size is the same as the joint goal passed from the publisher
  if (js->position.size() != joint_names_.size())
  {
    ROS_ERROR("Joint goal size is not the same as joint names.");
    return;
  }

  // Fill joint names
  sensor_msgs::JointState js_new;
  js_new.name     = joint_names_;
  js_new.position = js->position;

  // Send the goal to the dynamic planner V1
  if      ( inst_kine_) {planner_->moveRobot(js_new);}
  else if (!inst_kine_) {  motors_controller(js_new);}
}

// Callback function to handle a tcp 3D goal with the inverse kinematics
void ManipulatorPlanner::tcpGoalIK_NoPlanner_Callback(const geometry_msgs::Pose::ConstPtr& p)
{
  // Make the inverse kinematics
  std::vector<double> joint_values = planner_->invKine(*p);

  if (joint_values.size() < joint_names_.size())
  {
    ROS_WARN("InvKine failed: goal not sent to MoveIt!");
    return;
  }

  sensor_msgs::JointState js;
  js.name = joint_names_;
  for (unsigned int k = 0; k < 6; k++) {js.position.push_back(joint_values[k]);}

  // Send the joint goal to the fake move group controller
  if      ( inst_kine_) {planner_->moveRobot(js);}
  else if (!inst_kine_) {  motors_controller(js);}
}

// Callback function for goals in the 3D cartesian space for the robot TCP
// Joint positions are computed through InvKine of inputs
// DYNAMIC PLANNER V9 and move along a carthesian direction
void ManipulatorPlanner::cartesianMoveCallback(const geometry_msgs::PoseArray::ConstPtr& p_seq)
{
  geometry_msgs::PoseArray ps = *p_seq;
  std::vector<geometry_msgs::Pose> waypoints;
  for (unsigned int k = 0; k < ps.poses.size(); k++)
  {
    waypoints.push_back(ps.poses[k]);
  }

  // Send to joint goal dynamic planner V4
  double fraction = planner_->cartesianPlan(waypoints);
  if (fraction < 0.01) {ROS_WARN("Cartesian trajectory unfeasible");}
}

// Set the the motors' position and speed through the controllers
void ManipulatorPlanner::motors_controller(const sensor_msgs::JointState js)
{
  std_msgs::Float64 msg;
  msg.data = js.position[0];
  j0_pub_.publish(msg);
  msg.data = js.position[1];
  j1_pub_.publish(msg);
  msg.data = js.position[2];
  j2_pub_.publish(msg);
  msg.data = js.position[3];
  j3_pub_.publish(msg);
  msg.data = js.position[4];
  j4_pub_.publish(msg);
  msg.data = js.position[5];
  j5_pub_.publish(msg);
}

// Set the instantaneous inverse Kinematics
void ManipulatorPlanner::instantKineSetterCallback(const std_msgs::Bool::ConstPtr& msg)
{
  inst_kine_ = msg->data;
}

// TODO: the following two functions give an allocator error on the compiler
// because ROS doesn't accept vector as msgs
/*
  // Callback function for goals in the 3D cartesian space for the robot TCP
  // DYNAMIC PLANNER V9
  void ManipulatorPlanner::tcpGoalSeqCallback(const std::vector<geometry_msgs::Pose>& p_seq)
  { 
    // Create a vector of PoseStamped msgs
    std::vector<const geometry_msgs::PoseStamped> p_stamp_seq;
    
    // Convert each pose of the input vector into PoseStamped goals
    for (const auto& p : p_stamp_seq)
    {   
      // Declaration of the goal variable as PS
      geometry_msgs::PoseStamped goal;

      // Fill the fields of the goal variable
      goal.header.frame_id = base_name_;
      // Set the pose as passed from the publisher
      goal.pose            = *p;                              

      // Check if the quaternion has unit norm, if not return an error
      tf2::Quaternion quat_tf;
      tf2::convert(goal.pose.orientation, quat_tf);
      if (quat_tf.length() >= 1.1 || quat_tf.length() <= 0.9)
      {
        ROS_ERROR("Quaternion must have unit norm.");
        return;
      }
      // If the norm is not so far from the unit, normalize the orientation quaternion
      quat_tf.normalize();
      goal.pose.orientation = tf2::toMsg(quat_tf);

      // Push back the goal
      p_stamp_seq.push_back(goal);
    }

    // Call the sequencial planner
    planner_->plan(p_seq, ee_name_); 
  }

  // Callback function for goals in the joint space
  // DYNAMIC PLANNER V5
  void ManipulatorPlanner::jointsGoalSeqCallback(const std::vector<std::vector<double>>& js_seq)
  {
    // Call the sequential planner
    planner_->plan(js_seq);
  }
*/