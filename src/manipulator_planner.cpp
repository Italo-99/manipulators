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

// CLASS SOURCE IMLEMENTATION OF MANIPULATOR PLANNER -> CHILD OF DYNAMIC PLANNER 

// The following code introduces the class of manipulator planner, an instance of the
// dynamic planner, plus a table object in RViz to test planning

// IMPORT LIBRARIES
#include "manipulators/ManipulatorPlanner.h"

// --------------------- PUBLIC CONSTRUCTOR ---------------------

ManipulatorPlanner::ManipulatorPlanner()
{
  // ---------------------  TCP AND JOINT GOALS SUBSCRIBERS ---------------------

  tcp_goal_sub_   = nh_.subscribe("/desired_tcp_pose",   1, &ManipulatorPlanner::tcpGoalCallback,    this);
  joint_goal_sub_ = nh_.subscribe("/desired_joint_pose", 1, &ManipulatorPlanner::jointsGoalCallback, this);
  tcp_goalIK_sub_ = nh_.subscribe("/desired_tcpIK_pose", 1, &ManipulatorPlanner::tcpGoalIKCallback,  this);

  // tcp_goalSeq_sub_   = nh_.subscribe("/desired_tcpSeq_poses",   1, &ManipulatorPlanner::tcpGoalSeqCallback,    this);
  // tcpIK_goalSeq_sub_ = nh_.subscribe("/desired_tcpIKSeq_poses", 1, &ManipulatorPlanner::tcpIKGoalSeqCallback,  this);
  // joint_goalSeq_sub_ = nh_.subscribe("/desired_jointSeq_poses", 1, &ManipulatorPlanner::jointsGoalSeqCallback, this);

  carthesian_move_sub_ = nh_.subscribe("/desired_cartesian_move", 1, &ManipulatorPlanner::cartesianMoveCallback, this);

  // ---------------------  ADD COLLISION OBJECT SUBSCRIBER  ---------------------

  add_coll_obj_sub_ = nh_.subscribe("/add_collision_object", 1, &ManipulatorPlanner::addCollObjCallback, this);

  // ---------------------  PRIVATE VARIABLES SETUP  ---------------------------

  check_param();

  // Call to the dynamic planner constructor
  planner_ = new DynamicPlanner(manipulator_name_, joint_names_, vel_factor_, acc_factor_, false);  

  // Set the sim mode for the dynamic planner
  planner_->setSimMode(sim_);

  // --------------------- ENVIRONMENT SETUP ---------------------

  // // Add a table to the scene
  // std::vector<double> dim_obj = {4.,4.,0.079};
  // double pos_obj[]            = {0.,0.,-0.04};
  // double rot_obj[]            = {0.,0.,0.,1.};
  // createObj("table", 1, dim_obj, pos_obj,rot_obj,0);

  // std::vector<double> dim_obj = {0.05,0.7,0.25};
  // double pos_obj[]            = {-0.15,0.,0.};
  // double rot_obj[]            = {0.,0.,0.,1.};
  // createObj("table", 1, dim_obj, pos_obj,rot_obj,0);
  
}

// Destructor of the object manipulator planner's class
ManipulatorPlanner::~ManipulatorPlanner() {delete planner_;}

// ---------------------  PUBLIC FUNCTIONS --------------------- //

// Manipulator planner spin function -> NOTE: the sleep rate is set in the node
void ManipulatorPlanner::spinner()  {planner_->spinner();} 

// ---------------------  PRIVATE FUNCTIONS --------------------- //

// --------------------- UTILS FUNCTIONS -------------------- //

// Check manipulators parameters passed to the node
void ManipulatorPlanner::check_param()
{
  // If one of the following parameters has not been defined, shutdwon ROS
  // The names of the params are passed with the prefix of the name of the node

  // Check for manipulator name parameter
  if (!nh_.getParam("manipulator_planner/manipulator_name", manipulator_name_))
  {
    ROS_ERROR("Manipulator name not defined");
    ros::shutdown();
    return;
  }

  // Check for joint names parameter
  if (!nh_.getParam("manipulator_planner/joint_names", joint_names_))
  {
    ROS_ERROR("Joint names not defined");
    ros::shutdown();
    return;
  }

  // Check for ee name parameter
  if (!nh_.getParam("manipulator_planner/ee_name", ee_name_))
  {
    ROS_ERROR("End-effector name not defined");
    ros::shutdown();
    return;
  }

  // Check for robot base name parameter
  if (!nh_.getParam("manipulator_planner/base_name", base_name_))
  {
    ROS_ERROR("Base name not defined");
    ros::shutdown();
    return;
  }

  // The following parameters can also be unset, but it's better to show a warning to the console to make the user aware
  // Check for velocity factor parameter
  if (!nh_.getParam("manipulator_planner/vel_factor", vel_factor_))
  {
    ROS_WARN("Velocity factor not defined, assuming 0.5");
    // If the parameter has not been set by the user, it is set here
    vel_factor_ = 0.5;
  }

  // Check for acceleration factor parameter
  if (!nh_.getParam("manipulator_planner/acc_factor", acc_factor_))
  {
    ROS_WARN("Acceleration factor not defined, assuming 0.5");

    // If the parameter has not been set by the user, it is set here
    acc_factor_ = 0.5;
  }

  // Get simulation status from the user (simulation or debug)
  nh_.getParam("manipulator_planner/sim", sim_);
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

// --------------------- JACOBIAN-FKINE FUNCTIONS -------------------- //

// THE FOLLOWING FUNCTION CANNOT BE USED SINCE DYNAMIC PLANNER METHODS CALLED DOENS'T WORK
// Get the tcp pose through FKINE of a given joint pose
const geometry_msgs::PoseStamped ManipulatorPlanner::get_manip_FKine()
{
  // Get current tcp pose through FKINE
  return planner_->get_currentFKine();
}

// Get manipulator Jacobian
const Eigen::MatrixXd ManipulatorPlanner::get_manip_Jacobian()
{
  // Get robot jacobian
  return planner_->getJacobian();
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
  }
  // If the norm is not so far from the unit, normalize the orientation quaternion
  quat_tf.normalize();
  goal.pose.orientation = tf2::toMsg(quat_tf);

  // Send the goal to the dynamic planner V6 
  planner_->plan(goal, ee_name_);
}

// Callback function to handle a tcp 3D goal with the inverse kinematics
void ManipulatorPlanner::tcpGoalIKCallback(const geometry_msgs::Pose::ConstPtr& p)
{
  // Fill the pose stamped goal
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = base_name_;
  goal.pose            = *p;

  // Make the inverse kinematics
  std::vector<double> joint_values = planner_->invKine(goal,ee_name_);
  // std::vector<double> joint_values = planner_->invKine(goal,"link_6");
  sensor_msgs::JointState js;
  for (unsigned int k = 0; k < 6; k++) {js.position.push_back(joint_values[k]);}

  // Send to joint goal dynamic planner V1
  planner_->plan(js.position);
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
  double fraction = planner_->cartesianPlan(waypoints,0.02);
  if (fraction < 0.01) {fraction = planner_->cartesianPlan(waypoints,0.05);}
  if (fraction < 0.01) {fraction = planner_->cartesianPlan(waypoints,0.10);}
  if (fraction < 0.01) {ROS_WARN("Cartesian trajectory unfeasible");}
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