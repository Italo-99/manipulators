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
#include "manipulator_planner/ManipulatorPlanner.h"

// TODO: together with node launch, UR10 should go immediatly to a preconfigured configuration
// initial position {0,-90,+90,-90,-90,0}


// ---------------------  PUBLIC CONSTRUCTOR ---------------------

ManipulatorPlanner::ManipulatorPlanner()
{
  // ---------------------  TCP AND JOINT GOALS SUBSCRIBERS ---------------------

  tcp_goal_sub_   = nh_.subscribe("/desired_tcp_pose",   1, &ManipulatorPlanner::tcpGoalCallback,    this);
  joint_goal_sub_ = nh_.subscribe("/desired_joint_pose", 1, &ManipulatorPlanner::jointsGoalCallback, this);

  tcp_goal_sub_   = nh_.subscribe("/desired_tcpSeq_poses",   1, &ManipulatorPlanner::tcpGoalSeqCallback,    this);
  joint_goal_sub_ = nh_.subscribe("/desired_jointSeq_poses", 1, &ManipulatorPlanner::jointsGoalSeqCallback, this);


  // Planner args
  std::string manipulator_name;     // Manipulator name
  double vel_factor, acc_factor;    // Scale factor for joint velocities and accelerations
  bool sim;                         // Simulation status (true for sim, false for debug)

  // CALL TO THE DYNAMIC PLANNER
  planner_ = new DynamicPlanner(manipulator_name, joint_names_, vel_factor, acc_factor, false);

  // Set the sim mode for the dynamic planner
  planner_->setSimMode(sim);

  // Add a table to the scene
  // moveit_msgs::CollisionObject table = createObj("table", 1, [4,4,0.079], [0,0,-0.04]);  

  // Update collision objects scene
  planner_->getPlanningSceneInterface().applyCollisionObjects(planner_->getCollisionObjects());
}

// Destructor of the object manipulator planner's class
ManipulatorPlanner::~ManipulatorPlanner() {delete planner_;}

// ---------------------  PUBLIC FUNCTIONS ---------------------

// Manipulator planner spin function -> NOTE: the sleep rate is set in the node
void ManipulatorPlanner::spinner()  {ros::spinOnce();}

// // Creation of a collision object
// moveit_msgs::CollisionObject ManipulatorPlanner::createObj(std::string& name, 
//                                                            int          obj_type, 
//                                                            float        obj_dims, 
//                                                            float        obj_pos = [0,0,0], 
//                                                            bool         rot_90  = false)
// {
//   // Creation of the obj
//   moveit_msgs::CollisionObject obj;

//   obj.header.frame_id = base_name_;
//   obj.id              = name;
//   obj.primitives[0].type = obj_type;
//   obj.operation = 0;  // static obj
//   obj.primitives[0].dimensions.resize(int(size(obj_dims)));

//   // Set primitive type
//   switch(obj_type)
//   {
//     case 1:   // BOX: Rectangular shape setting
//       if (size(obj_dims) != 3) {ROS_WARN_THROTTLE(3, "obj_dims array is not compatible with obj_type");}
//       else                     {obj.primitives.resize(1);
//                                 // Set the three dimensions of the parallelepiped
//                                 obj.primitives[0].dimensions[0] = obj_dims[0];
//                                 obj.primitives[0].dimensions[1] = obj_dims[1];
//                                 obj.primitives[0].dimensions[2] = obj_dims[2];}
//       break;

//     case 2:   // SPHERE
//       if (size(obj_dims) != 1) {ROS_WARN_THROTTLE(3, "obj_dims array is not compatible with obj_type");}
//       else                     {obj.primitives.resize(1);
//                                 // Set the sphere radius
//                                 obj.primitives[0].dimensions[0] = obj_dims[0];}
//       break;

//     default:   // CYLINDER OR CONE
//       if (size(obj_dims) != 2) {ROS_WARN_THROTTLE(3, "obj_dims array is not compatible with obj_type");}
//       else                     {obj.primitives.resize(1);
//                                 // Set height and radius of the cylinder/cone
//                                 obj.primitives[0].dimensions[0] = obj_dims[0];
//                                 obj.primitives[0].dimensions[1] = obj_dims[1];}
//       break;
//   }

//   // Set obj position
//   obj.primitive_poses.resize(1);
//   obj.primitive_poses[0].position.x = obj_pos[0];
//   obj.primitive_poses[0].position.y = obj_pos[1];
//   obj.primitive_poses[0].position.z = obj_pos[2];

//   // Set obj orientation
//   if (rot90)
//   {
//     obj.primitive_poses[0].orientation.x = 0;
//     obj.primitive_poses[0].orientation.y = 0;
//     obj.primitive_poses[0].orientation.z = PI/4;
//     obj.primitive_poses[0].orientation.w = PI/4;
//   }
//   else
//   {
//     obj.primitive_poses[0].orientation.x = 0;
//     obj.primitive_poses[0].orientation.y = 0;
//     obj.primitive_poses[0].orientation.z = 0;
//     obj.primitive_poses[0].orientation.w = 1;
//   }


//   // THIS IS THE WAY TO HANDLE OBSTACLES. Push back in the vector if you want to add, 
//   // remove from the vector if you want to remove. Be sure to also process and apply!
//   planner_->getCollisionObjects().push_back(obj);                       // add the obj object as obstacle
//   planner_->getPlanningScenePtr()->processCollisionObjectMsg(obj);      // map the collision object into the joint space

//   return obj;
// }

// // Check manipulators parameters passed to the node
void ManipulatorPlanner::check_param(std::string manipulator_name,
                                     double      vel_factor, 
                                     double      acc_factor,
                                     bool        sim)
{
  // If one of the following parameters has not been defined, shutdwon ROS

  // Check for manipulator name parameter
  if (!nh_.getParam("manipulator_planner/manipulator_name", manipulator_name))
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
  if (!nh_.getParam("manipulator_planner/vel_factor", vel_factor))
  {
    ROS_WARN("Velocity factor not defined, assuming 0.5");
    // If the parameter has not been set by the user, it is set here
    vel_factor = 0.5;
  }

  // Check for acceleration factor parameter
  if (!nh_.getParam("manipulator_planner/acc_factor", acc_factor))
  {
    ROS_WARN("Acceleration factor not defined, assuming 0.5");

    // If the parameter has not been set by the user, it is set here
    acc_factor = 0.5;
  }

  // Get simulation status from the user (simulation or debug)
  nh_.getParam("manipulator_planner/sim", sim);
}

// Callback function to handle a tcp 3D goal
void ManipulatorPlanner::tcpGoalCallback(const geometry_msgs::Pose::ConstPtr& p)
{

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

  // Send the goal to the planner
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

  // Send the goal to the planner -> FIND THIS FUNCTION IN THE DYNAMIC PLANNER CLASS TO UNDERSTAND !!!!!!!!!!!!
  planner_->plan(js->position);
}

// // Callback function for goals in the 3D cartesian space for the robot TCP
void tcpGoalSeqCallback(const std::vector<geometry_msgs::Pose>& p_seq)
{ 
  // // Create a vector of PoseStamped msgs
  // std::vector<const geometry_msgs::PoseStamped> p_stamp_seq;
  
  // // Convert each pose of the input vector into PoseStamped goals
  // for (const auto& p : p_stamp_seq)
  // {   
  //   // Declaration of the goal variable as PS
  //   geometry_msgs::PoseStamped goal;

  //   // Fill the fields of the goal variable
  //   goal.header.frame_id = base_name_;
  //   // Set the pose as passed from the publisher
  //   goal.pose            = *p;                              

  //   // Check if the quaternion has unit norm, if not return an error
  //   tf2::Quaternion quat_tf;
  //   tf2::convert(goal.pose.orientation, quat_tf);
  //   if (quat_tf.length() >= 1.1 || quat_tf.length() <= 0.9)
  //   {
  //     ROS_ERROR("Quaternion must have unit norm.");
  //     return;
  //   }
  //   // If the norm is not so far from the unit, normalize the orientation quaternion
  //   quat_tf.normalize();
  //   goal.pose.orientation = tf2::toMsg(quat_tf);

  //   // Push back the goal
  //   p_stamp_seq.push_back(goal);
  // }

  // // Call the sequencial planner
  // plan(p_seq, ee_name_); 
}

// // Callback function for goals in the joint space
void jointsGoalSeqCallback(const std::vector<sensor_msgs::JointState>& js_seq)
{
  // // Create a vector of vectors
  // std::vector<std::vector<int>> js_seq_vecs;

  // // Convert each joint state msg into a vector
  // for (const auto& js : js_seq)
  // {
  //   js_seq_vecs.push_back(js.position);
  // }

  // // Call the sequential planner
  // plan(js_seq_vecs);
}