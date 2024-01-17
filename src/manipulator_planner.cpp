// THE FOLLOWING CODE INTRODUCES THE CLASS OF MANIPULATOR PLANNER
// WHICH INCLUDES AN INSTANCE OF DYNAMIC PLANNER
// AND A "TABLE" OBJECT IN RVIZ TO TEST PLANNING

// IMPORT LIBRARIES
#include "manipulator_planner/ManipulatorPlanner.h"

// TODO: together with node launch, UR10 should go immediatly to a preconfigured configuration
// initial position {0,-90,+90,-90,-90,0}

// MANIPULATOR PLANNER CLASS
ManipulatorPlanner::ManipulatorPlanner()
{
  // TCP goal subscriber
  tcp_goal_sub_ =
    nh_.subscribe("/desired_tcp_pose", 1, &ManipulatorPlanner::tcpGoalCallback, this);
  
  // Joint goal subscriber
  joint_goal_sub_ = nh_.subscribe("/desired_joint_pose", 1,
                                  &ManipulatorPlanner::jointsGoalCallback, this);

  // Class variables declarations .> SHOULD THEY BE PUT INTO THE HEADER FILE??????
  std::string manipulator_name;     // Manipulator name
  double vel_factor, acc_factor;    // Scale factor for joint velocities and accelerations
  bool sim;                         // Simulation status (true for sim, false for debug)

  // This variable checks if something is missing among simulation parameters
  // BUT WHAT IF THIS MANIPULATOR IS LAUNCHED FROM A NODE???
  bool params = true;

  // Check for manipulator name parameter
  if (!nh_.getParam("manipulator_planner/manipulator_name", manipulator_name))
  {
    ROS_ERROR("Manipulator name not defined");
    params = false;
  }

  // Check for joint names parameter
  if (!nh_.getParam("manipulator_planner/joint_names", joint_names_))
  {
    ROS_ERROR("Joint names not defined");
    params = false;
  }

  // Check for ee name parameter
  if (!nh_.getParam("manipulator_planner/ee_name", ee_name_))
  {
    ROS_ERROR("End-effector name not defined");
    params = false;
  }

  // Check for robot base name parameter
  if (!nh_.getParam("manipulator_planner/base_name", base_name_))
  {
    ROS_ERROR("Base name not defined");
    params = false;
  }

  // If one of the parameters above has not been set, shutdown the roscore
  if (!params)
  {
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

  // Get simulation goal status from the user (simulation or debug)
  nh_.getParam("manipulator_planner/sim", sim);


  // CREATION OF A TABLE AS COLLISION OBJECT
  moveit_msgs::CollisionObject table;

  // Set the position of the table relating to the robot base frame
  table.header.frame_id = base_name_;
  table.id              = "table";

  // Rectangular shape setting
  table.primitives.resize(1);
  table.primitives[0].type = 1;

  // Set the three dimensions of the parallelepaid
  table.primitives[0].dimensions.resize(3);
  table.primitives[0].dimensions[0] = 4;
  table.primitives[0].dimensions[1] = 4;
  table.primitives[0].dimensions[2] = 0.079;

  // Set table position
  table.primitive_poses.resize(1);
  table.primitive_poses[0].position.x = 0;
  table.primitive_poses[0].position.y = 0;
  table.primitive_poses[0].position.z = -0.04;

  // Set table orientation
  table.primitive_poses[0].orientation.x = 0;
  table.primitive_poses[0].orientation.y = 0;
  table.primitive_poses[0].orientation.z = 0;
  table.primitive_poses[0].orientation.w = 1;

  // Set the table static
  table.operation = 0;

  // CALL TO THE DYNAMIC PLANNER
  planner_ = new DynamicPlanner(manipulator_name, joint_names_, vel_factor, acc_factor);

  // Set the sim mode for the dynamic planner
  planner_->setSimMode(sim);

  // THIS IS THE WAY TO HANDLE OBSTACLES. Push back in the vector if you want to add,
  // remove from the vector if you want to remove. You must add a callback that reads data
  // from optitrack and push back or modify the objects. Be sure to also process and
  // apply!

  planner_->getCollisionObjects().push_back(table);                       // add the table object as obstacle
  planner_->getPlanningScenePtr()->processCollisionObjectMsg(table);      // map the collision object into the joint space
  planner_->getPlanningSceneInterface().applyCollisionObjects(            // apply the collision object to the planning scene interface
            planner_->getCollisionObjects());
}

// Destructor of the object manipulator planner's class
ManipulatorPlanner::~ManipulatorPlanner() { delete planner_; }

// CALLBACK FUNCTION TO HANDLE THE TCP GOAL
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

// CALLBACK FUNCTION TO HANDLE THE JOINT GOAL
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

// Manipulator planner spin function
void ManipulatorPlanner::spinner()
{
  // Simple ROS once spinner (NOTE: the sleep rate is set outside of this class implementation)
  ros::spinOnce();
  // The scaling MUST publish on the topic /trajectory_counter!!!
  planner_->checkTrajectory();  // UPDATE THE TRAJECTORY GOAL STATE OF THE DYNAMIC PLANNER
                                // FIND THIS FUNCTION IN THE DYNAMIC PLANNER CLASS TO UNDERSTAND !!!!!!!!!!!!
  return;
}
