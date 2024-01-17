#ifndef MANIPULATOR_PLANNER_H
#define MANIPULATOR_PLANNER_H

// IMPORT LIBRARIES
#include "dynamic_planner/dynamic_planner.h"
#include "ros/ros.h"

// CLASS OF MANIPULATOR PLANNER, THAT USES FUNCTIONS OF THE DYNAMIC PLANNER
class ManipulatorPlanner
{

  // Public functions declarations
 public:
  ManipulatorPlanner();     // Constructor
  ~ManipulatorPlanner();    // Destructor
  void spinner(void);       // Asynchronous spinner for ROS routines

// Private functions declarations
 private:

  // Callback function for goals in the 3D cartesian space for the TCP
  void tcpGoalCallback(const geometry_msgs::Pose::ConstPtr& p);

  // Callback function for goals in the joint space
  void jointsGoalCallback(const sensor_msgs::JointState::ConstPtr& js);

  // Private class variables declaration
  ros::NodeHandle nh_;                    // Node object
  ros::Subscriber tcp_goal_sub_;          // Subscriber to TCP goal
  ros::Subscriber joint_goal_sub_;        // Publisher to joint goal

  std::vector<std::string> joint_names_;  // Joints' names
  std::string ee_name_;                   // End-effector's name
  std::string base_name_;                 // Robot base's name
  DynamicPlanner* planner_;               // Dynamic planner object 
};

#endif /* MANIPULATOR_PLANNER_H */
