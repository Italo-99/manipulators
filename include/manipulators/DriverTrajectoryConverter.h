#ifndef SPEED_LIMITATION_H
#define SPEED_LIMITATION_H

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <unordered_map>


class UR10TrajectoryConverter
{

 public:
  UR10TrajectoryConverter();
  void spinner(const double);

 private:
  void trajectoryCallback(const trajectory_msgs::JointTrajectory::ConstPtr& t);
  void jointCallback(const sensor_msgs::JointState::ConstPtr& j);
  void checkEnd(trajectory_msgs::JointTrajectory& t, sensor_msgs::JointState j);
  void computeVel();
  ros::NodeHandle nh_;
  ros::Subscriber trajectory_sub_;
  ros::Subscriber joint_sub_;
  ros::Publisher trajectory_action_publisher_;
  ros::Publisher trajectory_execution_publisher_;
  ros::Publisher velocity_publisher_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* client_;
  std::vector<double> joints_values_;
  std::unordered_map<std::string, double> joints_map_;
  double toll_;
  trajectory_msgs::JointTrajectory traj_;
  std::string control_type_;
  bool vel_traj_received_;
  int iteration_ = 0;
  std_msgs::Float64MultiArray dq_cmd_;
};

#endif // SPEED_LIMITATION_H
