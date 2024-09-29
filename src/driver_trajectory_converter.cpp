#include "ur10_trajectory_converter/UR10TrajectoryConverter.h"

UR10TrajectoryConverter::UR10TrajectoryConverter()
{
  trajectory_sub_ =
    nh_.subscribe("/trajectory", 2, &UR10TrajectoryConverter::trajectoryCallback, this);
  joint_sub_ =
    nh_.subscribe("/joint_states", 2, &UR10TrajectoryConverter::jointCallback, this);
  trajectory_action_publisher_ =
    nh_.advertise<control_msgs::FollowJointTrajectoryActionGoal>(
      "/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal", 1);

  client_ = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(
    "/scaled_pos_joint_traj_controller/follow_joint_trajectory", true);


  trajectory_execution_publisher_ =
    nh_.advertise<std_msgs::Bool>("/trajectory_execution", 1);

  velocity_publisher_ = nh_.advertise<std_msgs::Float64MultiArray>(
    "/ur_rtde/controllers/joint_velocity_controller/command", 1);
  toll_ = 5e-4;
  if (!nh_.getParam("/ur10_trajectory_converter/control_type", control_type_))
  {
    control_type_ = "velocity";
  }

  vel_traj_received_ = false;
  dq_cmd_.data.resize(6);
}

void UR10TrajectoryConverter::jointCallback(const sensor_msgs::JointState::ConstPtr& j)
{
  checkEnd(traj_, *j);
}

void UR10TrajectoryConverter::trajectoryCallback(
  const trajectory_msgs::JointTrajectory::ConstPtr& t)
{
  std::cout << "Trajectory received" << std::endl;
  for (const std::string& name : t->joint_names)
    joints_map_[name] = 0;
  traj_ = *t;
  if (control_type_ == "position")
  {
    std::string decision;
    if (!nh_.getParam("/ur10_trajectory_converter/decision", decision))
    {
      decision = "no_client";
    }

    if (decision == "no_client")
    {
      control_msgs::FollowJointTrajectoryActionGoal trajectory_action;
      trajectory_action.goal.trajectory = traj_;
      trajectory_action_publisher_.publish(trajectory_action);
    }
    else
    {
      control_msgs::FollowJointTrajectoryGoal trajectory_goal;
      trajectory_goal.trajectory = traj_;
      client_->waitForServer();
      client_->sendGoal(trajectory_goal);
    }
  }
  else
  {
    vel_traj_received_ = true;
    iteration_         = 0;
  }
}

void UR10TrajectoryConverter::checkEnd(trajectory_msgs::JointTrajectory& t,
                                       sensor_msgs::JointState j)
{
  if (t.points.size() > 0)
  {
    joints_values_.resize(traj_.joint_names.size());
    static std::unordered_map<std::string, double>::iterator it;
    uint joint_counter = 0;
    for (uint i = 0; i < t.joint_names.size(); i++)
    {

      it = joints_map_.find(j.name[i]);
      if (it != joints_map_.end())
      {
        it->second = j.position[i];
        joint_counter++;
        if (joint_counter == t.joint_names.size())
        {
          for (uint k = 0; k < t.joint_names.size(); k++)
            joints_values_[k] = joints_map_[t.joint_names[k]];
        }
        continue;
      }
    }

    double max = 0;
    double tmp_max;

    if (iteration_ >= int(t.points.size()) - 1)
    {
      for (int i = 0; i < joints_values_.size(); i++)
      {
        // tmp_max = abs(joints_values_[i] - t.points.back().positions[i]);
        // if (tmp_max > max)
        // {
        //   max = tmp_max;
        // }
        max = std::max(max, abs(joints_values_[i] - t.points.back().positions[i]));
      }

      if (max < toll_)
      {
        vel_traj_received_ = false;
        std_msgs::Bool ex;
        ex.data = true;
        trajectory_execution_publisher_.publish(ex);
        traj_.points.clear();
      }
    }
  }
}

void UR10TrajectoryConverter::computeVel()
{
  if (vel_traj_received_ && joints_values_.size() > 0)
  {
    iteration_ = std::min(iteration_, int(traj_.points.size()) - 1);

    for (uint i = 0; i < 6; i++)
      dq_cmd_.data[i] = traj_.points[iteration_].velocities[i] +
                        4.0 * (traj_.points[iteration_].positions[i] - joints_values_[i]);
    iteration_++;
  }
  else
  {
    dq_cmd_.data = std::vector<double>(6, 0.0);
  }
  velocity_publisher_.publish(dq_cmd_);
}

void UR10TrajectoryConverter::spinner(const double rate)
{  
  ros::Rate r(rate);

  while (ros::ok())
  {
    ros::spinOnce();
    if (control_type_ == "velocity")
      computeVel();
    r.sleep();
  }

  // Publish null vel data when shutdown
  dq_cmd_.data = std::vector<double>(6, 0.0);
  velocity_publisher_.publish(dq_cmd_);
}
