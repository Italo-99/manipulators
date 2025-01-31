#include <manipulators/DynamicPlanner.h>

DynamicPlanner::DynamicPlanner(const rclcpp::Node::SharedPtr &node,
                               const std::string &planning_group,
                               const double vel_factor,
                               const double acc_factor,
                               bool dynamic_behavior)
    : node_(node),
      params_(),
      dynamic_behavior_(dynamic_behavior),
      planning_group_(planning_group),
      trajpoint_(0UL)
{
    RCLCPP_INFO(node_->get_logger(), "Initializing DynamicPlanner...");

    params_.vel_factor = vel_factor;
    params_.acc_factor = acc_factor;

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node_, 
        moveit::planning_interface::MoveGroupInterface::Options(
            planning_group,
            "robot_description",
            ""
        )
    );

    move_group_->startStateMonitor();

    //Fetch robot state
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_);
    robot_model_ = robot_model_loader_->getModel();

    joint_names_ = move_group_->getJointNames();

    RCLCPP_INFO(node_->get_logger(), "MoveGroupInterface initialized");

    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

    //Initialize visual tools
    namespace rvt = rviz_visual_tools;
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(node_,
                                                                             "base_link", 
                                                                             "/moveit_visual_markers", 
                                                                             move_group_->getRobotModel());

    //Initialize time optimal trajectory generation
    time_optimal_traj_gen = std::make_shared<trajectory_processing::TimeOptimalTrajectoryGeneration>(
        totg_tolerance,
        totg_resample_dt,
        totg_min_angle_change
    );

    updatePlannerParams();
    initialize();

    RCLCPP_INFO(node_->get_logger(), "DynamicPlanner initialized");
}

void DynamicPlanner::initialize()
{
    // Publishers
    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/move_group/fake_controller_joint_states", 1);
    trajectory_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(planning_group_ + "/trajectory", 1);
    trajectory_res_pub_ = node_->create_publisher<std_msgs::msg::Bool>(planning_group_ + "/traj_planning_result", 1);

    // Subscribers
    stop_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/move_group/stop_execution", 1, 
        std::bind(&DynamicPlanner::stop_callback, this, std::placeholders::_1)
    );

    joints_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 1,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->jointsState_callback(msg);
        }
    );

    trajpoint_sub_ = node_->create_subscription<std_msgs::msg::UInt32>(
        planning_group_ + "/trajpoint", 1,
        [this](const std_msgs::msg::UInt32::SharedPtr msg) {
            this->trajpoint_callback(msg);
        }
    );
}

// ------------------------------------- PUBLIC METHODS -------------------------------------


//PLAN: Joint space

void DynamicPlanner::plan(const std::vector<double> joint_positions)
{
    /*
    Plans and executes a trajectory to the joint positions for the manipulator (joint space)
    Args:
        joint_positions: Array of target joint positions
    */
    
    //Sets the target joint positions
    bool is_within_bounds = move_group_->setJointValueTarget(joint_positions);
    std_msgs::msg::Bool result_msg;

    if (checkJointDiff(joint_positions)){
        result_msg.data = false;
        trajectory_res_pub_->publish(result_msg);
        return;
    }

    if (!is_within_bounds) //Check if the joint positions are within the joint limits
    {
        RCLCPP_ERROR(node_->get_logger(), "Joint positions are out of bounds");
        result_msg.data = false;
        trajectory_res_pub_->publish(result_msg);
        return;
    }

    //Create the plan and execute
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode error = move_group_->plan(plan);
    if (error != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed with error code: %d", error.val);
        result_msg.data = false;
        trajectory_res_pub_->publish(result_msg);
        return;
    }
    moveit_msgs::msg::RobotTrajectory trajectory = plan.trajectory_;

    bool totg_success = processTrajectory(trajectory); //Apply time optimal trajectory generation

    if (!totg_success){
        result_msg.data = false;
        trajectory_res_pub_->publish(result_msg);
        return;
    }

    setTrajectory(trajectory);

    result_msg.data = true;
    trajectory_res_pub_->publish(result_msg);
}

//PLAN: Cartesian space

void DynamicPlanner::plan(const geometry_msgs::msg::Pose& goal_pose, const std::string& ee_link, const std::string& frame)
{
    /*
    Plan: pose goal
        Args:
            goal_pose: Target position
            ee_link: End effector link
            frame: Reference frame
            space: Planning space (joint or operative)
    */

    std_msgs::msg::Bool result_msg;

    //Sets the target pose
    move_group_->setPoseReferenceFrame(frame);
    move_group_->setPoseTarget(goal_pose, ee_link);

    //Create the plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode error = move_group_->plan(plan);
    if (error != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed with error code: %d", error.val);
        result_msg.data = false;
        trajectory_res_pub_->publish(result_msg);
        return;
    }

    moveit_msgs::msg::RobotTrajectory trajectory = plan.trajectory_;

    bool totg_success = processTrajectory(trajectory); //Apply time optimal trajectory generation

    if (!totg_success){
        result_msg.data = false;
        trajectory_res_pub_->publish(result_msg);
        return;
    }

    setTrajectory(trajectory);

    result_msg.data = true;
    trajectory_res_pub_->publish(result_msg);
}

void DynamicPlanner::plan(const geometry_msgs::msg::Pose& goal_pose, const std::string& ee_link)
{
    plan(goal_pose, ee_link, world_frame_);
}

void DynamicPlanner::plan(const geometry_msgs::msg::Pose& goal_pose)
{
    plan(goal_pose, end_effector_link_, world_frame_);
}

double DynamicPlanner::cartesianPlan(const std::vector<geometry_msgs::msg::Pose>& waypoints)
{
    // Setup cartesian planner
    double jump_treshold = 0.0;
    double eef_step = params_.max_velocity*params_.sample_time; // Ideal distance step
    double fraction = 0.0;
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    moveit_msgs::msg::RobotTrajectory trajectory;

    for (int k = 0; k < params_.num_attempts; k++)
    {
        fraction = move_group_->computeCartesianPath(
            waypoints, 
            (pow(10,k))*eef_step, 
            jump_treshold, 
            trajectory    
        );

        if (fraction > 0.0) {break;}
    }
    // Resample trajectory time
    for (unsigned int k = 0; k < trajectory.joint_trajectory.points.size(); k++)
    {
        trajectory.joint_trajectory.points[k].time_from_start = rclcpp::Duration::from_seconds((double)((k)*params_.sample_time));
    }  

    // Display results
    RCLCPP_INFO(node_->get_logger(), 
                "Computed cartesian path of %.2f%% fraction achieved, in time %.6f s", 
                fraction * 100.0,
                std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count());
  
    // Display and send trajectory
    //trajectoryVisualizer(trajectory_);
    // For the robot driver
    setTrajectory(trajectory);
    // For simulated robot
    if (fraction > 0.0) {moveRobot();}

    return fraction;
}

void DynamicPlanner::moveRobot(const sensor_msgs::msg::JointState& joint_state)
{
    //Executes a single joint state
    joint_state_pub_->publish(joint_state);
}

void DynamicPlanner::moveRobot(const trajectory_msgs::msg::JointTrajectoryPoint& traj_pt)
{
    //Executes a single trajectory position

    sensor_msgs::msg::JointState joint_state;
    // Fill the name of the joints
    joint_state.name = joint_names_;
    joint_state.position = traj_pt.positions;
    moveRobot(joint_state);
}

void DynamicPlanner::moveRobot()
{
    //Executes the last planned trajectory
    if (robot_trajectory_.joint_trajectory.points.empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "No trajectory to execute");
        return;
    }

    // Setup the rate of the planner execution
    // Hypothesis: all the points of the trajectory are uniformely sampled in time; if not, thery are forced here
    double points_time = robot_trajectory_.joint_trajectory.points[1].time_from_start.sec +             //seconds
                         robot_trajectory_.joint_trajectory.points[1].time_from_start.nanosec * 1e-9;
    
    is_moving = true;

    rclcpp::Rate traj_exec_rate(1/(points_time));

    for (auto traj_pt : robot_trajectory_.joint_trajectory.points)
    {
        if (is_moving == false){
                //Send a trajectory msg with the last position and velocities set to 0
                trajectory_msgs::msg::JointTrajectoryPoint stopping_point = traj_pt;
                stopping_point.velocities = std::vector<double>(joint_names_.size(), 0.);
                moveRobot(stopping_point);
                break;
        } else if (dynamic_behavior_){
            //Check if the trajectory is still clear of obstacles
            if (!checkTrajectory()){
                //If not recalculate
                RCLCPP_INFO(node_->get_logger(), "Recalculating trajectory");
                //recalculateTrajectory(traj_current_index_);
            }
        }

        // Execute the move
        moveRobot(traj_pt);

        traj_exec_rate.sleep();
    }

    RCLCPP_INFO(node_->get_logger(), "Trajectory executed.");

    is_moving = false;
}

void DynamicPlanner::moveRobot(moveit_msgs::msg::RobotTrajectory& robot_trajectory)
{
    setTrajectory(robot_trajectory);
    moveRobot();
}

bool DynamicPlanner::isMoving()
{
    //Check if the robot is moving
    return is_moving;
}

// Check if the planner has received group definition, so the dynamic planner can start working
bool DynamicPlanner::isReady() const
{
  // The following booleans are true when the three subs have read something from active pubs 
  return joints_group_received_;
}

void DynamicPlanner::stop()
{
    is_moving = false;
}

std::shared_ptr<moveit::planning_interface::MoveGroupInterface> DynamicPlanner::getMoveGroup() const
{
    return move_group_;
}

std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> DynamicPlanner::getPlanningScene() const
{
    return planning_scene_interface_;
}

std::shared_ptr<moveit_visual_tools::MoveItVisualTools> DynamicPlanner::getVisualTools() const
{
    return visual_tools_;
}

DynamicPlannerParams DynamicPlanner::getParams() const
{
    return params_;
}

void DynamicPlanner::setParams(const std::string &planner_id, const int attempts, const double time,
                               const double v_factor, const double a_factor)
{
    params_.planner_id = planner_id;
    params_.num_attempts = attempts;
    params_.planning_time = time;
    params_.vel_factor = v_factor;
    params_.acc_factor = a_factor;

    updatePlannerParams();
}

void DynamicPlanner::setParams(const DynamicPlannerParams &params)
{
    params_ = params;
}

void DynamicPlanner::setDynamicBehavior(bool dynamic_behavior)
{
    dynamic_behavior_ = dynamic_behavior;
}

bool DynamicPlanner::isDynamic() const
{
    return dynamic_behavior_;
}

void DynamicPlanner::setPlanningSpace(PlanningSpace space)
{
    if (isMoving())
    {
        RCLCPP_ERROR(node_->get_logger(), "Cannot change planning space while the robot is moving");
        return;
    }
    planning_space_ = space;
}

DynamicPlanner::PlanningSpace DynamicPlanner::getPlanningSpace() const
{
    return planning_space_;
}

void DynamicPlanner::setRobotState(moveit::core::RobotStatePtr &robot_state)
{
    move_group_->setStartState(*robot_state);
}

moveit::core::RobotStatePtr DynamicPlanner::getRobotState()
{
    return move_group_->getCurrentState();
}

std::vector<double> DynamicPlanner::getNamedTarget(const std::string &target_name)
{
    /*
    Returns the joint positions from a named target (either remembered or predefined in srdf)
    Args:
        target_name: Name of the target
    Returns:
        Array of joint positions
    */

    std::map<std::string, double> named_target = move_group_->getNamedTargetValues(target_name);
    std::vector<double> joint_states;

    std::vector<std::pair<std::string, double>> named_targets_pair;
    for (auto iterator = named_target.begin(); iterator != named_target.end(); iterator++)
    {
        joint_states.push_back(iterator->second);
    }

    return joint_states;
}

// ------------------------------------- PATH CONSTRAINTS ------------------------------------

void DynamicPlanner::setPathConstraints(const moveit_msgs::msg::Constraints &constraints)
{
    move_group_->setPathConstraints(constraints);
}

moveit_msgs::msg::Constraints DynamicPlanner::getPathConstraints() const
{
    return move_group_->getPathConstraints();
}

void DynamicPlanner::clearPathConstraints()
{
    move_group_->clearPathConstraints();
}

// ------------------------------------- FORWARD KINEMATICS ------------------------------------

geometry_msgs::msg::PoseStamped DynamicPlanner::getFKine(const std::vector<double> &joint_positions, const std::string &end_effector_link)
{
    /*
    Computes the forward kinematics for the given joint positions
    Args:
        joint_positions: Array of joint positions
    Returns:
        Pose of the end-effector
    */

    //Sets the joint positions
    moveit::core::RobotStatePtr robot_state = getRobotState();
    robot_state->setJointGroupPositions(move_group_->getName(), joint_positions);
    robot_state->update();

    const Eigen::Isometry3d &end_effector_pose = robot_state->getGlobalLinkTransform(end_effector_link);

    return toPoseStamped(end_effector_pose, move_group_->getPlanningFrame());
}

geometry_msgs::msg::PoseStamped DynamicPlanner::getFKine(const std::string &end_effector_link)
{
    return getFKine(
        move_group_->getCurrentJointValues(),
        end_effector_link
    );
}

geometry_msgs::msg::PoseStamped DynamicPlanner::getFKine()
{
    return getFKine(end_effector_link_);
}

// ------------------------------------- INVERSE KINEMATICS ------------------------------------

std::vector<double> DynamicPlanner::invKine(const geometry_msgs::msg::Pose &target_pose, const std::string &end_effector_link)
{
    moveit::core::RobotStatePtr kinematic_state = getRobotState();
    const moveit::core::JointModelGroup *joint_model_group = kinematic_state->getJointModelGroup(planning_group_);

    std::vector<double> joint_values;

    bool success = kinematic_state->setFromIK(
        joint_model_group,
        target_pose,
        end_effector_link,
        0.05 //timeout - TODO: Create a parameter for this
    );

    if (success){
        kinematic_state->copyJointGroupPositions(planning_group_, joint_values);
    } 
    else {
        RCLCPP_ERROR(node_->get_logger(), "Unable to perform inverse kinematics");
    }

    return joint_values;
}

std::vector<double> DynamicPlanner::invKine(const geometry_msgs::msg::Pose &target_pose)
{
    return invKine(target_pose, end_effector_link_);
}


// ------------------------------------- JACOBIAN -------------------------------------

const Eigen::MatrixXd DynamicPlanner::getJacobian(const std::string &end_effector_link)
{
    moveit::core::RobotStatePtr kinematic_state = getRobotState();

    Eigen::MatrixXd jacobian;
    Eigen::Vector3d reference_point {0.0, 0.0, 0.0};
    const moveit::core::LinkModel *link_model = kinematic_state->getLinkModel(end_effector_link);

    bool success = kinematic_state->getJacobian(
        kinematic_state->getJointModelGroup(planning_group_),
        link_model,
        reference_point,
        jacobian
    );

    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Unable to retrieve jacobian for link: %s", end_effector_link.c_str());
    }

    return jacobian;
}

const Eigen::MatrixXd DynamicPlanner::getJacobian()
{
    return getJacobian(end_effector_link_);
}

const Eigen::MatrixXd DynamicPlanner::getPseudoInverseJacobian(const std::string &end_effector_link)
{
    return getJacobian(end_effector_link).completeOrthogonalDecomposition().pseudoInverse();
}

const Eigen::MatrixXd DynamicPlanner::getPseudoInverseJacobian()
{
    return getPseudoInverseJacobian(end_effector_link_);
}

// -------------------------------------------------------------------------------------------
// ------------------------------------- PRIVATE METHODS -------------------------------------
// -------------------------------------------------------------------------------------------


// ------------------------------------- CALLBACK METHODS -------------------------------------

void DynamicPlanner::stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    //Stop the robot if true
    if (msg->data)
    {
        stop();
    }
}

void DynamicPlanner::jointsState_callback(const sensor_msgs::msg::JointState::SharedPtr &joints_state)
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
      
      // Insert the joint velocity into a velocity map (assuming you have dq_jts_map_group_ for velocities)
      dq_jts_map_group_[it->first] = (i < joints_state->velocity.size()) ? joints_state->velocity[i] : 0.0;

      // Increment the number of joints received from the joints state subscriber
      counter_group++;

      // If we have reached the last joint of the group
      if (counter_group == joints_names_group_.size())
      {
        // Iterate over the joints
        for (uint k = 0; k < joints_names_group_.size(); k++)
        {
          // Store the joints values from the joints map
          joints_values_group_[k] = joints_map_group_[joints_names_group_[k]];
          joints_speed_group_[k]  = dq_jts_map_group_[joints_names_group_[k]];
        }

        // Log gripper planning group
        RCLCPP_INFO(node_->get_logger(), "%s joints values received.", planning_group_.c_str());
        joints_group_received_ = true;
      }
    }
  }
}

void DynamicPlanner::trajpoint_callback(const std_msgs::msg::UInt32::SharedPtr msg)
{
    //Update the current trajectory point
    trajpoint_ = msg->data;
}


// ------------------------------------- TRAJECTORY METHODS -------------------------------------

void DynamicPlanner::setTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory, const std::string &end_effector_link)
{
    /*
    Set the planned trajectory as well as other information used for dynamic planning, then publishes the joint_trajectory to the trajectory_pub_
    Args:
        trajectory: The trajectory
        end_effector_link: The end effector link the planning must be relative to in case planning_space_ is set to OPERATIVE_SPACE
    */
    robot_trajectory_ = trajectory;
    traj_end_effector_link_ = end_effector_link;
    trajpoint_ = 0UL;
    trajectory_msgs::msg::JointTrajectoryPoint final_traj_pt = robot_trajectory_.joint_trajectory.points.back();
    final_joint_positions_ = final_traj_pt.positions;

    if (planning_space_ == PlanningSpace::OPERATIVE_SPACE){
        if (end_effector_link == ""){
            traj_end_effector_link_ = end_effector_link;
            RCLCPP_WARN(node_->get_logger(), "End effector link not specified for trajectory. Using default link: %s.", end_effector_link_.c_str());
        }
        //This might be a redundant calculation in some cases but it allows for safe access to the final pose
        final_pose_ = getFKine(final_joint_positions_, traj_end_effector_link_).pose;
    }

    trajectory_pub_->publish(robot_trajectory_.joint_trajectory);
}

bool DynamicPlanner::processTrajectory(moveit_msgs::msg::RobotTrajectory &trajectory_msg) {
    robot_trajectory::RobotTrajectory robot_trajectory(
        planning_scene_->getRobotModel(),
        planning_group_
    );

    robot_trajectory.setRobotTrajectoryMsg(*getRobotState(), trajectory_msg.joint_trajectory);

    bool success = time_optimal_traj_gen->computeTimeStamps(
        robot_trajectory,
        totg_max_vel_scaling_factor,
        totg_max_acc_scaling_factor
    );

    if (!success) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to compute time stamps for trajectory");
        return false;
    }

    robot_trajectory.getRobotTrajectoryMsg(trajectory_msg);

    return success;
}

bool DynamicPlanner::checkTrajectory()
{
    //Check if the trajectory is still clear of obstacles
    size_t traj_size = robot_trajectory_.joint_trajectory.points.size();
    size_t bound = std::min(traj_size, trajpoint_ + traj_size / 3);
    moveit::core::RobotState robot_state = *getRobotState();

    for (size_t i = trajpoint_; i < bound; i++)
    {
        trajectory_msgs::msg::JointTrajectoryPoint traj_pt = robot_trajectory_.joint_trajectory.points[i];
        robot_state.setJointGroupPositions(planning_group_, traj_pt.positions);
        //robot_state.update();
        if (!planning_scene_->isStateColliding(robot_state)) {
            RCLCPP_INFO(node_->get_logger(), "Obstacle detected at point %ld", i);
            return false;
        }
    }

    return true;
}

void DynamicPlanner::recalculateTrajectory(size_t start_index)
{
    //Recalculates the trajectory from the point at start_index onwards
    moveit::core::RobotStatePtr robot_state = getRobotState();
    robot_state->setJointGroupPositions(planning_group_, final_joint_positions_);
    robot_state->update();
    move_group_->setStartState(*robot_state);

    //Create a new plan starting from the current position
    moveit::planning_interface::MoveGroupInterface::Plan new_plan;

    if(planning_space_ == PlanningSpace::OPERATIVE_SPACE){
        move_group_->setPoseTarget(final_pose_, traj_end_effector_link_);
        move_group_->plan(new_plan);
    } 
    else if (planning_space_ == PlanningSpace::JOINTS_SPACE) {
        move_group_->setJointValueTarget(final_joint_positions_);
        move_group_->plan(new_plan);
    }

    //Merge trajectories
    mergeTrajectory(new_plan.trajectory_, start_index);

}

void DynamicPlanner::mergeTrajectory(moveit_msgs::msg::RobotTrajectory &new_traj, size_t start_index)
{
    /*
    Merge new_traj with trajectory_ from start_index onwards
    Args:
        new_traj: The new trajectory
        start_index: The index from which trajectory_ will be deleted and new_traj will be inserted

    NOTE:
        For optimization purposes if the size of new_traj is smaller than trajectory_ the function 
        will append new_traj to trajectory_, otherwise it will replace trajectory_ and the points
        before start_index will be inserted ate the start
    */

    if (new_traj.joint_trajectory.points.size() <= robot_trajectory_.joint_trajectory.points.size())
    {
        //Append new_traj to trajectory_
        robot_trajectory_.joint_trajectory.points.erase(
            robot_trajectory_.joint_trajectory.points.begin() + start_index,
            robot_trajectory_.joint_trajectory.points.end()
        );
        robot_trajectory_.joint_trajectory.points.insert(
            robot_trajectory_.joint_trajectory.points.begin() + start_index,
            new_traj.joint_trajectory.points.begin(),
            new_traj.joint_trajectory.points.end()
        );
    }
    else
    {
        //Replace trajectory_ with new_traj
        new_traj.joint_trajectory.points.insert(
            new_traj.joint_trajectory.points.begin(),
            robot_trajectory_.joint_trajectory.points.begin(),
            robot_trajectory_.joint_trajectory.points.begin() + start_index
        );
        robot_trajectory_ = new_traj;
    }
}

// ------------------------------------- HELPER METHODS -------------------------------------

const bool DynamicPlanner::checkJointDiff(const std::vector<double>& final_position)
{
  // Set a reasonable threshold 
  double th = 0.0001;
  // Counter check: it's increased by 1 if two joint positions are similar
  unsigned long counter_check = 0;
  // Iterate above all joints
  for(uint k = 0; k < final_position.size(); k++)
  {
    // If current and goal single joint position are similar
    if ((joints_values_group_[k] - final_position[k] < +th) && 
        (joints_values_group_[k] - final_position[k] > -th))
    {
      counter_check++;  // Increment counter check
    }    
  }
  if (counter_check == joints_values_group_.size())
  {
    RCLCPP_WARN(node_->get_logger(), "User input error: sent goal state is near or equal to current joint pose.");
    RCLCPP_WARN(node_->get_logger(), "Checked %ld out of %ld joints equal to current position.",counter_check,final_position.size());
    return true;
  }
  else
  {
    return false;
  }  
}

void DynamicPlanner::updatePlannerParams()
{
    //Updates the planner with values stored in params_
    
    move_group_->setPlanningTime(params_.planning_time);
    move_group_->setNumPlanningAttempts(params_.num_attempts);
    move_group_->setPlannerId(params_.planner_id);
    move_group_->setGoalTolerance(params_.tolerance);

    move_group_->setMaxVelocityScalingFactor(params_.vel_factor);
    move_group_->setMaxAccelerationScalingFactor(params_.acc_factor);

    move_group_->setPoseReferenceFrame(world_frame_);
    move_group_->setEndEffectorLink(end_effector_link_);
}

geometry_msgs::msg::PoseStamped DynamicPlanner::toPoseStamped(const Eigen::Isometry3d &pose, const std::string &frame_id)
{
    /*
    Converts an Eigen::Isometry3d pose to a geometry_msgs::msg::PoseStamped
    Args:
        pose: The pose to convert
        frame_id: The reference frame of the pose
    Returns:
        The converted PoseStamped message
    */
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = node_->now();

    pose_stamped.pose.position.x = pose.translation().x();
    pose_stamped.pose.position.y = pose.translation().y();
    pose_stamped.pose.position.z = pose.translation().z();

    Eigen::Quaterniond quat(pose.rotation());
    pose_stamped.pose.orientation.x = quat.x();
    pose_stamped.pose.orientation.y = quat.y();
    pose_stamped.pose.orientation.z = quat.z();
    pose_stamped.pose.orientation.w = quat.w();

    return pose_stamped;
}