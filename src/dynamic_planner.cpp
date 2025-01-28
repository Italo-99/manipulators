#include <manipulators/DynamicPlanner.h>

DynamicPlanner::DynamicPlanner(const rclcpp::Node::SharedPtr &node,
                               const std::string &planning_group,
                               const double vel_factor,
                               const double acc_factor,
                               bool dynamic_behavior)
    : node_(node),
      params_(),
      dynamic_behavior_(dynamic_behavior),
      planning_group_(planning_group)
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
    updatePlannerParams();
    initialize();

    RCLCPP_INFO(node_->get_logger(), "DynamicPlanner initialized");
}

void DynamicPlanner::initialize()
{
    //Initialize the dynamic planner (vars, subscribers, publishers, ...)
    joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("/move_group/fake_controller_joint_states", 5);
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

    if (!is_within_bounds) //Check if the joint positions are within the joint limits
    {
        RCLCPP_ERROR(node_->get_logger(), "Joint positions are out of bounds");
        return;
    }

    //Create the plan and execute
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode error = move_group_->plan(plan);
    if (error != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed with error code: %d", error.val);
        return;
    }
    setTrajectory(plan.trajectory_);
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

    //Sets the target pose
    move_group_->setPoseReferenceFrame(frame);
    move_group_->setPoseTarget(goal_pose, ee_link);

    //Create the plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode error = move_group_->plan(plan);
    if (error != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed with error code: %d", error.val);
        return;
    }
    setTrajectory(plan.trajectory_);
}

void DynamicPlanner::plan(const geometry_msgs::msg::Pose& goal_pose, const std::string& ee_link)
{
    plan(goal_pose, ee_link, world_frame_);
}

void DynamicPlanner::plan(const geometry_msgs::msg::Pose& goal_pose)
{
    plan(goal_pose, end_effector_link_, world_frame_);
}

void DynamicPlanner::moveRobot(const trajectory_msgs::msg::JointTrajectoryPoint& traj_pt)
{
    //Executes a single trajectory position

    sensor_msgs::msg::JointState joint_state;
    // Fill the name of the joints
    joint_state.name = joint_names_;
    joint_state.position = traj_pt.positions;
    joint_state_publisher_->publish(joint_state);
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
    double points_time = robot_trajectory_.joint_trajectory.points[1].time_from_start.sec + 
                         robot_trajectory_.joint_trajectory.points[1].time_from_start.nanosec * 1e-9;
    
    is_moving = true;

    rclcpp::Rate traj_exec_rate(1/(points_time));

    double time_tracker_check_only = 0.0;
    double time_tracker_recalculation = 0.0;

    while (traj_current_index_ < robot_trajectory_.joint_trajectory.points.size())
    {
        //RCLCPP_INFO(node_->get_logger(), "Executing trajectory point %ld", traj_current_index_);
        trajectory_msgs::msg::JointTrajectoryPoint traj_pt = robot_trajectory_.joint_trajectory.points[traj_current_index_];

        if (is_moving == false){
                //Send a trajectory msg with the last position and velocities set to 0
                trajectory_msgs::msg::JointTrajectoryPoint stopping_point = traj_pt;
                stopping_point.velocities = std::vector<double>(joint_names_.size(), 0.);
                moveRobot(stopping_point);
                break;
        } else if (dynamic_behavior_){
            //Check if the trajectory is still clear of obstacles
            auto dynamic_start = std::chrono::high_resolution_clock::now();
            auto dynamic_check = std::chrono::high_resolution_clock::now();

            if (!checkTrajectory()){
                dynamic_check = std::chrono::high_resolution_clock::now();
                //If not recalculate
                RCLCPP_INFO(node_->get_logger(), "Recalculating trajectory");
                //recalculateTrajectory(traj_current_index_);
            } else {
                dynamic_check = std::chrono::high_resolution_clock::now();
            }
            auto dynamic_end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> check_time = dynamic_check - dynamic_start;
            std::chrono::duration<double, std::milli> recalc_time = dynamic_end - dynamic_check;
            time_tracker_check_only += check_time.count();
            time_tracker_recalculation += recalc_time.count();
        }

        // Execute the move
        moveRobot(traj_pt);

        traj_current_index_++; //Increment the index
        traj_exec_rate.sleep();
    }

    RCLCPP_INFO(node_->get_logger(), "Trajectory execution finished");
    RCLCPP_INFO(node_->get_logger(), "Trajectory check time statistics: total=%f ms ; mean=%f", 
        time_tracker_check_only,
        time_tracker_check_only / robot_trajectory_.joint_trajectory.points.size());

    RCLCPP_INFO(node_->get_logger(), "Trajectory recalculation time statistics: total=%f ms ; mean=%f", 
        time_tracker_recalculation,
        time_tracker_recalculation / robot_trajectory_.joint_trajectory.points.size());

    RCLCPP_INFO(node_->get_logger(), "Time between each trajectory point: %f ms",traj_exec_rate.period().count() / 1000000.0);

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


// ------------------------------------- PRIVATE METHODS -------------------------------------

void DynamicPlanner::setTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory, const std::string &end_effector_link)
{
    /*
    Set the planned trajectory as well as other information used for dynamic planning
    Args:
        trajectory: The trajectory
        end_effector_link: The end effector link the planning must be relative to in case planning_space_ is set to OPERATIVE_SPACE
    */
    robot_trajectory_ = trajectory;
    traj_end_effector_link_ = end_effector_link;
    traj_current_index_ = 0;
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
}

bool DynamicPlanner::checkTrajectory()
{
    //Check if the trajectory is still clear of obstacles
    size_t traj_size = robot_trajectory_.joint_trajectory.points.size();
    size_t bound = std::min(traj_size, traj_current_index_ + traj_size / 3);
    moveit::core::RobotState robot_state = *getRobotState();

    for (size_t i = traj_current_index_; i < bound; i++)
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