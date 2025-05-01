#include <manipulators/DynamicPlanner.h>

DynamicPlanner::DynamicPlanner(const rclcpp::Node::SharedPtr &node,
                               const std::string &planning_group,
                               DynamicPlannerParams params,
                               bool dynamic_behavior)
    : node_(node),
      params_(params),
      dynamic_behavior_(dynamic_behavior),
      planning_group_(planning_group),
      trajpoint_(0UL)
{
    RCLCPP_INFO(node_->get_logger(), "Initializing DynamicPlanner...");

    RCLCPP_INFO(node_->get_logger(), "Planning group: %s", planning_group_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Dynamic behavior: %s", dynamic_behavior_ ? "true" : "false");
    RCLCPP_INFO(node_->get_logger(), "Velocity factor: %f", params_.vel_factor);
    RCLCPP_INFO(node_->get_logger(), "Acceleration factor: %f", params_.acc_factor);
    RCLCPP_INFO(node_->get_logger(), "Position tolerance: %f", params_.position_tolerance);
    RCLCPP_INFO(node_->get_logger(), "Orientation tolerance: %f", params_.orientation_tolerance);
    RCLCPP_INFO(node_->get_logger(), "Joint tolerance: %f", params_.joint_tolerance);
    RCLCPP_INFO(node_->get_logger(), "World frame: %s", params_.world_frame.c_str());
    RCLCPP_INFO(node_->get_logger(), "End effector link: %s", params_.end_effector_link.c_str());
    
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        node_, 
        moveit::planning_interface::MoveGroupInterface::Options(
            planning_group,
            "robot_description",
            node_->get_namespace()
        )
    );

    move_group_->startStateMonitor();

    joints_names_group_ = move_group_->getJointNames();

    joints_values_group_.resize(joints_names_group_.size());    // Adjust joints values array size
    joints_speed_group_.resize(joints_names_group_.size());     // Adjust joints values array size

    // Initialize joints map for robot state update: per each joint name, set its value to 0
    for (const std::string& name : joints_names_group_)
    {
        RCLCPP_INFO(node_->get_logger(), "Joint name: %s", name.c_str());
        joints_map_group_[name] = 0.;
        dq_jts_map_group_[name] = 0.;
    }

    //Fetch robot state
    joint_names_ = move_group_->getJointNames();

    RCLCPP_INFO(node_->get_logger(), "MoveGroupInterface initialized");

    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(move_group_->getRobotModel());

    //Initialize visual tools
    //Moveit
    moveit_visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(node_,
                                                                                    world_frame_, 
                                                                                    "moveit_visual_markers", 
                                                                                    move_group_->getRobotModel());

    //Rviz
    rviz_visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(world_frame_, "moveit_visual_markers", node_));

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

DynamicPlanner::~DynamicPlanner()
{
    RCLCPP_INFO(node_->get_logger(), "Destroying DynamicPlanner...");
    move_group_.reset();
    planning_scene_interface_.reset();
    planning_scene_.reset();
    moveit_visual_tools_.reset();
    rviz_visual_tools_.reset();
    node_.reset();
}

void DynamicPlanner::initialize()
{
    std::string ns = node_->get_namespace();
    ns = ns == "/" ? "" : ns; // Remove leading slash if namespace is empty

    RCLCPP_INFO(node_->get_logger(), "Node namespace: %s", ns.c_str());

    // Publishers
    joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(ns + "/move_group/fake_controller_joint_states", 1);
    trajectory_pub_ = node_->create_publisher<manipulator_interfaces::msg::TrajectoryResult>(planning_group_ + "/planned_trajectory", 1);
    
    // Subscribers
    trajectory_sub_ = node_->create_subscription<moveit_msgs::msg::RobotTrajectory>(
        planning_group_ + "/trajectory", 1,
        [this](const moveit_msgs::msg::RobotTrajectory::SharedPtr msg) {
            setTrajectory(*msg);
        }
    );

    joints_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        ns + "/joint_states", 1,
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
    manipulator_interfaces::msg::TrajectoryResult result_msg;

    // PRE PLANNING CHECKS
    // 1. joint_positions are not too close to the current position
    // 2. joint_positions are within the joint limits

    if (checkJointDiff(joint_positions)){
        RCLCPP_ERROR(node_->get_logger(), "Joint positions are too close to current position");
        moveit_msgs::msg::RobotTrajectory trajectory;
        result_msg.success = false;
        result_msg.message = "Joint positions are too close to the current ones";
        result_msg.trajectory = trajectory;
        result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::SAME_POSITION;

        trajectory_pub_->publish(result_msg);
        setTrajectory(trajectory); //Set empty trajectory to avoid re-execution of previous trajectory
        return;
    }

    if (!is_within_bounds) //Check if the joint positions are within the joint limits
    {
        RCLCPP_ERROR(node_->get_logger(), "Joint positions are out of bounds");
        moveit_msgs::msg::RobotTrajectory trajectory;
        result_msg.success = false;
        result_msg.message = "Joint positions are out of bounds";
        result_msg.trajectory = trajectory;
        result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::OUT_OF_BOUNDS;

        trajectory_pub_->publish(result_msg);
        setTrajectory(trajectory); //Set empty trajectory to avoid re-execution of previous trajectory
        return;
    }

    //Create the plan
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode error = move_group_->plan(plan);

    // PAST PLANNING CHECKS
    // 1. Planning succeded
    // 2. Trajectory end point matches with the target joint positions
    // 3. Time optimal trajectory generation succeded
    // 4. Trajectory respects the path constraints

    if (error != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed with error code: %d", error.val);
        moveit_msgs::msg::RobotTrajectory trajectory;
        result_msg.success = false;
        result_msg.message = "Planning failed";
        result_msg.trajectory = trajectory;
        result_msg.error_code = error.val;

        setTrajectory(trajectory);
        trajectory_pub_->publish(result_msg);
        return;
    }

    moveit_msgs::msg::RobotTrajectory trajectory = plan.trajectory_;

    trajectory_msgs::msg::JointTrajectoryPoint last_point = trajectory.joint_trajectory.points.back();
    if(!checkJointDiff(last_point.positions, joint_positions)){
        RCLCPP_ERROR(node_->get_logger(), "Trajectory end point does not match with the target joint positions");
        moveit_msgs::msg::RobotTrajectory trajectory;
        result_msg.success = false;
        result_msg.message = "Trajectory end point does not match with the target joint positions";
        result_msg.trajectory = trajectory;
        result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::END_POINT_MISMATCH;

        setTrajectory(trajectory);
        trajectory_pub_->publish(result_msg);
        return;
    }

    bool totg_success = processTrajectory(trajectory); //Apply time optimal trajectory generation

    if (!totg_success){ //Check time opetimal trajectory generation success
        result_msg.success = false;
        result_msg.message = "Time optimal trajectory generation failed";
        result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::TIME_OPTIMAL_FAILED;
        result_msg.trajectory = trajectory;
        setTrajectory(moveit_msgs::msg::RobotTrajectory());
        trajectory_pub_->publish(result_msg);
        return;
    }

    if (!checkTrajectoryConstraints(trajectory)){ //Check that the trajectory respects the path constraints
        result_msg.success = false;
        result_msg.message = "Trajectory violates path constraints";
        result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::CONSTRAINTS_VIOLATED;
        result_msg.trajectory = trajectory;

        setTrajectory(moveit_msgs::msg::RobotTrajectory());
        trajectory_pub_->publish(result_msg);
        return;
    }

    setTrajectory(trajectory);

    //Visualize trajectory line
    auto robot_model = move_group_->getRobotModel();
    moveit_visual_tools_->publishTrajectoryLine(trajectory, 
                                                robot_model->getLinkModel(end_effector_link_),
                                                robot_model->getJointModelGroup(planning_group_));

    result_msg.success = true;
    result_msg.message = "Trajectory planned successfully";
    result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::SUCCESS;
    result_msg.trajectory = trajectory;
    trajectory_pub_->publish(result_msg);
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

    manipulator_interfaces::msg::TrajectoryResult result_msg;

    //Sets the target pose
    move_group_->setPoseReferenceFrame(frame);
    move_group_->setPoseTarget(goal_pose, ee_link);

    //Create the plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode error = move_group_->plan(plan);

    // Clear pose target
    move_group_->clearPoseTarget(ee_link);

    // PAST PLANNING CHECKS
    // 1. Planning succeded
    // 2. Trajectory end point matches with the target pose
    // 3. Time optimal trajectory generation succeded
    // 4. Trajectory respects the path constraints

    if (error != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed with error code: %d", error.val);
        moveit_msgs::msg::RobotTrajectory trajectory;
        result_msg.success = false;
        result_msg.message = "Planning failed";
        result_msg.trajectory = trajectory;
        result_msg.error_code = error.val;

        setTrajectory(trajectory);
        trajectory_pub_->publish(result_msg);
        return;
    }

    moveit_msgs::msg::RobotTrajectory trajectory = plan.trajectory_;

    trajectory_msgs::msg::JointTrajectoryPoint last_point = trajectory.joint_trajectory.points.back();
    if(!checkPoseDiff(getFKine(last_point.positions, ee_link).pose, goal_pose)){
        RCLCPP_ERROR(node_->get_logger(), "Trajectory end point does not match with the target pose");
        moveit_msgs::msg::RobotTrajectory trajectory;
        result_msg.success = false;
        result_msg.message = "Trajectory end point does not match with the target pose";
        result_msg.trajectory = trajectory;
        result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::END_POINT_MISMATCH;

        setTrajectory(trajectory);
        trajectory_pub_->publish(result_msg);
        return;
    }

    bool totg_success = processTrajectory(trajectory); //Apply time optimal trajectory generation

    if (!totg_success){ //Check time opetimal trajectory generation success
        result_msg.success = false;
        result_msg.message = "Time optimal trajectory generation failed";
        result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::TIME_OPTIMAL_FAILED;
        result_msg.trajectory = trajectory;
        
        setTrajectory(moveit_msgs::msg::RobotTrajectory());
        trajectory_pub_->publish(result_msg);
        return;
    }

    if (!checkTrajectoryConstraints(trajectory)){ //Check that the trajectory respects the path constraints
        result_msg.success = false;
        result_msg.message = "Trajectory violates path constraints";
        result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::CONSTRAINTS_VIOLATED;
        result_msg.trajectory = trajectory;

        setTrajectory(moveit_msgs::msg::RobotTrajectory());
        trajectory_pub_->publish(result_msg);
        return;
    }


    //Trajectory meets all the requirements
    setTrajectory(trajectory);

    //Visualize trajectory line
    auto robot_model = move_group_->getRobotModel();
    moveit_visual_tools_->publishTrajectoryLine(trajectory, 
                                                robot_model->getLinkModel(ee_link),
                                                robot_model->getJointModelGroup(planning_group_));

    result_msg.success = true;
    result_msg.message = "Trajectory planned successfully";
    result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::SUCCESS;
    result_msg.trajectory = trajectory;
    trajectory_pub_->publish(result_msg);
}

void DynamicPlanner::plan(const geometry_msgs::msg::Pose& goal_pose, const std::string& ee_link)
{
    plan(goal_pose, ee_link, world_frame_);
}

void DynamicPlanner::plan(const geometry_msgs::msg::Pose& goal_pose)
{
    plan(goal_pose, params_.end_effector_link, world_frame_);
}

double DynamicPlanner::cartesianPlan(const std::vector<geometry_msgs::msg::Pose>& waypoints)
{
    // Setup cartesian planner
    double jump_treshold = 0.0;
    double eef_step = params_.max_velocity*params_.sample_time; // Ideal distance step
    double fraction = 0.0;
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    manipulator_interfaces::msg::TrajectoryResult result_msg;
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
    bool totg_success = processTrajectory(trajectory); //Apply time optimal trajectory generation

    if (!totg_success){
        result_msg.success = false;
        result_msg.message = "Time optimal trajectory generation failed";
        result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::TIME_OPTIMAL_FAILED;
        result_msg.trajectory = trajectory;
        
        setTrajectory(moveit_msgs::msg::RobotTrajectory());
        trajectory_pub_->publish(result_msg);
        return -1.0;
    }

    if (!checkTrajectoryConstraints(trajectory)){ //Check that the trajectory respects the path constraints
        result_msg.success = false;
        result_msg.message = "Trajectory violates path constraints";
        result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::CONSTRAINTS_VIOLATED;
        result_msg.trajectory = trajectory;

        setTrajectory(moveit_msgs::msg::RobotTrajectory());
        trajectory_pub_->publish(result_msg);
        return -1.0;
    }

    // Display results
    RCLCPP_INFO(node_->get_logger(), 
                "Computed cartesian path of %.2f%% fraction achieved, in time %.6f s", 
                fraction * 100.0,
                std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count());
  
    
    setTrajectory(trajectory);

    //Visualize trajectory line
    auto robot_model = move_group_->getRobotModel();
    moveit_visual_tools_->publishTrajectoryLine(trajectory, 
                                                robot_model->getLinkModel(end_effector_link_),
                                                robot_model->getJointModelGroup(planning_group_));

    result_msg.success = true;
    result_msg.message = "Trajectory planned successfully";
    result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::SUCCESS;
    result_msg.trajectory = trajectory;
    trajectory_pub_->publish(result_msg);

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
    joint_state.velocity = traj_pt.velocities;
    joint_state.effort = traj_pt.effort;
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

    if(!checkJointDiff(robot_trajectory_.joint_trajectory.points[0].positions)){
        RCLCPP_ERROR(node_->get_logger(), "Trajectory doesn't start from the current position");
        return;
    }

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

DynamicPlannerParams DynamicPlanner::getParams() const
{
    return params_;
}

void DynamicPlanner::setParams(const DynamicPlannerParams &params)
{
    params_ = params;
    updatePlannerParams();
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

    rviz_visual_tools_->deleteAllMarkers("pos_constraints");

    for (const auto &constraint : constraints.position_constraints)
    {
        for(size_t i {0}; i < constraint.constraint_region.primitives.size(); i++)
        {
            const auto &primitive = constraint.constraint_region.primitives[i];
            const auto &pose = constraint.constraint_region.primitive_poses[i];
            visualizePrimitive(primitive, pose, {0.0, 0.9, 0.1, 0.3}, "pos_constraints");
        }
    }

    rviz_visual_tools_->trigger();
}

moveit_msgs::msg::Constraints DynamicPlanner::getPathConstraints() const
{
    return move_group_->getPathConstraints();
}

void DynamicPlanner::clearPathConstraints()
{
    move_group_->clearPathConstraints();
    rviz_visual_tools_->deleteAllMarkers("pos_constraints");
    rviz_visual_tools_->trigger();
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
        // move_group_->getCurrentJointValues(),
        joints_values_group_,
        end_effector_link
    );
}

geometry_msgs::msg::PoseStamped DynamicPlanner::getFKine()
{
    return getFKine(params_.end_effector_link);
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
    return invKine(target_pose, params_.end_effector_link);
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
    return getJacobian(params_.end_effector_link);
}

const Eigen::MatrixXd DynamicPlanner::getPseudoInverseJacobian(const std::string &end_effector_link)
{
    return getJacobian(end_effector_link).completeOrthogonalDecomposition().pseudoInverse();
}

const Eigen::MatrixXd DynamicPlanner::getPseudoInverseJacobian()
{
    return getPseudoInverseJacobian(params_.end_effector_link);
}

// -------------------------------------------------------------------------------------------
// ------------------------------------- PRIVATE METHODS -------------------------------------
// -------------------------------------------------------------------------------------------


// ------------------------------------- CALLBACK METHODS -------------------------------------

void DynamicPlanner::jointsState_callback(const sensor_msgs::msg::JointState::SharedPtr &joints_state)
{
    // Map to store couples joint name - joint values
    static std::unordered_map<std::string, double>::iterator it;
    uint counter_group  = 0;

    for (uint i = 0; i < joints_state->name.size(); i++)
    {
        // Look for joints group names within joints current state
        it = joints_map_group_.find(joints_state->name[i]);

        if (it != joints_map_group_.end()) //Check if joint has been found in planning group
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
                if (!isReady())
                {
                    RCLCPP_INFO(node_->get_logger(), "%s joints values received, planner is ready.", planning_group_.c_str());
                }
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
    Set the planned trajectory as well as other information used for dynamic planning
    Args:
        trajectory: The trajectory
        end_effector_link: The end effector link the planning must be relative to in case planning_space_ is set to OPERATIVE_SPACE
    */
    robot_trajectory_ = trajectory;
    if (trajectory.joint_trajectory.points.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "Empty trajectory");
        return;
    }
    traj_end_effector_link_ = end_effector_link;
    trajpoint_ = 0UL;
    trajectory_msgs::msg::JointTrajectoryPoint final_traj_pt = robot_trajectory_.joint_trajectory.points.back();
    final_joint_positions_ = final_traj_pt.positions;

    if (planning_space_ == PlanningSpace::OPERATIVE_SPACE){
        if (end_effector_link == ""){
            traj_end_effector_link_ = end_effector_link;
            RCLCPP_WARN(node_->get_logger(), "End effector link not specified for trajectory. Using default link: %s.", params_.end_effector_link.c_str());
        }
        //This might be a redundant calculation in some cases but it allows for safe access to the final pose
        final_pose_ = getFKine(final_joint_positions_, traj_end_effector_link_).pose;
    }
}

bool DynamicPlanner::processTrajectory(moveit_msgs::msg::RobotTrajectory &trajectory_msg) {
    robot_trajectory::RobotTrajectory robot_trajectory(
        planning_scene_->getRobotModel(),
        planning_group_
    );

    robot_trajectory.setRobotTrajectoryMsg(*getRobotState(), trajectory_msg.joint_trajectory);

    bool success = time_optimal_traj_gen->computeTimeStamps(
        robot_trajectory,
        params_.vel_factor,
        params_.acc_factor
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

bool DynamicPlanner::checkTrajectoryConstraints(const moveit_msgs::msg::RobotTrajectory &trajectory)
{
    /*
    Check if the trajectory is within the constraints
    Args:
        trajectory: The trajectory to check
    Returns:
        True if the trajectory is within the constraints, false otherwise
    */

    // Check if the trajectory is within the constraints

    moveit::core::RobotState robot_state = *getRobotState();
    moveit_msgs::msg::Constraints constraints = getPathConstraints();

    for (const auto &point : trajectory.joint_trajectory.points)
    {
        robot_state.setJointGroupPositions(planning_group_, point.positions);
        robot_state.update();

        // Check if the trajectory is within the constraints
        if (!planning_scene_->isStateConstrained(robot_state, constraints))
        {
            return false;
        }
    }

    return true;
}

// ------------------------------------- HELPER METHODS -------------------------------------

bool DynamicPlanner::checkJointDiff(const std::vector<double>& joint_positions)
{
    if (!isReady()){
        RCLCPP_ERROR(node_->get_logger(), "Unable to check joint difference: joints group not received.");
        return false;
    }

    return checkJointDiff(joint_positions, joints_values_group_);
}

bool DynamicPlanner::checkJointDiff(const std::vector<double>& val_a, const std::vector<double>& val_b)
{
    // Set a reasonable threshold 
    double th = params_.joint_tolerance;
    // Counter check: it's increased by 1 if two joint positions are similar
    unsigned long counter_check = 0;
    // Iterate above all joints

    if(val_a.size() != val_b.size())
    {
        RCLCPP_ERROR(node_->get_logger(), "Joint positions vectors are not the same size");
        return false;
    }
    
    for(unsigned long k = 0; k < val_a.size(); k++)
    {
        // If current and goal single joint position are similar
        if ((val_b[k] - val_a[k] < +th) && 
            (val_b[k] - val_a[k] > -th))
        {
            counter_check++;  // Increment counter check
        }    
    }
    return (counter_check == val_a.size());
}

bool DynamicPlanner::checkPoseDiff(const geometry_msgs::msg::Pose& pose, const std::string& end_effector_link)
{
    // Check if the pose is too close to the current pose
    return checkPoseDiff(pose, getFKine(end_effector_link).pose);
}

bool DynamicPlanner::checkPoseDiff(const geometry_msgs::msg::Pose& pose_a, const geometry_msgs::msg::Pose& pose_b)
{
    //For now orientation is not considered
    // Set a reasonable threshold 
    double pos_th = params_.position_tolerance;
    //double rot_th = params_.orientation_tolerance;

    bool check = true;

    check = ((abs(pose_a.position.x - pose_b.position.x) < pos_th) && check);
    check = ((abs(pose_a.position.y - pose_b.position.y) < pos_th) && check);
    check = ((abs(pose_a.position.z - pose_b.position.z) < pos_th) && check);

    return check;
}

void DynamicPlanner::updatePlannerParams()
{
    //Updates the planner with values stored in params_
    
    move_group_->setPlanningTime(params_.planning_time);
    move_group_->setNumPlanningAttempts(params_.num_attempts);
    move_group_->setPlannerId(params_.planner_id);

    //Set tolerances
    move_group_->setGoalPositionTolerance(params_.position_tolerance);
    move_group_->setGoalOrientationTolerance(params_.orientation_tolerance);
    move_group_->setGoalJointTolerance(params_.joint_tolerance);

    move_group_->setMaxVelocityScalingFactor(params_.vel_factor);
    move_group_->setMaxAccelerationScalingFactor(params_.acc_factor);

    move_group_->setPoseReferenceFrame(world_frame_);
    move_group_->setEndEffectorLink(params_.end_effector_link);
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

// ------------------------------------- VISUALIZATION METHODS -------------------------------------

void DynamicPlanner::visualizePrimitive(const shape_msgs::msg::SolidPrimitive &primitive, 
                                        const geometry_msgs::msg::Pose &primitive_pose,
                                        const std::vector<double> rgba_color,
                                        const std::string &ns)
{
    /*
    Visualizes the primitive
    Args:
        primitive: The primitive to visualize
        primitive_pose: The pose of the primitive
        rgba_color: Color in rgba format
        ns: Namespace for the marker

    NOTE: Remember to use rviz_visual_tools_->trigger() to publish the marker
    */
    if (rviz_visual_tools_ == nullptr)
    {
        RCLCPP_ERROR(node_->get_logger(), "Rviz visual tools not initialized");
        return;
    }

    //Create a color message
    std_msgs::msg::ColorRGBA color;
    color.r = rgba_color[0];
    color.g = rgba_color[1];
    color.b = rgba_color[2];
    color.a = rgba_color[3];

    //Iterate over each primitive and add publish the shape through rviz visual tools

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = node_->now();
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = ns;
    marker.pose = primitive_pose;
    marker.color = color;
    marker.id = int32_t(node_->now().seconds());

    switch(primitive.type)
    {
        case shape_msgs::msg::SolidPrimitive::BOX:
        {
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.scale.x = primitive.dimensions[0];
            marker.scale.y = primitive.dimensions[1];
            marker.scale.z = primitive.dimensions[2];
            break;
        }
        case shape_msgs::msg::SolidPrimitive::SPHERE:
        {
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.scale.x = primitive.dimensions[0];
            marker.scale.y = primitive.dimensions[0];
            marker.scale.z = primitive.dimensions[0];
            break;
        }
        case shape_msgs::msg::SolidPrimitive::CYLINDER:
        {
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.scale.x = primitive.dimensions[0];
            marker.scale.y = primitive.dimensions[0];
            marker.scale.z = primitive.dimensions[1];
            break;
        }
        case shape_msgs::msg::SolidPrimitive::CONE:
        {
            RCLCPP_ERROR(node_->get_logger(), "Cones are not supported for visualization yet.");
            break;
        }
        default:
        {
            RCLCPP_ERROR(node_->get_logger(), "Unknown primitive type");
            break;
        }
    }

    rviz_visual_tools_->publishMarker(marker);    
}