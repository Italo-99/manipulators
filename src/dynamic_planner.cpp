#include <manipulators/DynamicPlanner.h>

DynamicPlanner::DynamicPlanner(const rclcpp::Node::SharedPtr &node,
                               const std::string &planning_group)
    : node_(node),
      planning_group_(planning_group),
      trajpoint_(0UL)
{
    RCLCPP_INFO(node_->get_logger(), "Initializing DynamicPlanner...");

    params_ = DynamicPlannerParams::fromNode(node_);

    RCLCPP_INFO(node_->get_logger(), "Planning group: %s", planning_group_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Velocity factor: %f", params_.vel_factor);
    RCLCPP_INFO(node_->get_logger(), "Acceleration factor: %f", params_.acc_factor);
    RCLCPP_INFO(node_->get_logger(), "Position tolerance: %f", params_.position_tolerance);
    RCLCPP_INFO(node_->get_logger(), "Orientation tolerance: %f", params_.orientation_tolerance);
    RCLCPP_INFO(node_->get_logger(), "Joint tolerance: %f", params_.joint_tolerance);
    RCLCPP_INFO(node_->get_logger(), "World frame: %s", params_.world_frame.c_str());
    RCLCPP_INFO(node_->get_logger(), "End effector link: %s", params_.end_effector_link.c_str());
    
    kinematic_model_ = moveit::planning_interface::getSharedRobotModel(node_, "robot_description");
    kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);

    const moveit::core::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup(planning_group_);
    const size_t num_joints = joint_model_group->getActiveJointModels().size();

    if (num_joints != 6) {
        RCLCPP_ERROR(node_->get_logger(), 
                     "Found %zu joints for model group %s, DynamicPlanner is built to work with 6 joints, this can lead to undefined behavior",
                     num_joints, planning_group_.c_str());
    }

    joints_values_group_.resize(num_joints);    // Adjust joints values array size
    joints_speed_group_.resize(num_joints);     // Adjust joints values array size
    joints_names_group_ = joint_model_group->getActiveJointModelNames();

    //RCLCPP_INFO(node_->get_logger(), "Found [%s] joints for planning group %s", std::string{joints_names_group_.begin(), joints_names_group_.end()}.c_str(), planning_group_.c_str());

    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, "robot_description");
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->startStateMonitor();

    //Rviz visual tools
    rviz_visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(params_.world_frame, "moveit_visual_markers", node_));

    //Initialize time optimal trajectory generation
    time_optimal_traj_gen_ = std::make_shared<trajectory_processing::TimeOptimalTrajectoryGeneration>(
        totg_tolerance,
        params_.sample_time,
        totg_min_angle_change
    );

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    initialize();

    RCLCPP_INFO(node_->get_logger(), "DynamicPlanner initialized");
}

DynamicPlanner::~DynamicPlanner()
{
    RCLCPP_INFO(node_->get_logger(), "Destroying DynamicPlanner...");
    planning_scene_monitor_.reset();
    rviz_visual_tools_.reset();
    node_.reset();
}

void DynamicPlanner::initialize()
{
    std::string ns = node_->get_namespace();
    ns = ns == "/" ? "" : ns; // Remove leading slash if namespace is empty

    RCLCPP_INFO(node_->get_logger(), "Node namespace: %s", ns.c_str());

    cb_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );

    // Publishers
    joint_cmd_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(ns + "/move_group/fake_controller_joint_states", 1);
    trajectory_pub_ = node_->create_publisher<manipulator_interfaces::msg::TrajectoryResult>(planning_group_ + "/planned_trajectory", 1);
    
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = cb_group_;

    collision_object_pub_ = node_->create_publisher<moveit_msgs::msg::CollisionObject>(
        "collision_object", 1
    );
    attached_collision_object_pub_ = node_->create_publisher<moveit_msgs::msg::AttachedCollisionObject>(
        "attached_collision_object", 1
    );

    // Subscribers
    trajectory_sub_ = node_->create_subscription<moveit_msgs::msg::RobotTrajectory>(
        planning_group_ + "/trajectory", qos_reliable(1),
        [this](const moveit_msgs::msg::RobotTrajectory::SharedPtr msg) {
            RCLCPP_INFO(node_->get_logger(), "Received trajectory");
            setTrajectory(*msg);
        },
        sub_options
    );

    joints_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        ns + "/joint_states", qos_best_effort(1),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->jointsState_callback(msg);
        },
        sub_options
    );

    execution_ctrl_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        planning_group_ + "/execution_control", qos_reliable(1), 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            this->executionControl_callback(msg);
        },
        sub_options
    );

    //Timers
    traj_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(params_.sample_time * 1000)),
        [this]() {
            this->trajectoryExecution_callback();
        },
        cb_group_
    );

    //Action clients
    plan_action_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
        node_,
        move_group::MOVE_ACTION
    );

    //Service clients
    cartesian_motion_sequence_client_ = node_->create_client<moveit_msgs::srv::GetMotionSequence>(
        "plan_sequence_path"
    );
}

// ------------------------------------- PUBLIC METHODS -------------------------------------


//PLAN: Joint space

void DynamicPlanner::plan(const std::vector<double> joint_positions, const moveit::core::RobotStatePtr start_state)
{
    /*
    Plans and executes a trajectory to the joint positions for the manipulator (joint space)
    Args:
        joint_positions: Array of target joint positions
        start_state: Robot state to start the planning from
    */

    stop(); // Stop any ongoing trajectory execution

    //Sets the target joint positions
    
    moveit::core::RobotState goal_state(*kinematic_state_);
    goal_state.setJointGroupPositions(planning_group_, joint_positions);

    bool goal_valid = getPlanningScene()->isStateValid(goal_state, "", true);

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

    if (!goal_valid) //Check if the joint positions are within the joint limits
    {
        RCLCPP_ERROR(node_->get_logger(), "Requested joint positions are invalid.");
        moveit_msgs::msg::RobotTrajectory trajectory;
        result_msg.success = false;
        result_msg.message = "Requested joint positions are invalid.";
        result_msg.trajectory = trajectory;
        result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::OUT_OF_BOUNDS;

        trajectory_pub_->publish(result_msg);
        setTrajectory(trajectory); //Set empty trajectory to avoid re-execution of previous trajectory
        return;
    }

    //Create the plan
    
    moveit_msgs::msg::MotionPlanRequest motion_plan_request = createMotionPlanRequest();
    moveit::core::robotStateToRobotStateMsg(*start_state, motion_plan_request.start_state);
    motion_plan_request.goal_constraints.push_back(createJointGoalConstraints(joint_positions));

    moveit_msgs::action::MoveGroup::Result plan = computeMotionPlan(motion_plan_request);

    // PAST PLANNING CHECKS
    // 1. Planning succeded
    // 2. Trajectory end point matches with the target joint positions
    // 3. Time optimal trajectory generation succeded
    // 4. Trajectory respects the path constraints

    if (plan.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed with error code: %d", plan.error_code.val);
        moveit_msgs::msg::RobotTrajectory trajectory;
        result_msg.success = false;
        result_msg.message = "Planning failed";
        result_msg.trajectory = trajectory;
        result_msg.error_code = plan.error_code.val;

        setTrajectory(trajectory);
        trajectory_pub_->publish(result_msg);
        return;
    }

    moveit_msgs::msg::RobotTrajectory trajectory = plan.planned_trajectory;

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

    result_msg.success = true;
    result_msg.message = "Trajectory planned successfully";
    result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::SUCCESS;
    result_msg.trajectory = trajectory;
    trajectory_pub_->publish(result_msg);
}

void DynamicPlanner::plan(const std::vector<double> joint_positions)
{
    /*
    Plans and executes a trajectory to the joint positions for the manipulator (joint space)
    Args:
        joint_positions: Array of target joint positions
    */

    moveit::core::RobotStatePtr current_state = getRobotState();
    plan(joint_positions, current_state);
}

//PLAN: Cartesian space

void DynamicPlanner::plan(const geometry_msgs::msg::Pose& goal_pose, const std::string& ee_link, const std::string& frame, const moveit::core::RobotStatePtr start_state)
{
    /*
    Plan: pose goal
        Args:
            goal_pose: Target position
            ee_link: End effector link
            frame: Reference frame
            space: Planning space (joint or operative)
    */

    stop(); // Stop any ongoing trajectory execution

    // PRE PLANNING CHECK
    // 1. goal_pose is not too close to the current position
    
    geometry_msgs::msg::Pose pose_world;
    if (frame != params_.world_frame)
    {
        pose_world = transformPoseToWorld(goal_pose, frame);
    } else {
        pose_world = goal_pose;
    }

    if (checkPoseDiff(pose_world, ee_link))
    {
        RCLCPP_ERROR(node_->get_logger(), "Goal pose is too close to the current position");
        manipulator_interfaces::msg::TrajectoryResult result_msg;
        result_msg.success = false;
        result_msg.message = "Goal pose is too close to the current position";
        result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::SAME_POSITION;

        moveit_msgs::msg::RobotTrajectory trajectory;
        result_msg.trajectory = trajectory;

        setTrajectory(trajectory);
        trajectory_pub_->publish(result_msg);
        return;
    }

    manipulator_interfaces::msg::TrajectoryResult result_msg;
    
    //Create the plan and execute
    moveit_msgs::msg::MotionPlanRequest motion_plan_request = createMotionPlanRequest();
    moveit::core::robotStateToRobotStateMsg(*start_state, motion_plan_request.start_state);
    motion_plan_request.goal_constraints.push_back(createTcpGoalConstraints(pose_world, ee_link, params_.world_frame));

    moveit_msgs::action::MoveGroup::Result plan = computeMotionPlan(motion_plan_request);


    // POST PLANNING CHECKS
    // 1. Planning succeded
    // 2. Time optimal trajectory generation succeded
    // 3. Trajectory respects the path constraints

    if (plan.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed with error code: %d", plan.error_code.val);
        moveit_msgs::msg::RobotTrajectory trajectory;
        result_msg.success = false;
        result_msg.message = "Planning failed";
        result_msg.trajectory = trajectory;
        result_msg.error_code = plan.error_code.val;

        setTrajectory(trajectory);
        trajectory_pub_->publish(result_msg);
        return;
    }

    moveit_msgs::msg::RobotTrajectory trajectory = plan.planned_trajectory;

//    trajectory_msgs::msg::JointTrajectoryPoint last_point = trajectory.joint_trajectory.points.back();
//    if(!checkPoseDiff(getFKine(last_point.positions, ee_link).pose, goal_pose)){
//        RCLCPP_ERROR(node_->get_logger(), "Trajectory end point does not match with the target pose");
//        moveit_msgs::msg::RobotTrajectory trajectory;
//        result_msg.success = false;
//        result_msg.message = "Trajectory end point does not match with the target pose";
//        result_msg.trajectory = trajectory;
//        result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::END_POINT_MISMATCH;
//
//        setTrajectory(trajectory);
//        trajectory_pub_->publish(result_msg);
//        return;
//    }

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


    result_msg.success = true;
    result_msg.message = "Trajectory planned successfully";
    result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::SUCCESS;
    result_msg.trajectory = trajectory;
    trajectory_pub_->publish(result_msg);
}

void DynamicPlanner::plan(const geometry_msgs::msg::Pose& goal_pose, const std::string& ee_link, const std::string& frame)
{
    plan(goal_pose, ee_link, frame, kinematic_state_);
}

void DynamicPlanner::plan(const geometry_msgs::msg::Pose& goal_pose, const std::string& ee_link)
{
    plan(goal_pose, ee_link, params_.world_frame, kinematic_state_);
}

void DynamicPlanner::plan(const geometry_msgs::msg::Pose& goal_pose)
{
    plan(goal_pose, params_.end_effector_link, params_.world_frame, kinematic_state_);
}

void DynamicPlanner::cartesianPlan(const std::vector<geometry_msgs::msg::Pose>& waypoints, const std::string& ee_link, const std::string& frame)
{
    /*
    Plan: cartesian goal
        Args:
            waypoints: Array of target positions to follow
            ee_link: End effector link
            frame: Reference frame
        Returns:
            fraction: Fraction of the trajectory that was planned
    */

    stop(); // Stop any ongoing trajectory execution
    
    manipulator_interfaces::msg::TrajectoryResult result_msg;
    
    //Create the motion sequence request
    moveit_msgs::msg::MotionSequenceRequest motion_sequence_request;

    for (size_t i {0}; i < waypoints.size(); i++)
    {
        geometry_msgs::msg::Pose wp = waypoints[i]; 
        moveit_msgs::msg::MotionSequenceItem item;
        item.req = createMotionPlanRequest("pilz_industrial_motion_planner", "LIN");

        if (i != 0) {
            item.blend_radius = params_.caresian_blend_radius;
            item.req.start_state = moveit_msgs::msg::RobotState(); //Start state is ignored for subsequent waypoints
        } else {
            moveit::core::robotStateToRobotStateMsg(*kinematic_state_, item.req.start_state);
        }
        item.req.goal_constraints.push_back(createTcpGoalConstraints(wp, ee_link, frame));
        motion_sequence_request.items.push_back(item);
    }

    moveit_msgs::msg::MotionSequenceResponse plan = computeMotionSequence(motion_sequence_request);

    // POST PLANNING CHECKS
    // 1. Planning succeded
    // 2. Time optimal trajectory generation succeded
    // 3. Trajectory respects the path constraints

    if (plan.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Planning failed with error code: %d", plan.error_code.val);
        moveit_msgs::msg::RobotTrajectory trajectory;
        result_msg.success = false;
        result_msg.message = "Planning failed";
        result_msg.trajectory = trajectory;
        result_msg.error_code = plan.error_code.val;

        setTrajectory(trajectory);
        trajectory_pub_->publish(result_msg);
        return;
    }

    moveit_msgs::msg::RobotTrajectory trajectory = plan.planned_trajectories.front();

//    trajectory_msgs::msg::JointTrajectoryPoint last_point = trajectory.joint_trajectory.points.back();
//    if(!checkPoseDiff(getFKine(last_point.positions, ee_link).pose, goal_pose)){
//        RCLCPP_ERROR(node_->get_logger(), "Trajectory end point does not match with the target pose");
//        moveit_msgs::msg::RobotTrajectory trajectory;
//        result_msg.success = false;
//        result_msg.message = "Trajectory end point does not match with the target pose";
//        result_msg.trajectory = trajectory;
//        result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::END_POINT_MISMATCH;
//
//        setTrajectory(trajectory);
//        trajectory_pub_->publish(result_msg);
//        return;
//    }

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

    result_msg.success = true;
    result_msg.message = "Trajectory planned successfully";
    result_msg.error_code = manipulator_interfaces::msg::TrajectoryResult::SUCCESS;
    result_msg.trajectory = trajectory;
    trajectory_pub_->publish(result_msg);

    return;
}

void DynamicPlanner::cartesianPlan(const std::vector<geometry_msgs::msg::Pose>& waypoints, const std::string& ee_link)
{
    cartesianPlan(waypoints, ee_link, params_.world_frame);
}

void DynamicPlanner::cartesianPlan(const std::vector<geometry_msgs::msg::Pose>& waypoints)
{
    cartesianPlan(waypoints, params_.end_effector_link, params_.world_frame);
}

void DynamicPlanner::moveRobot(const sensor_msgs::msg::JointState& joint_state)
{
    //Executes a single joint state
    joint_cmd_pub_->publish(joint_state);
}

void DynamicPlanner::moveRobot(const trajectory_msgs::msg::JointTrajectoryPoint& traj_pt)
{
    //Executes a single trajectory position

    sensor_msgs::msg::JointState joint_state;
    // Fill the name of the joints
    joint_state.name = joints_names_group_;
    joint_state.position = traj_pt.positions;
    joint_state.velocity = traj_pt.velocities;
    joint_state.effort = traj_pt.effort;
    moveRobot(joint_state);
}

void DynamicPlanner::executeTrajectory()
{
    if (isExecuting()) {
        RCLCPP_WARN(node_->get_logger(), "Robot is already moving");
        return;
    }

    //Asynchronous execution of the last planned trajectory
    if (robot_trajectory_.joint_trajectory.points.empty())
    {
        RCLCPP_ERROR(node_->get_logger(), "No trajectory to execute");
        return;
    }

    is_executing_.store(true); //Set moving state to true
}

void DynamicPlanner::executeTrajectory(moveit_msgs::msg::RobotTrajectory& robot_trajectory)
{
    setTrajectory(robot_trajectory);
    executeTrajectory();
}

bool DynamicPlanner::isExecuting()
{
    //Check if the robot is moving
    return is_executing_.load();
}

// Check if the planner has received group definition, so the dynamic planner can start working
bool DynamicPlanner::isReady() const
{
  // The following booleans are true when the three subs have read something from active pubs 
  return joints_group_received_.load();
}

void DynamicPlanner::stop()
{
    force_stop_.store(true);
}

planning_scene_monitor::LockedPlanningSceneRW DynamicPlanner::getPlanningScene()
{
    return planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_);
}

DynamicPlannerParams DynamicPlanner::getParams() const
{
    return params_;
}

void DynamicPlanner::setParams(const DynamicPlannerParams &params)
{
    params_ = params;
}

moveit::core::RobotStatePtr DynamicPlanner::getRobotState() const
{
    return kinematic_state_;
}

const moveit::core::JointModelGroup* DynamicPlanner::getJointModelGroup() const
{
    return kinematic_model_->getJointModelGroup(planning_group_);
}

// ------------------------------------- PATH CONSTRAINTS ------------------------------------

void DynamicPlanner::setPathConstraints(const moveit_msgs::msg::Constraints &constraints)
{
    path_constraints_ = constraints;

    //Visualize path constraints
    for (const auto &pc : path_constraints_.position_constraints)
    {
        for (size_t i {0}; i < pc.constraint_region.primitives.size(); i++)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = pc.header.frame_id;
            pose_stamped.pose = pc.constraint_region.primitive_poses[i];

            visualizePrimitive(
                pc.constraint_region.primitives[i],
                pose_stamped,
                std::vector<double>{0.26, 0.72, 0.25, 0.5},
                "PATH_CONSTRAINTS",
                marker_id_
            );
            marker_id_++;
        }
    }
    rviz_visual_tools_->trigger();
}

moveit_msgs::msg::Constraints DynamicPlanner::getPathConstraints() const
{
    return path_constraints_;
}

void DynamicPlanner::clearPathConstraints()
{
    path_constraints_ = moveit_msgs::msg::Constraints();

    rviz_visual_tools_->deleteAllMarkers("PATH_CONSTRAINTS");
}

bool DynamicPlanner::checkJointConstraints(const std::vector<double> &joint_positions)
{
    /*
    Checks if the joint positions respect the joint limits
    Args:
        joint_positions: Array of joint positions (radians)
    Returns:
        True if the joint positions respect the joint limits, False otherwise
    */

    moveit::core::RobotStatePtr robot_state = getRobotState();
    robot_state->setJointGroupPositions(planning_group_, joint_positions);

    moveit_msgs::msg::Constraints path_constraints = getPathConstraints();

    path_constraints.position_constraints.clear();
    path_constraints.orientation_constraints.clear();

    return getPlanningScene()->isStateConstrained(*robot_state, path_constraints) && robot_state->satisfiesBounds();
}

bool DynamicPlanner::checkPoseConstraints(const std::vector<double> &joint_positions)
{
    /*
    Checks if the pose respects the path constraints
    Args:
        pose: Pose to check
    Returns:
        True if the pose respects the path constraints, False otherwise
    */

    moveit::core::RobotStatePtr robot_state = getRobotState();
    robot_state->setJointGroupPositions(planning_group_, joint_positions);

    moveit_msgs::msg::Constraints path_constraints = getPathConstraints();

    path_constraints.joint_constraints.clear();
    path_constraints.orientation_constraints.clear();

    return getPlanningScene()->isStateConstrained(*robot_state, path_constraints);
}

// ------------------------------------- COLLISION OBJECTS -------------------------------------

void DynamicPlanner::processCollisionObject(const moveit_msgs::msg::CollisionObject &collision_object)
{
    collision_object_pub_->publish(collision_object);

    //Visualize collision object
    if (collision_object.operation == moveit_msgs::msg::CollisionObject::ADD)
    {
        for (size_t i {0}; i < collision_object.primitives.size(); i++)
        {
            geometry_msgs::msg::PoseStamped pose_stamped; 
            pose_stamped.header.frame_id = collision_object.header.frame_id;
            pose_stamped.pose = collision_object.primitive_poses[i];

            visualizePrimitive(
                collision_object.primitives[i],
                pose_stamped,
                std::vector<double>{0.92, 0.25, 0.2, 0.5},
                "COLLISION_OBJECTS_" + collision_object.id,
                marker_id_
            );
            marker_id_++;
        }
    }
    else if (collision_object.operation == moveit_msgs::msg::CollisionObject::REMOVE)
    {
        rviz_visual_tools_->deleteAllMarkers("COLLISION_OBJECTS_" + collision_object.id);
    }
    rviz_visual_tools_->trigger();
}

void DynamicPlanner::processAttachedCollisionObject(const moveit_msgs::msg::AttachedCollisionObject &attached_collision_object)
{
    attached_collision_object_pub_->publish(attached_collision_object);

    //Visualize attached collision object
    if (attached_collision_object.object.operation == moveit_msgs::msg::CollisionObject::ADD)
    {
        for (size_t i {0}; i < attached_collision_object.object.primitives.size(); i++)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = attached_collision_object.link_name;
            pose_stamped.pose = attached_collision_object.object.primitive_poses[i];

            visualizePrimitive(
                attached_collision_object.object.primitives[i],
                pose_stamped,
                std::vector<double>{0.47, 0.06, 0.41, 0.5},
                "ATTACHED_OBJECTS_" + attached_collision_object.object.id,
                marker_id_,
                true //Move with the link frame
            );
            marker_id_++;
        }
    }
    else if (attached_collision_object.object.operation == moveit_msgs::msg::CollisionObject::REMOVE)
    {
        rviz_visual_tools_->deleteAllMarkers("ATTACHED_OBJECTS_" + attached_collision_object.object.id);
    }
    rviz_visual_tools_->trigger();
}

void DynamicPlanner::setCollisionEnabled(const std::string& object_1, const std::string& object_2, bool enabled)
{
    planning_scene_monitor::LockedPlanningSceneRW scene = getPlanningScene();
    collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();
    acm.setEntry(object_1, object_2, !enabled);
    planning_scene_monitor_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
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
    robot_state->setJointGroupPositions(planning_group_, joint_positions);
    robot_state->update();

    const Eigen::Isometry3d &end_effector_pose = robot_state->getGlobalLinkTransform(end_effector_link);

    return toPoseStamped(end_effector_pose, params_.world_frame);
}

geometry_msgs::msg::PoseStamped DynamicPlanner::getFKine(const std::string &end_effector_link)
{
    if (!isReady()) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot compute forward kinematics, joint states not received yet");
        return geometry_msgs::msg::PoseStamped();
    }

    std::lock_guard<std::mutex> lock(joint_val_mutex_);

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

const Eigen::MatrixXd DynamicPlanner::getJacobian(const std::vector<double> &joint_positions, const std::string &end_effector_link)
{
    moveit::core::RobotStatePtr kinematic_state = getRobotState();
    kinematic_state->setJointGroupPositions(planning_group_, joint_positions);
    kinematic_state->update();

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

const Eigen::MatrixXd DynamicPlanner::getJacobian(const std::string &end_effector_link)
{
    if (!isReady()) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot compute jacobian, joint states not received yet");
        return Eigen::MatrixXd();
    }

    std::lock_guard<std::mutex> lock(joint_val_mutex_);

    return getJacobian(
        joints_values_group_,
        end_effector_link
    );
}

const Eigen::MatrixXd DynamicPlanner::getJacobian()
{
    return getJacobian(params_.end_effector_link);
}

const Eigen::MatrixXd DynamicPlanner::getPseudoInverseJacobian(const std::vector<double> &joint_positions, const std::string &end_effector_link)
{
    return getJacobian(joint_positions, end_effector_link).completeOrthogonalDecomposition().pseudoInverse();
}

const Eigen::MatrixXd DynamicPlanner::getPseudoInverseJacobian(const std::string &end_effector_link)
{
    if (!isReady()) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot compute pseudo-inverse jacobian, joint states not received yet");
        return Eigen::MatrixXd();
    }

    std::lock_guard<std::mutex> lock(joint_val_mutex_);

    return getPseudoInverseJacobian(
        joints_values_group_,
        end_effector_link
    );
}

const Eigen::MatrixXd DynamicPlanner::getPseudoInverseJacobian()
{
    return getPseudoInverseJacobian(params_.end_effector_link);
}

// ------------------------------------- JOINT STATE -------------------------------------

std::vector<double> DynamicPlanner::getJointValues()
{
    if (!isReady()) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot get current joint values, joint states not received yet");
        return std::vector<double>();
    }

    std::lock_guard<std::mutex> lock(joint_val_mutex_);

    return joints_values_group_;
}

std::vector<double> DynamicPlanner::getJointSpeeds()
{
    if (!isReady()) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot get current joint speeds, joint states not received yet");
        return std::vector<double>();
    }

    std::lock_guard<std::mutex> lock(joint_speed_mutex_);

    return joints_speed_group_;
}

std::vector<std::string> DynamicPlanner::getJointNames()
{
    return joints_names_group_;
}

sensor_msgs::msg::JointState DynamicPlanner::getJointState()
{
    if (!isReady()) {
        RCLCPP_ERROR(node_->get_logger(), "Cannot get current joint state, joint states not received yet");
        return sensor_msgs::msg::JointState();
    }

    std::lock_guard<std::mutex> lock_val(joint_val_mutex_);
    std::lock_guard<std::mutex> lock_speed(joint_speed_mutex_);

    sensor_msgs::msg::JointState joint_state;
    joint_state.name = joints_names_group_;
    joint_state.position = joints_values_group_;
    joint_state.velocity = joints_speed_group_;

    return joint_state;
}

// -------------------------------------------------------------------------------------------
// ------------------------------------- PRIVATE METHODS -------------------------------------
// -------------------------------------------------------------------------------------------


// ------------------------------------- CALLBACK METHODS -------------------------------------

void DynamicPlanner::jointsState_callback(const sensor_msgs::msg::JointState::SharedPtr &joint_state)
{
    // Lock joint state mutexes
    std::lock_guard<std::mutex> lock_val(joint_val_mutex_);
    std::lock_guard<std::mutex> lock_speed(joint_speed_mutex_);

    for (size_t i {0}; i < joint_state->name.size(); i++)
    {
        auto it = std::find(joints_names_group_.begin(), joints_names_group_.end(), joint_state->name[i]);
        if (it != joints_names_group_.end())
        {
            size_t index = std::distance(joints_names_group_.begin(), it);
            joints_values_group_[index] = joint_state->position[i];
            if (!joint_state->velocity.empty()){
                joints_speed_group_[index] = joint_state->velocity[i];
            }
        }
    }

    last_joint_state_time_.store(node_->now().seconds());
    
    //kinematic_state_->setJointGroupPositions(planning_group_, joints_values_group_);
    //kinematic_state_->update();

    if (!isReady()) {
        RCLCPP_INFO(node_->get_logger(), "Joint states for planning group %s received.", planning_group_.c_str());
        joints_group_received_.store(true);
    }
}

void DynamicPlanner::trajectoryExecution_callback(){
    /*
    Callback for trajectory execution from traj_timer_
    */

    if (rclcpp::ok() == false || !isReady()){
        return;
    }

    //Check that joint states are being received
    if (node_->now().seconds() - last_joint_state_time_.load() > params_.joint_states_timeout){
        RCLCPP_ERROR(node_->get_logger(), "No joint states received for more than %.2f seconds, blocking planner.", params_.joint_states_timeout);
        is_executing_.store(false);
        force_stop_.store(false); // Reset force stop flag
        trajpoint_ = 0UL; // Reset trajectory point index
        joints_group_received_.store(false);
    }


    if(force_stop_.load()){
        std::lock_guard<std::mutex> joint_val_lock(joint_val_mutex_);

        RCLCPP_INFO(node_->get_logger(), "Force stop received, stopping trajectory execution.");
        is_executing_.store(false);
        force_stop_.store(false); // Reset force stop flag
        trajpoint_ = 0UL; // Reset trajectory point index
        
        //Construct a joint state message with the final joint positions
        sensor_msgs::msg::JointState joint_state;
        joint_state.name = joints_names_group_;
        joint_state.position = joints_values_group_;
        joint_state.velocity = std::vector<double>(joints_names_group_.size(), 0.0);
        moveRobot(joint_state);

        return;
    }

    if (isExecuting()){
        std::lock_guard<std::mutex> joint_val_lock(joint_val_mutex_);

        trajectory_msgs::msg::JointTrajectoryPoint traj_pt;
        if (trajpoint_ < robot_trajectory_.joint_trajectory.points.size())
        {
            traj_pt = robot_trajectory_.joint_trajectory.points[trajpoint_];
            moveRobot(traj_pt);
            trajpoint_++;
        }
        else if (trajpoint_ == robot_trajectory_.joint_trajectory.points.size())
        {
            // If the trajectory is finished, stop the execution
            RCLCPP_INFO(node_->get_logger(), "Trajectory executed.");
            is_executing_.store(false);
            trajpoint_ = 0UL; // Reset trajectory point index
        }
    }
    
}


void DynamicPlanner::executionControl_callback(const std_msgs::msg::Bool::SharedPtr& msg)
{
    //If true move the robot, otherwise stop
    if (msg->data)
    {
        RCLCPP_INFO(node_->get_logger(), "Execution control: start");
        executeTrajectory();
    } else {
        RCLCPP_INFO(node_->get_logger(), "Execution control: stop");
        stop();
    }
}

// ------------------------------------- TRAJECTORY METHODS -------------------------------------

void DynamicPlanner::setTrajectory(const moveit_msgs::msg::RobotTrajectory &trajectory)
{
    /*
    Set the planned trajectory as well as other information used for dynamic planning
    Args:
        trajectory: The trajectory
    */

    if (isExecuting()){
        // Memory safety check
        RCLCPP_ERROR(node_->get_logger(), "Cannot set trajectory while robot is moving");
        throw std::runtime_error("Cannot set trajectory while robot is moving");
    }


    robot_trajectory_ = trajectory;
    if (trajectory.joint_trajectory.points.empty())
    {
        RCLCPP_WARN(node_->get_logger(), "Empty trajectory");
        return;
    }
    trajpoint_ = 0UL;
}

bool DynamicPlanner::processTrajectory(moveit_msgs::msg::RobotTrajectory &trajectory_msg) {
    robot_trajectory::RobotTrajectory robot_trajectory(
        kinematic_model_,
        planning_group_
    );

    robot_trajectory.setRobotTrajectoryMsg(*getRobotState(), trajectory_msg.joint_trajectory);

    bool success = time_optimal_traj_gen_->computeTimeStamps(
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
        if (!getPlanningScene()->isStateConstrained(robot_state, constraints))
        {
            return false;
        }
    }

    return true;
}

// ------------------------------------- PLANNING METHODS -------------------------------------

moveit_msgs::msg::MotionPlanRequest DynamicPlanner::createMotionPlanRequest(const std::string &planning_pipeline, const std::string &planner_id)
{
    moveit_msgs::msg::MotionPlanRequest req;

    req.group_name = planning_group_;
    req.planner_id = planner_id;
    req.pipeline_id = planning_pipeline;
    req.num_planning_attempts = params_.num_attempts;
    req.allowed_planning_time = params_.planning_time;
    req.max_velocity_scaling_factor = params_.vel_factor;
    req.max_acceleration_scaling_factor = params_.acc_factor;

    req.path_constraints = getPathConstraints();

    return req;
}

moveit_msgs::msg::MotionPlanRequest DynamicPlanner::createMotionPlanRequest()
{
    return createMotionPlanRequest(params_.planning_pipeline, params_.planner_id);
}

moveit_msgs::msg::Constraints DynamicPlanner::createJointGoalConstraints(const std::vector<double> &joint_positions)
{
    moveit::core::RobotState robot_state = *getRobotState();
    robot_state.setJointGroupPositions(planning_group_, joint_positions);
    robot_state.update();

    return kinematic_constraints::constructGoalConstraints(robot_state, getJointModelGroup(), params_.joint_tolerance);
}

moveit_msgs::msg::Constraints DynamicPlanner::createTcpGoalConstraints(const geometry_msgs::msg::Pose &pose, const std::string &end_effector_link, const std::string &frame)
{
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = node_->now();
    pose_stamped.header.frame_id = params_.world_frame;

    if (frame != params_.world_frame){
        pose_stamped.pose = transformPoseToWorld(pose, frame);
    } else {
        pose_stamped.pose = pose;
    }

    return kinematic_constraints::constructGoalConstraints(end_effector_link, pose_stamped, params_.position_tolerance, params_.orientation_tolerance);
}

moveit_msgs::action::MoveGroup::Result DynamicPlanner::computeMotionPlan(const moveit_msgs::msg::MotionPlanRequest &motion_plan_request)
{
    auto req = moveit_msgs::action::MoveGroup::Goal();
    req.request = motion_plan_request;
    req.planning_options.plan_only = true;
    auto res = std::make_shared<moveit_msgs::action::MoveGroup::Result>();

    //Check that service is available
    if (!plan_action_client_->wait_for_action_server(std::chrono::seconds(1))){
        RCLCPP_ERROR(node_->get_logger(), "Motion plan action service not available");
        res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
        return *res;
    }

    if (!plan_action_client_->action_server_is_ready()){
        RCLCPP_ERROR(node_->get_logger(), "Motion plan action server not ready.");
        res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
        return *res;
    }

    //Check that planner is ready
    if (!isReady()){
        RCLCPP_ERROR(node_->get_logger(), "Unable to compute motion plan: joints group not received.");
        res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
        return *res;
    }
    
    //Send the request
    bool done = false;
    auto goal_opts = rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SendGoalOptions();

    goal_opts.goal_response_callback = [&](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::SharedPtr& goal_handle) {
        if (!goal_handle) {
            done = true;
            RCLCPP_INFO(node_->get_logger(), "Planning request rejected");
        }
        else {
            RCLCPP_INFO(node_->get_logger(), "Planning request accepted");
        }
    };
    goal_opts.result_callback = [&](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::MoveGroup>::WrappedResult& result) {
        RCLCPP_INFO(node_->get_logger(), "Motion plan completed in %f seconds", result.result->planning_time);
        res = result.result;
        done = true;
    };
    goal_opts.feedback_callback = [](auto, auto) {};

    plan_action_client_->async_send_goal(req, goal_opts);
    
    auto start_time = node_->now();

    while (!done && rclcpp::ok()){
        std::this_thread::sleep_for(std::chrono::milliseconds((int)params_.sample_time * 1000));
        if ((node_->now() - start_time).seconds() > params_.planning_time + 1.0){ //Add a second of tolerance
            break;
        }
    }

    if (!done){
        RCLCPP_ERROR(node_->get_logger(), "Motion plan service call timed out");
        res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    }

    return *res;
}

moveit_msgs::msg::MotionSequenceResponse DynamicPlanner::computeMotionSequence(const moveit_msgs::msg::MotionSequenceRequest &motion_sequence_request)
{
    auto req = std::make_shared<moveit_msgs::srv::GetMotionSequence::Request>();
    req->request = motion_sequence_request;
    auto res = std::make_shared<moveit_msgs::msg::MotionSequenceResponse>();

    //Check that service is available
    if (!cartesian_motion_sequence_client_->wait_for_service(std::chrono::seconds(1))){
        RCLCPP_ERROR(node_->get_logger(), "Motion sequence service not available");
        res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
        return *res;
    }

    //Check that planner is ready
    if (!isReady()){
        RCLCPP_ERROR(node_->get_logger(), "Unable to compute motion sequence: joints group not received.");
        res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
        return *res;
    }
    
    //Send the request
    bool done = false;

    auto cb = [&](rclcpp::Client<moveit_msgs::srv::GetMotionSequence>::SharedFuture future) {
        RCLCPP_INFO(node_->get_logger(), "Motion sequence completed in %f seconds", future.get()->response.planning_time);
        res = std::make_shared<moveit_msgs::msg::MotionSequenceResponse>(future.get()->response);
        done = true;
    };

    cartesian_motion_sequence_client_->async_send_request(req, cb);
    
    auto start_time = node_->now();

    while (!done && rclcpp::ok()){
        std::this_thread::sleep_for(std::chrono::milliseconds((int)params_.sample_time * 1000));
        if ((node_->now() - start_time).seconds() > params_.planning_time + 1.0){ //Add a second of tolerance
            break;
        }
    }

    if (!done){
        RCLCPP_ERROR(node_->get_logger(), "Motion plan service call timed out");
        res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    }

    return *res;
}

// ------------------------------------- HELPER METHODS -------------------------------------

bool DynamicPlanner::checkJointDiff(const std::vector<double>& joint_positions)
{
    if (!isReady()){
        RCLCPP_ERROR(node_->get_logger(), "Unable to check joint difference: joints group not received.");
        return false;
    }

    std::lock_guard<std::mutex> lock(joint_val_mutex_);

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
    double angle_th = params_.orientation_tolerance;
    //double rot_th = params_.orientation_tolerance;

    bool check = true;

    check = ((abs(pose_a.position.x - pose_b.position.x) < pos_th) && check);
    check = ((abs(pose_a.position.y - pose_b.position.y) < pos_th) && check);
    check = ((abs(pose_a.position.z - pose_b.position.z) < pos_th) && check);

    //Compute angular distance between the two orientations
    // Convert geometry_msgs::msg::Quaternion to Eigen::Quaterniond
    Eigen::Quaterniond quat1(pose_a.orientation.w, pose_a.orientation.x, pose_a.orientation.y, pose_a.orientation.z);
    Eigen::Quaterniond quat2(pose_b.orientation.w, pose_b.orientation.x, pose_b.orientation.y, pose_b.orientation.z);

    // Compute the relative rotation
    Eigen::Quaterniond relative_rotation = quat1.inverse() * quat2;

    // Extract the angle of rotation
    double angular_distance = 2 * std::acos(relative_rotation.w());

    // Normalize the angle to the range [0, pi]
    if (angular_distance > M_PI)
    {
        angular_distance = 2 * M_PI - angular_distance;
    }

    check = ((angular_distance < angle_th) && check);

    return check;
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

// ------------------------------------- TF2 METHODS -------------------------------------

geometry_msgs::msg::TransformStamped DynamicPlanner::getTransform(const std::string &target_frame, const std::string &source_frame)
{
    /*
    Looks up the transform between two frames
    Args:
        target_frame: The target frame
        source_frame: The source frame
    Returns:
        The transform between the two frames
    */
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_->lookupTransform(
            target_frame,
            source_frame,
            tf2::TimePointZero,
            tf2::durationFromSec(0.5)
        );
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(node_->get_logger(), "Could not get transform from %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
    }
    return transform_stamped;
}

geometry_msgs::msg::Pose DynamicPlanner::transformPose(const geometry_msgs::msg::Pose &pose, const std::string &target_frame, const std::string &source_frame)
{
    /*
    Transforms a pose from one frame to another
    Args:
        pose: The pose to transform
        target_frame: The target frame
        source_frame: The source frame
    Returns:
        The transformed pose
    */
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = source_frame;
    pose_stamped.header.stamp = node_->now();
    pose_stamped.pose = pose;

    geometry_msgs::msg::PoseStamped transformed_pose_stamped;
    try {
        transformed_pose_stamped = tf_buffer_->transform(
            pose_stamped,
            target_frame,
            tf2::durationFromSec(params_.tf_timeout)
        );
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(node_->get_logger(), "Could not transform pose from %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
    }
    return transformed_pose_stamped.pose;
}

geometry_msgs::msg::Pose DynamicPlanner::transformPoseToWorld(const geometry_msgs::msg::Pose &pose, const std::string &src_frame)
{
    return transformPose(pose, params_.world_frame, src_frame);
}

// ------------------------------------- VISUALIZATION METHODS -------------------------------------

void DynamicPlanner::visualizePrimitive(const shape_msgs::msg::SolidPrimitive &primitive, 
                                        const geometry_msgs::msg::PoseStamped &primitive_pose,
                                        const std::vector<double> rgba_color,
                                        const std::string &ns,
                                        const int &id,
                                        const bool frame_locked)
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
    marker.header.frame_id = primitive_pose.header.frame_id;
    marker.header.stamp = rclcpp::Time(0);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = ns;
    marker.pose = primitive_pose.pose;
    marker.color = color;
    marker.id = id;
    marker.frame_locked = frame_locked;

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
            marker.scale.x = primitive.dimensions[0] * 2;
            marker.scale.y = primitive.dimensions[0] * 2;
            marker.scale.z = primitive.dimensions[0] * 2;
            break;
        }
        case shape_msgs::msg::SolidPrimitive::CYLINDER:
        {
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.scale.x = primitive.dimensions[1] * 2;
            marker.scale.y = primitive.dimensions[1] * 2;
            marker.scale.z = primitive.dimensions[0];
            break;
        }
        case shape_msgs::msg::SolidPrimitive::CONE:
        {
            marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
            marker.mesh_resource = "package://manipulators/models/meshes/Cone.stl";
            marker.scale.x = primitive.dimensions[1] * 2;
            marker.scale.y = primitive.dimensions[1] * 2;
            marker.scale.z = primitive.dimensions[0];
            break;
        }
        default:
        {
            RCLCPP_ERROR(node_->get_logger(), "Unknown primitive type");
            break;
        }
    }

    if (frame_locked)
    {
        rviz_visual_tools_->enableFrameLocking(true);
        rviz_visual_tools_->publishMarker(marker);
        rviz_visual_tools_->enableFrameLocking(false);
    }
    else
    {
        rviz_visual_tools_->publishMarker(marker);
    }
}

DynamicPlannerParams DynamicPlannerParams::fromNode(const rclcpp::Node::SharedPtr &node)
{
    DynamicPlannerParams params;

    params.planning_pipeline        = node->get_parameter_or("planning_pipeline", params.planning_pipeline);
    params.planner_id               = node->get_parameter_or("planner_id", params.planner_id);
    params.num_attempts             = node->get_parameter_or("num_attempts", params.num_attempts);
    params.planning_time            = node->get_parameter_or("planning_time", params.planning_time);
    params.vel_factor               = node->get_parameter_or("vel_factor", params.vel_factor);
    params.acc_factor               = node->get_parameter_or("acc_factor", params.acc_factor);
    params.sample_time              = node->get_parameter_or("sample_time", params.sample_time);
    params.max_velocity             = node->get_parameter_or("max_velocity", params.max_velocity);
    params.position_tolerance       = node->get_parameter_or("position_tolerance", params.position_tolerance);
    params.orientation_tolerance    = node->get_parameter_or("orientation_tolerance", params.orientation_tolerance);
    params.joint_tolerance          = node->get_parameter_or("joint_tolerance", params.joint_tolerance);
    params.world_frame              = node->get_parameter_or("world_frame", params.world_frame);
    params.end_effector_link        = node->get_parameter_or("ee_name", params.end_effector_link);
    params.min_cartesian_fraction   = node->get_parameter_or("min_cartesian_fraction", params.min_cartesian_fraction);
    params.joint_states_timeout     = node->get_parameter_or("joint_states_timeout", params.joint_states_timeout);
    params.tf_timeout               = node->get_parameter_or("tf_timeout", params.tf_timeout);

    return params;
}
