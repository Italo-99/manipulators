// IMPORT LIBRARIES
#include "manipulators/ManipulatorMenu.h"

// --------------------- PUBLIC CONSTRUCTOR ---------------------

ManipulatorMenu::ManipulatorMenu(ManipulatorMenuParams params, const rclcpp::Node::SharedPtr& node, const bool sync_parameters) 
    : params_(params), node_(node), traj_result_(), traj_received_(false)
{
    // Display Manipulator
    RCLCPP_INFO(node_->get_logger(), "Manipulator menu initialized with the following setup:");
    RCLCPP_INFO(node_->get_logger(), "Manipulator name: %s", params_.manipulator_name.c_str());

    // ---------------------- Params sync ----------------------

    getManipulatorParams_client_ = std::make_shared<rclcpp::SyncParametersClient>(node_, "/manipulator_planner");

    if (sync_parameters){
        waitManipulatorParameters();
        RCLCPP_INFO(node_->get_logger(), "Manipulator parameters synchronized automatically with planning group: %s", params_.planning_group.c_str());
    }

    for (unsigned long k = 0; k < params_.joint_names.size(); k++)
    {
        RCLCPP_DEBUG(node_->get_logger(), "Joint %ld name: %s", k, params_.joint_names[k].c_str());
    }

    // Init arrays
    for (const std::string& name : params_.joint_names) {
        joints_map_group_[name] = 0.;
    }

    joints_values_group_.resize(params_.joint_names.size());
    current_joint_pose_.name      = params_.joint_names;

    // --------------------- PUBS & SUBS DELCARATIONS ---------------------
    jointGoal_pub_               = node_->create_publisher<manipulator_interfaces::msg::JointGoal>(params_.manipulator_name+"/joint_goal", 1);
    tcpGoal_pub_                 = node_->create_publisher<manipulator_interfaces::msg::TcpGoal>(params_.manipulator_name+"/tcp_goal", 1);
    cartesianPlan_pub_           = node_->create_publisher<manipulator_interfaces::msg::CartesianGoal>(params_.manipulator_name+"/cartesian_plan", 1);
    collisionObject_pub_         = node_->create_publisher<moveit_msgs::msg::CollisionObject>(params_.manipulator_name+"/collision_object", 1);
    attachedCollisionObject_pub_ = node_->create_publisher<moveit_msgs::msg::AttachedCollisionObject>(params_.manipulator_name+"/attached_collision_object", 1);
    jointConstraints_pub_        = node_->create_publisher<moveit_msgs::msg::JointConstraint>(params_.manipulator_name+"/joint_constraint", 1);
    positionConstraints_pub_     = node_->create_publisher<moveit_msgs::msg::PositionConstraint>(params_.manipulator_name+"/position_constraint", 1);
    orientationConstraints_pub_  = node_->create_publisher<moveit_msgs::msg::OrientationConstraint>(params_.manipulator_name+"/orientation_constraint", 1);
    clearConstraints_pub_        = node_->create_publisher<std_msgs::msg::Empty>(params_.manipulator_name+"/clear_constraints", 1);

    jointState_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 1, 
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->jointStateCallback(msg);
        }
    );

    eePose_sub_ = node_->create_subscription<geometry_msgs::msg::Pose>(
        params_.manipulator_name+"/tcp_pose", 1,
        [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
            this->current_tcp_pose_ = *msg;
        }
    );

    // --------------------- Kinematics client init ---------------------
    invKine_client_                      = node_->create_client<manipulator_interfaces::srv::InvKine>(params_.manipulator_name+"/get_invkine");
    pseudoInverse_client_                = node_->create_client<manipulator_interfaces::srv::PseudoInverse>(params_.manipulator_name+"/get_pseudo_inverse");
    fKine_client_                        = node_->create_client<manipulator_interfaces::srv::FKine>(params_.manipulator_name+"/get_fkine");
    jacobian_client_                     = node_->create_client<manipulator_interfaces::srv::Jacobian>(params_.manipulator_name+"/get_jacobian");

    setJacobianControl_client_           = node_->create_client<std_srvs::srv::SetBool>(params_.manipulator_name+"/jacobian_control_setter");
    setRealTimeControl_client_           = node_->create_client<std_srvs::srv::SetBool>(params_.manipulator_name+"/joints_real_time_setter");
    enableRealTimeConstraints_client_    = node_->create_client<manipulator_interfaces::srv::EnableRealTimeConstraints>(params_.manipulator_name+"/enable_real_time_constraints");
    changePlannerScalingFactors_client_  = node_->create_client<manipulator_interfaces::srv::ChangePlannerScalingFactors>(params_.manipulator_name+"/change_planner_scaling_factors");
    changePlannerTolerances_client_      = node_->create_client<manipulator_interfaces::srv::ChangePlannerTolerances>(params_.manipulator_name+"/change_planner_tolerances");

    // --------------------- Admittace control clients ---------------------
    setAdmittanceControl_client_         = node_->create_client<std_srvs::srv::SetBool>(params_.manipulator_name+"/enable_admittance");
    setAdmittanceVelMode_client_         = node_->create_client<std_srvs::srv::SetBool>(params_.manipulator_name+"/admittance_vel_mode");

    // ---------------------- TF ----------------------
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---------------------- Planning ----------------------
    
    auto cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group;

    plannedTrajectory_sub_ = node_->create_subscription<manipulator_interfaces::msg::TrajectoryResult>(
        params_.planning_group+"/planned_trajectory", 1,
        [this](const manipulator_interfaces::msg::TrajectoryResult::SharedPtr msg) {
            trajectoryCallback(msg);
        },
        sub_options
    );

    trajectory_pub_ = node_->create_publisher<moveit_msgs::msg::RobotTrajectory>(params_.planning_group+"/trajectory", 1);
    executionControl_pub_ = node_->create_publisher<std_msgs::msg::Bool>(params_.planning_group+"/execution_control", 1);

    // ---------------------- Gripper ----------------------

    if(params_.gripper == "robotiq_85"){
        gripperMove_client_ = node_->create_client<std_srvs::srv::SetBool>(params_.gripper_group+"/move_gripper");
    } else if (params_.gripper == "toolIO"){
        toolDigitalIO_pub_ = node_->create_publisher<std_msgs::msg::Int8>("/ur_rtde/tool_digitalIO/command", 1);
    }

    if(!params_.known_poses_path.empty()){
        loadKnownPoses();
    }

    rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
        std::bind(&ManipulatorMenu::shutdown_handler, this) // Register shutdown handler
    );

}

ManipulatorMenu::~ManipulatorMenu()
{
    delete menu_;
}

/*
    ================================================================
    ====================== PUBLIC FUNCTIONS ========================
    ================================================================
*/

std::vector<double> ManipulatorMenu::invKineClient(const geometry_msgs::msg::Pose pose)
{
    std::vector<double> joint_values;

    // Set target pose
    auto request = std::make_shared<manipulator_interfaces::srv::InvKine::Request>();
    request->target_pose = pose;

    // Wait for the service to be available
    size_t num_tries {0};
    while (!invKine_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if(num_tries > clients_wait_timeout_) {
            RCLCPP_ERROR(node_->get_logger(), "Unable to connect to invKine service");
            return {};
        }
        num_tries++;

        if (!rclcpp::ok()){
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return joint_values;
        }
    }

    // Send the request asynchronously
    auto response_future = invKine_client_->async_send_request(request);

    // Wait until the future is completed
    if (response_future.wait_for(std::chrono::seconds(clients_wait_timeout_)) != std::future_status::ready)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call invKine service");
        return joint_values;
    }

    // If the service call was successful, process the response
    auto response = response_future.get();
    for (unsigned long k = 0; k < response->joint_values.size(); ++k)
    {
        joint_values.push_back(response->joint_values[k]);
    }

    return joint_values;
}

Eigen::MatrixXd ManipulatorMenu::pseudoInverseClient()
{
    Eigen::MatrixXd matrix(params_.joint_names.size(), 6);

    auto request = std::make_shared<manipulator_interfaces::srv::PseudoInverse::Request>();

    // Wait for the service to be available
    size_t num_tries {0};
    while (!pseudoInverse_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if(num_tries > clients_wait_timeout_) {
            RCLCPP_ERROR(node_->get_logger(), "Unable to connect to pseudoInverse service");
            return {};
        }
        num_tries++;

        if (!rclcpp::ok()){
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return matrix;
        }
    }

    // Send the request asynchronously
    auto response_future = pseudoInverse_client_->async_send_request(request);

    // Wait until the future is completed
    if (response_future.wait_for(std::chrono::seconds(clients_wait_timeout_)) != std::future_status::ready)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call pseudoInverse service");
        return matrix;
    }

    // If the service call was successful, process the response
    auto response = response_future.get();
    listToMatrix(response->matrix_values, matrix);

    return matrix;
}

geometry_msgs::msg::Pose ManipulatorMenu::getFKineClient(const sensor_msgs::msg::JointState joint_positions)
{
    geometry_msgs::msg::Pose pose;

    auto request = std::make_shared<manipulator_interfaces::srv::FKine::Request>();
    request->joint_state = joint_positions;

    // Wait for the service to be available
    size_t num_tries {0};
    while (!fKine_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if(num_tries > clients_wait_timeout_) {
            RCLCPP_ERROR(node_->get_logger(), "Unable to connect to fKine service");
            return geometry_msgs::msg::Pose();
        }
        num_tries++;

        if (!rclcpp::ok()){
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return pose;
        }
    }

    // Send the request asynchronously
    auto response_future = fKine_client_->async_send_request(request);

    // Wait until the future is completed
    if (response_future.wait_for(std::chrono::seconds(clients_wait_timeout_)) != std::future_status::ready)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call getFKine service");
        return pose;
    }

    // If the service call was successful, process the response
    auto response = response_future.get();
    pose = response->tcp_pose.pose;

    return pose;
}

Eigen::MatrixXd ManipulatorMenu::getJacobianClient()
{
    Eigen::MatrixXd matrix(params_.joint_names.size(), 6);

    auto request = std::make_shared<manipulator_interfaces::srv::Jacobian::Request>();

    // Wait for the service to be available
    size_t num_tries {0};
    while (!jacobian_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if(num_tries > clients_wait_timeout_) {
            RCLCPP_ERROR(node_->get_logger(), "Unable to connect to jacobian service");
            return {};
        }
        num_tries++;

        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return matrix;
        }
    }

    // Send the request asynchronously and get the response
    auto response_future = jacobian_client_->async_send_request(request);

    // Wait until the future is completed
    if (response_future.wait_for(std::chrono::seconds(clients_wait_timeout_)) != std::future_status::ready)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call Jacobian service");
        return matrix;
    }

    // If we reached this point, the response is valid, so process it
    auto response = response_future.get();

    // Assign data from Float64[] to Eigen::MatrixXd
    listToMatrix(response->matrix_values, matrix);

    return matrix;
}

// -------------------- GRIPPERS ---------------------

bool ManipulatorMenu::gripperMoveClient(const bool close){

    if(params_.gripper != "robotiq_85"){
        RCLCPP_ERROR(node_->get_logger(), "robotiq_85 gripper is not available in this manipulator.");
        return false;
    }

    std_srvs::srv::SetBool::Request::SharedPtr request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = close;

    // Wait for the service to be available
    size_t num_tries {0};
    while (!gripperMove_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if(num_tries > clients_wait_timeout_) {
            RCLCPP_ERROR(node_->get_logger(), "Unable to connect to gripper service");
            return {};
        }
        num_tries++;

        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return false;
        }
    }

    // Send the request asynchronously and get the response
    auto response_future = gripperMove_client_->async_send_request(request);

    // Wait until the future is completed
    if (response_future.wait_for(std::chrono::seconds(clients_wait_timeout_)) != std::future_status::ready)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call gripperMove service");
        return false;
    }

    // If we reached this point, the response is valid, so process it
    auto response = response_future.get();

    return response->success;
}

// -------------------- PARAMETERS ---------------------

template <typename T>
T ManipulatorMenu::getManipulatorParameter(const std::string& param_name)
{
    // Check if the parameter client is initialized
    if (!getManipulatorParams_client_)
    {
        RCLCPP_ERROR(node_->get_logger(), "Parameter client is not initialized.");
        return T{};
    }

    // Check if the parameter exists
    if (!getManipulatorParams_client_->has_parameter(param_name))
    {
        RCLCPP_ERROR(node_->get_logger(), "Parameter %s does not exist in planner node.", param_name.c_str());
        return T{};
    }

    // Get the parameter
    return getManipulatorParams_client_->get_parameter<T>(param_name);
}


// --------------------- ROS HANDLER ---------------------

void ManipulatorMenu::spinnerMenu()
{
    // Setup a rate for ROS loop execution
    rclcpp::Rate r(params_.ros_freq);
    initializeMenu();

    std::thread spinner_thread = std::thread([this] {
        spinner();
    });

    int choice = -1;

    // ROS loop
    while (rclcpp::ok() && choice != 0)
    {
        // Wait for next loop time
        rclcpp::sleep_for(std::chrono::milliseconds(100));

        // Display the user menu and process user choices
        menu_->printMenu();
        choice = menu_->getUserChoice();
        RCLCPP_INFO(node_->get_logger(), "User choice: %d", choice);
        menu_->processChoice(choice);
    }

    // Shutdown ROS if Ctrl+C or Ctrl+D are pressed
    rclcpp::shutdown();
}

// Asynchronous spinner for ROS routines without user menu
void ManipulatorMenu::spinner()
{
    // Setup a rate for ROS loop execution
    executor_.add_node(node_);
    executor_.spin();

    // Shutdown ROS if Ctrl+C or Ctrl+D are pressed
    rclcpp::shutdown();
}

// --------------------- MOVEMENTS HANDLER ---------------------

// Publish a joint goal by passing a vector of joints in deg
sensor_msgs::msg::JointState ManipulatorMenu::publishJointGoal(const std::vector<double> joint_goal, const std::vector<double> start_state, const bool execute)
{
    // Fill the joint msg with degToRad conversion
    sensor_msgs::msg::JointState joint_state = joint_state_from_vector(joint_goal);
    return publishJointGoal(joint_state, start_state, execute);
}

// Publish a joint goal by passing a JointState msg
sensor_msgs::msg::JointState ManipulatorMenu::publishJointGoal(const sensor_msgs::msg::JointState joint_goal, const std::vector<double> start_state, const bool execute)
{
    manipulator_interfaces::msg::JointGoal joint_goal_msg;
    joint_goal_msg.start_state = joint_state_from_vector(start_state);
    joint_goal_msg.joint_goal = joint_goal;
    joint_goal_msg.execute = execute;
    // Publish the JointState message
    jointGoal_pub_->publish(joint_goal_msg);
    return joint_goal;
}

// Publish a Tcp goal by passing a vector (rotations must be expressed in deg)
geometry_msgs::msg::Pose ManipulatorMenu::publishTcpGoal(
    const std::vector<double> position, 
    const std::vector<double> start_state, 
    const std::string& frame, 
    const bool execute)
{
    geometry_msgs::msg::Pose tcp_pose = pose_from_vector(position);

    return publishTcpGoal(tcp_pose, start_state, frame, execute);
}

// Publish a Tcp goal by passing a geometry_msgs::msg::Pose
geometry_msgs::msg::Pose ManipulatorMenu::publishTcpGoal(
    const geometry_msgs::msg::Pose tcp_pose, 
    const std::vector<double> start_state,
    const std::string& frame, 
    const bool execute)
{
    manipulator_interfaces::msg::TcpGoal tcp_goal_msg;
    tcp_goal_msg.target_pose = tcp_pose;
    tcp_goal_msg.start_state =  joint_state_from_vector(start_state);
    tcp_goal_msg.execute = execute;
    tcp_goal_msg.end_effector = manipulator_interfaces::msg::TcpGoal::DEFAULT;

    if (!frame.empty())
    {
        tcp_goal_msg.frame = frame;
    }
    else
    {
        tcp_goal_msg.frame = manipulator_interfaces::msg::TcpGoal::DEFAULT;
    }

    tcpGoal_pub_->publish(tcp_goal_msg);
    return tcp_pose;
}

sensor_msgs::msg::JointState ManipulatorMenu::oneJointMove(const int num, const double joint_rot)
{
    // Fill current joints pose as target
    std::vector<double> joint_target;
    for (unsigned long k = 0; k < params_.joint_names.size(); k++)
    {
        joint_target.push_back(current_joint_pose_.position[k] * 180 / M_PI);
    }
    // Change the joint target position
    joint_target[num] = joint_target[num] + joint_rot;
    return publishJointGoal(joint_target);
}

void ManipulatorMenu::publishCartesianGoal(
    const std::vector<geometry_msgs::msg::Pose> waypoints,
    const std::vector<double> start_state,
    const std::string& frame,
    const bool execute)
{
    manipulator_interfaces::msg::CartesianGoal cartesian_goal_msg;
    cartesian_goal_msg.end_effector = manipulator_interfaces::msg::CartesianGoal::DEFAULT;

    for (const auto& waypoint : waypoints)
    {
        cartesian_goal_msg.waypoints.poses.push_back(waypoint);
    }

    cartesian_goal_msg.execute = execute;
    cartesian_goal_msg.start_state = joint_state_from_vector(start_state);

    if (!frame.empty())
    {
        cartesian_goal_msg.frame = frame;
    }
    else
    {
        cartesian_goal_msg.frame = manipulator_interfaces::msg::TcpGoal::DEFAULT;
    }

    // Publish the Cartesian goal message
    cartesianPlan_pub_->publish(cartesian_goal_msg);
}

std::vector<double> ManipulatorMenu::getKnownPose(const std::string& pose_name)
{
    // Check if the pose exists in the map
    auto it = known_poses_.find(pose_name);
    if (it != known_poses_.end())
    {
        return it->second;
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Pose %s not found in known poses.", pose_name.c_str());
        return std::vector<double>();
    }
}

// -------------------- PLANNING --------------------

manipulator_interfaces::msg::TrajectoryResult ManipulatorMenu::planAndWait(const sensor_msgs::msg::JointState joint_goal, const std::vector<double> start_state, uint timeout)
{
    traj_received_ = false;
    traj_result_ = manipulator_interfaces::msg::TrajectoryResult();
    
    // Publish the joint goal
    publishJointGoal(joint_goal, start_state, false);
    
    //Set a start time to check for timeout
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    auto start_time = steady_clock.now();

    rclcpp::Rate rate(params_.ros_freq);

    while (rclcpp::ok()){
        if((steady_clock.now() - start_time).seconds() > timeout){
            RCLCPP_ERROR(node_->get_logger(), "Timeout reached while waiting for planned trajectory.");
            break;
        }

        if(traj_received_){
            break;
        }

        rate.sleep();
    }

    return traj_result_;
}

manipulator_interfaces::msg::TrajectoryResult ManipulatorMenu::planAndWait(
    const geometry_msgs::msg::Pose tcp_goal, 
    const std::vector<double> start_state, 
    const std::string& frame,
    uint timeout)
{
    traj_received_ = false;
    traj_result_ = manipulator_interfaces::msg::TrajectoryResult();
    
    // Publish the tcp goal
    publishTcpGoal(tcp_goal, start_state, frame, false);

    //Set a start time to check for timeout
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    auto start_time = steady_clock.now();

    rclcpp::Rate rate(params_.ros_freq);

    while (rclcpp::ok()){

        if((steady_clock.now() - start_time).seconds() > timeout){
            RCLCPP_ERROR(node_->get_logger(), "Timeout reached while waiting for planned trajectory.");
            break;
        }

        if(traj_received_){
            break;
        }

        rate.sleep();
    }

    return traj_result_;
}

manipulator_interfaces::msg::TrajectoryResult ManipulatorMenu::cartesianPlanAndWait(
    const std::vector<geometry_msgs::msg::Pose> waypoints, 
    const std::vector<double> start_state, 
    const std::string& frame,
    uint timeout)
{
    traj_received_ = false;
    traj_result_ = manipulator_interfaces::msg::TrajectoryResult();
    
    // Publish the tcp goal
    publishCartesianGoal(waypoints, start_state, frame, false);

    //Set a start time to check for timeout
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    auto start_time = steady_clock.now();

    rclcpp::Rate rate(params_.ros_freq);

    while (rclcpp::ok()){

        if((steady_clock.now() - start_time).seconds() > timeout){
            RCLCPP_ERROR(node_->get_logger(), "Timeout reached while waiting for planned trajectory.");
            break;
        }

        if(traj_received_){
            break;
        }

        rate.sleep();
    }

    return traj_result_;
}

bool ManipulatorMenu::executeAndWait(moveit_msgs::msg::RobotTrajectory trajectory, uint timeout)
{
    trajectory_pub_->publish(trajectory);
    rclcpp::sleep_for(std::chrono::milliseconds(100)); // Sleep for a short time to ensure the trajectory is received before starting to check execution
    rclcpp::Rate rate(params_.ros_freq);

    sensor_msgs::msg::JointState goal_state;
    trajectory_msgs::msg::JointTrajectoryPoint last_traj_pt = trajectory.joint_trajectory.points.back();
    goal_state.position = last_traj_pt.positions;

    //Set a start time to check for timeout
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    auto start_time = steady_clock.now();

    //Tell the planner to start the execution
    std_msgs::msg::Bool ctrl_msg;
    ctrl_msg.data = true;
    RCLCPP_INFO(node_->get_logger(), "Starting trajectory execution");
    executionControl_pub_->publish(ctrl_msg);

    while(rclcpp::ok()){
        if((steady_clock.now() - start_time).seconds() > timeout){
            RCLCPP_ERROR(node_->get_logger(), "Timeout reached while waiting for trajectory execution.");
            break;
        }
        //Repeatedly check joint states to see if the goal has been reached
        for (size_t i {0}; i < goal_state.position.size(); i++){
            if (std::abs(goal_state.position[i] - current_joint_pose_.position[i]) > params_.joint_tolerance){
                break; //At least one joint is not in the goal position
            }

            if (i == goal_state.position.size() - 1) {return true;} //If we got to the last element trajectory exec is complete
        }

        rate.sleep();
    }

    return false;
}

bool ManipulatorMenu::planExecuteAndWait(
    const sensor_msgs::msg::JointState joint_goal, 
    uint timeout_planning,
    uint timeout_execution
){
    // Plan the trajectory
    manipulator_interfaces::msg::TrajectoryResult traj_result = planAndWait(joint_goal, std::vector<double>(), timeout_planning);

    if (traj_result.success)
    {
        // Execute the trajectory and wait for it to finish
        return executeAndWait(traj_result.trajectory, timeout_execution);
    }
    else if (traj_result_.error_code == manipulator_interfaces::msg::TrajectoryResult::SAME_POSITION)
    {
        return true; //If the robot was already in that position consider the execution successfull
    } else {
        return false;
    }
}

bool ManipulatorMenu::planExecuteAndWait(
    const std::vector<double> joint_goal, 
    uint timeout_planning,
    uint timeout_execution
){
    return planExecuteAndWait(joint_state_from_vector(joint_goal), timeout_planning, timeout_execution);
}

bool ManipulatorMenu::planExecuteAndWait(
    const std::string& known_pose, 
    uint timeout_planning,
    uint timeout_execution
){
    // Get the known pose
    std::vector<double> joint_goal = getKnownPose(known_pose);
    if (joint_goal.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Known pose %s not found.", known_pose.c_str());
        return false;
    }

    return planExecuteAndWait(joint_goal, timeout_planning, timeout_execution);
}

bool ManipulatorMenu::planExecuteAndWait(
    const geometry_msgs::msg::Pose tcp_goal, 
    const std::string& frame, //Leave empty for default frame
    uint timeout_planning,
    uint timeout_execution
){
    // Plan the trajectory
    manipulator_interfaces::msg::TrajectoryResult traj_result = planAndWait(tcp_goal, std::vector<double>(), frame, timeout_planning);
    
    std::vector<double> last_point = traj_result.trajectory.joint_trajectory.points.back().positions;
    for (unsigned long k = 0; k < last_point.size(); k++)
    {
        last_point[k] = last_point[k] * 180 / M_PI; //Convert rad to deg
    }
    geometry_msgs::msg::Pose goal_pose = getFKineClient(joint_state_from_vector(last_point));

    if (traj_result.success)
    {
        // Execute the trajectory and wait for it to finish
        return executeAndWait(traj_result.trajectory, timeout_execution);
    }
    else if (traj_result_.error_code == manipulator_interfaces::msg::TrajectoryResult::SAME_POSITION)
    {
        return true; //If the robot was already in that position consider the execution successfull
    } else {
        return false;
    }
}

bool ManipulatorMenu::cartesianPlanExecuteAndWait(
    const std::vector<geometry_msgs::msg::Pose> waypoints, 
    const std::string& frame, //Leave empty for default frame
    uint timeout_planning,
    uint timeout_execution
){
    // Plan the trajectory
    manipulator_interfaces::msg::TrajectoryResult traj_result = cartesianPlanAndWait(waypoints, std::vector<double>(), frame, timeout_planning);

    if (traj_result.success)
    {
        // Execute the trajectory and wait for it to finish
        return executeAndWait(traj_result.trajectory, timeout_execution);
    }
    else if (traj_result_.error_code == manipulator_interfaces::msg::TrajectoryResult::SAME_POSITION)
    {
        return true; //If the robot was already in that position consider the execution successfull
    } else {
        return false;
    }
}

void ManipulatorMenu::stopTrajectory(){
    // Publish an empty trajectory to stop the current one
    std_msgs::msg::Bool ctrl_msg;
    ctrl_msg.data = false;
    executionControl_pub_->publish(ctrl_msg);
    RCLCPP_INFO(node_->get_logger(), "Stopping trajectory execution");
}

// -------------------- TF END EFFECTOR LISTENER -----------------------

// Listen a TF between two given frames
geometry_msgs::msg::PoseStamped ManipulatorMenu::getTf(const std::string &reference_frame, const std::string &target_frame, uint num_tries)
{

    geometry_msgs::msg::TransformStamped transform;
    bool received_transform = false;
    for (size_t counter {0}; !received_transform && rclcpp::ok() && counter < num_tries; ++counter)
    {
        try {
            transform = tf_buffer_->lookupTransform(reference_frame, target_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
            received_transform = true;
        } catch (const tf2::TransformException & ex) {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }

    geometry_msgs::msg::PoseStamped pose_stamped;
    if (received_transform)
    {
        pose_stamped.header.stamp = transform.header.stamp;
        pose_stamped.header.frame_id = params_.base_link_name;
        pose_stamped.pose.position.x = transform.transform.translation.x;
        pose_stamped.pose.position.y = transform.transform.translation.y;
        pose_stamped.pose.position.z = transform.transform.translation.z;
        pose_stamped.pose.orientation = transform.transform.rotation;
    } else {
        RCLCPP_DEBUG(node_->get_logger(), "Failed to receive transform from %s to %s after %zu attempts.", reference_frame.c_str(), target_frame.c_str(), num_tries);
    }

    return pose_stamped;
}

geometry_msgs::msg::PoseStamped ManipulatorMenu::getTfOffset(const std::string& target_frame, 
                                                             const std::string& reference_frame,
                                                             const geometry_msgs::msg::Pose &offset)
{
    geometry_msgs::msg::PoseStamped pose = getTf(reference_frame, target_frame);
    pose.pose = getOffsetPose(pose.pose, offset);

    return pose;
}

geometry_msgs::msg::Pose ManipulatorMenu::getOffsetPose(const geometry_msgs::msg::Pose &pose, 
                                                        const geometry_msgs::msg::Pose &offset)
{
    // Convert base_pose to tf2::Transform
    tf2::Transform tf_base;
    tf2::fromMsg(pose, tf_base);

    // Convert relative offset to tf2::Transform
    tf2::Transform tf_offset;
    tf2::fromMsg(offset, tf_offset);

    // Compose the transforms: world_T_offset = world_T_base * base_T_offset
    tf2::Transform result = tf_base * tf_offset;

    // Convert back to geometry_msgs::Pose
    geometry_msgs::msg::Transform result_transform_msg = tf2::toMsg(result);

    geometry_msgs::msg::Pose result_pose;
    result_pose.position.x = result_transform_msg.translation.x;
    result_pose.position.y = result_transform_msg.translation.y;
    result_pose.position.z = result_transform_msg.translation.z;
    result_pose.orientation = result_transform_msg.rotation;

    return result_pose;
}


// Get current EE pose
geometry_msgs::msg::Pose ManipulatorMenu::getEEpose()
{
    return current_tcp_pose_;
}

// Get EE pose as vector with RPY euler angles
std::vector<double> ManipulatorMenu::getEEpos_rpy()
{
    // Read current EE pose by FKine
    geometry_msgs::msg::Pose pose = getEEpose();

    // Fill the rotation vector
    std::vector<double> tcp_rpy = euler_from_quaternion(pose.orientation);

    // Declaration of the pose vector
    std::vector<double> tcp_pose_rpy = {pose.position.x, pose.position.y, pose.position.z, tcp_rpy[0], tcp_rpy[1], tcp_rpy[2]};
    return tcp_pose_rpy;
}

sensor_msgs::msg::JointState ManipulatorMenu::getJointStateRadians()
{
    // Get current joint state
    return joint_state_from_vector(joints_values_group_);
}

std::vector<double> ManipulatorMenu::getJointStateDegrees()
{
    // Get current joint values in radians
    std::vector<double> joint_values;
    for (unsigned long k = 0; k < params_.joint_names.size(); ++k)
    {
        joint_values.push_back(joints_values_group_[k] * 180 / M_PI);
    }
    return joint_values;
}

// -------------------- SIMPLE MOVES ALONG CARTHESIAN AXES -----------------------//

// Set a carthesian move along x axis in metres
geometry_msgs::msg::Pose ManipulatorMenu::move_along_x(const double x_step, bool linear)
{
    // Get current EE pose
    geometry_msgs::msg::Pose goal_pose = getEEpose();
    // Update position along X
    goal_pose.position.x += x_step;

    if (linear)
    {
        // publishCartesianGoal({goal_pose});
        cartesianPlanAndWait({goal_pose});
        return goal_pose;
    }
    else
    {
        return publishTcpGoal(goal_pose);
    }
}

// Set a carthesian move along x axis in metres
geometry_msgs::msg::Pose ManipulatorMenu::move_along_y(const double y_step, bool linear)
{
    // Get current EE pose
    geometry_msgs::msg::Pose goal_pose = getEEpose();
    // Update position along Y
    goal_pose.position.y += y_step;

    if (linear)
    {
        // publishCartesianGoal({goal_pose});
        cartesianPlanAndWait({goal_pose});
        return goal_pose;
    }
    else
    {
        return publishTcpGoal(goal_pose);
    }
}

// Set a carthesian move along x axis in metres
geometry_msgs::msg::Pose ManipulatorMenu::move_along_z(const double z_step, bool linear)
{
    // Get current EE pose
    geometry_msgs::msg::Pose goal_pose = getEEpose();
    // Update position along Z
    goal_pose.position.z += z_step;

    if (linear)
    {
        // publishCartesianGoal({goal_pose});
        cartesianPlanAndWait({goal_pose});
        return goal_pose;
    }
    else
    {
        return publishTcpGoal(goal_pose);
    }
}

// -------------------- SIMPLE ROTATIONS AROUND CARTHESIAN AXES -----------------------//

// Set a RELATIVE ee rotation around the 3 carthesian axis (in degrees)
geometry_msgs::msg::Pose ManipulatorMenu::make_tcp_rot(const std::vector<double> rot_vec)
{
    // Get current EE pose
    std::vector<double> goal_pose = getEEpos_rpy();
    // Update tcp orient goal
    goal_pose[3] = goal_pose[3] + rot_vec[0];
    goal_pose[4] = goal_pose[4] + rot_vec[1];
    goal_pose[5] = goal_pose[5] + rot_vec[2];
    return publishTcpGoal(goal_pose);
}

// Set an ABSOLUTE orientation ee position around the 3 carthesian axis (in degrees)
geometry_msgs::msg::Pose ManipulatorMenu::change_tcp_orient(const std::vector<double> rot_vec)
{
    // Get current EE pose
    std::vector<double> goal_pose = getEEpos_rpy();
    // Update tcp orient goal
    goal_pose[3] = rot_vec[0];
    goal_pose[4] = rot_vec[1];
    goal_pose[5] = rot_vec[2];
    return publishTcpGoal(goal_pose);
}

// Set a relative rotation around x axis (in degrees)
geometry_msgs::msg::Pose ManipulatorMenu::rotate_around_x(const double x_rot_step)
{
    // Get current EE pose
    std::vector<double> goal_pose = getEEpos_rpy();
    // Update tcp orient goal
    goal_pose[3] = goal_pose[3] + x_rot_step;
    return publishTcpGoal(goal_pose);
}

// Set a relative rotation around y axis (in degrees)
geometry_msgs::msg::Pose ManipulatorMenu::rotate_around_y(const double y_rot_step)
{
    // Get current EE pose
    std::vector<double> goal_pose = getEEpos_rpy();
    // Update tcp orient goal
    goal_pose[4] = goal_pose[4] + y_rot_step;
    return publishTcpGoal(goal_pose);
}

// Set a relative rotation around z axis (in degrees)
geometry_msgs::msg::Pose ManipulatorMenu::rotate_around_z(const double z_rot_step)
{
    // Get current EE pose
    std::vector<double> goal_pose = getEEpos_rpy();
    // Update tcp orient goal
    goal_pose[5] = goal_pose[5] + z_rot_step;
    return publishTcpGoal(goal_pose);
}

// --------------------- GRIPPER ---------------------

void ManipulatorMenu::moveGripper(const bool close)
{
    if(params_.gripper == "robotiq_85"){
        gripperMoveClient(close);
    } else if (params_.gripper == "toolIO"){
        if (close){
            publishToolIOCmd(params_.gripper_IO_cmds[1], false);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            publishToolIOCmd(params_.gripper_IO_cmds[0], true);
        } else {
            publishToolIOCmd(params_.gripper_IO_cmds[0], false);
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            publishToolIOCmd(params_.gripper_IO_cmds[1], true);
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Gripper is not enabled.");
    }
}

// --------------------- COLLISION OBJECTS HANDLER ---------------------

// Create a collision object from a selected primitive
void ManipulatorMenu::addObj(const std::string &name,
                             const int obj_type,
                             std::vector<double> obj_dims,
                             double obj_pos[],
                             double rot_pos[],
                             uint operation)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = obj_pos[0];
    pose.position.y = obj_pos[1];
    pose.position.z = obj_pos[2];

    // Set obj orientation
    pose.orientation.x = rot_pos[0];
    pose.orientation.y = rot_pos[1];
    pose.orientation.z = rot_pos[2];
    pose.orientation.w = rot_pos[3];

    addObj(
        name,
        obj_type,
        obj_dims,
        pose,
        operation
    );
}

void ManipulatorMenu::addObj(const std::string& name,
                             const int obj_type, 
                             std::vector<double> obj_dims, 
                             geometry_msgs::msg::Pose obj_pos,
                             uint operation)
{
    // Creation of the obj
    moveit_msgs::msg::CollisionObject obj;

    obj.header.frame_id = params_.base_link_name;
    obj.id = name;
    obj.primitives.resize(1);
    obj.primitives[0].type = obj_type;
    int size_obj_dims = obj_dims.size();
    obj.primitives[0].dimensions.resize(size_obj_dims);

    // Set primitive type
    switch (obj_type)
    {
    case 1: // BOX: Rectangular shape setting
        if (size_obj_dims != 3)
        {
            RCLCPP_ERROR(node_->get_logger(), "obj_dims array is not compatible with obj_type");
        }
        else
        { // Set the three dimensions of the parallelepiped
            obj.primitives[0].dimensions[0] = obj_dims[0];
            obj.primitives[0].dimensions[1] = obj_dims[1];
            obj.primitives[0].dimensions[2] = obj_dims[2];
        }
        break;

    case 2: // SPHERE
        if (size_obj_dims != 1)
        {
            RCLCPP_ERROR(node_->get_logger(), "obj_dims array is not compatible with obj_type");
        }
        else
        { // Set the sphere radius
            obj.primitives[0].dimensions[0] = obj_dims[0];
        }
        break;

    default: // CYLINDER OR CONE
        if (size_obj_dims != 2)
        {
            RCLCPP_ERROR(node_->get_logger(), "obj_dims array is not compatible with obj_type");
        }
        else
        { // Set height and radius of the cylinder/cone
            obj.primitives[0].dimensions[0] = obj_dims[0];
            obj.primitives[0].dimensions[1] = obj_dims[1];
        }
        break;
    }

    // Set obj operation: ADD=0, REMOVE=1, APPEND=2, MOVE=3
    obj.operation = operation;

    // Set obj position
    obj.primitive_poses.push_back(obj_pos);

    publishCollisionObject(obj);
}

void ManipulatorMenu::removeObj(const std::string &name)
{
    // Create a collision object message to remove the object
    moveit_msgs::msg::CollisionObject collisionObjectMsg;
    collisionObjectMsg.id = name;
    collisionObjectMsg.primitives.resize(1);
    collisionObjectMsg.primitives[0].type = 1; // Use a dummy primitive type
    collisionObjectMsg.primitives[0].dimensions.resize(3, 0.0); // Set dummy dimensions
    collisionObjectMsg.primitive_poses.resize(1); //Set dummy pose
    collisionObjectMsg.operation = moveit_msgs::msg::CollisionObject::REMOVE;

    // Publish the collision object
    publishCollisionObject(collisionObjectMsg);
}

// Collision object publisher
void ManipulatorMenu::publishCollisionObject(const moveit_msgs::msg::CollisionObject collisionObjectMsg)
{
    collisionObject_pub_->publish(collisionObjectMsg);
}

// Create a collision object from a selected primitive
void ManipulatorMenu::addAttachedObj(const std::string &name,
                                     const int obj_type,
                                     std::vector<double> obj_dims,
                                     geometry_msgs::msg::Pose obj_pose,
                                     uint operation)
{
    // Creation of the obj
    moveit_msgs::msg::CollisionObject obj;

    obj.header.frame_id = params_.base_link_name;
    obj.id = name;
    obj.primitives.resize(1);
    obj.primitives[0].type = obj_type;
    int size_obj_dims = obj_dims.size();
    obj.primitives[0].dimensions.resize(size_obj_dims);

    // Set primitive type
    switch (obj_type)
    {
    case 1: // BOX: Rectangular shape setting
        if (size_obj_dims != 3)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000, "obj_dims array is not compatible with obj_type");
        }
        else
        { // Set the three dimensions of the parallelepiped
            obj.primitives[0].dimensions[0] = obj_dims[0];
            obj.primitives[0].dimensions[1] = obj_dims[1];
            obj.primitives[0].dimensions[2] = obj_dims[2];
        }
        break;

    case 2: // SPHERE
        if (size_obj_dims != 1)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000, "obj_dims array is not compatible with obj_type");
        }
        else
        { // Set the sphere radius
            obj.primitives[0].dimensions[0] = obj_dims[0];
        }
        break;

    default: // CYLINDER OR CONE
        if (size_obj_dims != 2)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000, "obj_dims array is not compatible with obj_type");
        }
        else
        { // Set height and radius of the cylinder/cone
            obj.primitives[0].dimensions[0] = obj_dims[0];
            obj.primitives[0].dimensions[1] = obj_dims[1];
        }
        break;
    }

    // Set obj operation: ADD=0, REMOVE=1, APPEND=2, MOVE=3
    obj.operation = operation;

    obj.primitive_poses.push_back(obj_pose);

    // Set the attached object
    moveit_msgs::msg::AttachedCollisionObject attachedObj;
    attachedObj.object = obj;
    publishAttachedCollisionObject(attachedObj);
}

void ManipulatorMenu::addAttachedObj(const std::string &name,
                                     const int obj_type,
                                     std::vector<double> obj_dims,
                                     double obj_pos[],
                                     double rot_pos[],
                                     uint operation)
{
    geometry_msgs::msg::Pose pose;
    // Set obj position
    pose.position.x = obj_pos[0];
    pose.position.y = obj_pos[1];
    pose.position.z = obj_pos[2];

    // Set obj orientation
    pose.orientation.x = rot_pos[0];
    pose.orientation.y = rot_pos[1];
    pose.orientation.z = rot_pos[2];
    pose.orientation.w = rot_pos[3];

    addAttachedObj(
        name,
        obj_type,
        obj_dims,
        pose,
        operation
    );
}

void ManipulatorMenu::removeAttachedObj(const std::string &name)
{
    // Create a collision object message to remove the object
    moveit_msgs::msg::AttachedCollisionObject collisionAttachedObjectMsg;
    collisionAttachedObjectMsg.object.id = name;
    collisionAttachedObjectMsg.object.primitives.resize(1);
    collisionAttachedObjectMsg.object.primitives[0].type = 1; // Use a dummy primitive type
    collisionAttachedObjectMsg.object.primitives[0].dimensions.resize(3, 0.0); // Set dummy dimensions
    collisionAttachedObjectMsg.object.primitive_poses.resize(1); //Set dummy pose
    collisionAttachedObjectMsg.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

    // Publish the collision object
    publishAttachedCollisionObject(collisionAttachedObjectMsg);

    rclcpp::sleep_for(std::chrono::milliseconds(300));

    removeObj(name);
}

// Collision Attached object publisher
void ManipulatorMenu::publishAttachedCollisionObject(const moveit_msgs::msg::AttachedCollisionObject collisionAttachedObjectMsg)
{
    attachedCollisionObject_pub_->publish(collisionAttachedObjectMsg);
}

void ManipulatorMenu::publishToolIOCmd(const size_t id, const bool value)
{
    std_msgs::msg::Int8 msg;
    msg.data = id + 1;

    if (!value){
        msg.data = -msg.data;
    }

    toolDigitalIO_pub_->publish(msg);
}

// ------------------- CONSTRAINTS ---------------------

void ManipulatorMenu::publishJointConstraint(const uint &joint_index,
                                             const double &min_position,
                                             const double &max_position,
                                             const double &weight)
{

    double middle_pos = (min_position + max_position) / 2.0 + min_position;
    double tolerance_below = middle_pos - min_position;
    double tolerance_above = max_position - middle_pos;

    // Create a joint constraint
    moveit_msgs::msg::JointConstraint constraint;
    constraint.joint_name = params_.joint_names[joint_index];
    constraint.position = middle_pos / 180.0 * M_PI; // Convert to radians
    constraint.tolerance_below = tolerance_below / 180.0 * M_PI;
    constraint.tolerance_above = tolerance_above / 180.0 * M_PI;
    constraint.weight = weight;

    RCLCPP_INFO(node_->get_logger(), "Joint constraint for joint: %s in range [%f, %f]", constraint.joint_name.c_str(), min_position, max_position);
    
    jointConstraints_pub_->publish(constraint);
}
void ManipulatorMenu::publishPositionConstraint(const std::string& link_name, 
                                                const geometry_msgs::msg::Pose& shape_pose, 
                                                const uint &shape_type, 
                                                const std::vector<double>& shape_dims, 
                                                const double &weight)
{
    shape_msgs::msg::SolidPrimitive shape;
    shape.type = shape_type;
    for (size_t i = 0; i < shape_dims.size(); ++i)
    {
        shape.dimensions.push_back(shape_dims[i]);
    }

    moveit_msgs::msg::PositionConstraint position_constraint;
    position_constraint.header.frame_id = params_.base_link_name;
    position_constraint.link_name = link_name;
    position_constraint.constraint_region.primitives.push_back(shape);
    position_constraint.constraint_region.primitive_poses.push_back(shape_pose);
    position_constraint.weight = weight;

    RCLCPP_INFO(node_->get_logger(), "Position constraint for link: %s, shape type: %d, position: (%fm, %fm, %fm)", link_name.c_str(), shape_type, shape_pose.position.x, shape_pose.position.y, shape_pose.position.z);

    positionConstraints_pub_->publish(position_constraint);
}

void ManipulatorMenu::publishOrientationConstraint(const std::string& link_name, 
                                                   const geometry_msgs::msg::Quaternion& orientation, 
                                                   const std::vector<double> &tolerances,
                                                   const double &weight)
{
    moveit_msgs::msg::OrientationConstraint orientation_constraint;
    orientation_constraint.header.frame_id = params_.base_link_name;
    orientation_constraint.link_name = link_name;
    orientation_constraint.orientation = orientation;
    orientation_constraint.absolute_x_axis_tolerance = tolerances[0];
    orientation_constraint.absolute_y_axis_tolerance = tolerances[1];
    orientation_constraint.absolute_z_axis_tolerance = tolerances[2];
    orientation_constraint.weight = weight;


    std::vector<double> pos_rpy = euler_from_quaternion(orientation);
    RCLCPP_INFO(node_->get_logger(), "Orientation constraint for link: %s, position: (%f˚, %f˚, %f˚)", link_name.c_str(), pos_rpy[0], pos_rpy[1], pos_rpy[2]);

    orientationConstraints_pub_->publish(orientation_constraint);
}

void ManipulatorMenu::publishClearConstraints()
{
    std_msgs::msg::Empty empty_msg;
    clearConstraints_pub_->publish(empty_msg);
}


// ------------------- KINEMATICS PARAMS SETTERS ---------------------- //

// Set Jacobian-based speed control
void ManipulatorMenu::setJacobianSpeedControl(bool set)
{
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data  = set;

    // Check if service is available
    if (!setJacobianControl_client_->wait_for_service(std::chrono::milliseconds(100)))
    {
        RCLCPP_ERROR(node_->get_logger(), "Unable to connect to jacobian_control_setter service");
        return;
    }

    auto cb = [set, this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future){
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO(node_->get_logger(), "Jacobian control set to %s", set ? "true" : "false");
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set Jacobian control: %s", result->message.c_str());
        }
    };

    // Send the request asynchronously
    setJacobianControl_client_->async_send_request(request, cb);
}

// Set Joints real time speed control
void ManipulatorMenu::setJsRealTimeControl(bool set)
{
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data  = set;

    // Check if service is available
    if (!setRealTimeControl_client_->wait_for_service(std::chrono::milliseconds(100)))
    {
        RCLCPP_ERROR(node_->get_logger(), "Unable to connect to set_real_time_control service");
        return;
    }

    auto cb = [set, this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future){
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO(node_->get_logger(), "Real time joints control set to %s", set ? "true" : "false");
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set real time joints control: %s", result->message.c_str());
        }
    };

    // Send the request asynchronously
    setRealTimeControl_client_->async_send_request(request, cb);
}

void ManipulatorMenu::setRealTimeConstraints(bool limit_joints, bool limit_jacobian){
    auto request = std::make_shared<manipulator_interfaces::srv::EnableRealTimeConstraints::Request>();
    request->limit_joints_control = limit_joints;
    request->limit_jacobian_control = limit_jacobian;

    // Check if service is available
    if (!enableRealTimeConstraints_client_->wait_for_service(std::chrono::milliseconds(100)))
    {
        RCLCPP_ERROR(node_->get_logger(), "Unable to connect to enable_real_time_constraints service");
        return;
    }

    auto cb = [&, this](rclcpp::Client<manipulator_interfaces::srv::EnableRealTimeConstraints>::SharedFuture future){
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO(node_->get_logger(), "Set real time constraints: limit_joints_control = %s, limit_jacobian_control = %s",
                        limit_joints ? "True" : "False", limit_jacobian ? "True" : "False");
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set real time constraints.");
        }
    };

    // Send the request asynchronously
    enableRealTimeConstraints_client_->async_send_request(request, cb); 
}

// Set new dynamic planners vel/acc params
void ManipulatorMenu::setPlannerScalingFactors(float new_vel, float new_acc)
{
    auto request = std::make_shared<manipulator_interfaces::srv::ChangePlannerScalingFactors::Request>();
    request->acc_factor = new_acc;
    request->vel_factor = new_vel;

    // Check if service is available
    if (!changePlannerScalingFactors_client_->wait_for_service(std::chrono::milliseconds(100)))
    {
        RCLCPP_ERROR(node_->get_logger(), "Unable to connect to set_planner_scaling_factors service");
        return;
    }

    auto cb = [&, this](rclcpp::Client<manipulator_interfaces::srv::ChangePlannerScalingFactors>::SharedFuture future){
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO(node_->get_logger(), "Planner scaling factors changed: acc_factor = %f, vel_factor = %f", new_acc, new_vel);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set planner scaling factors");
        }
    };

    // Send the request asynchronously
    changePlannerScalingFactors_client_->async_send_request(request, cb);
}

// Set new dynamic planners vel/acc params
void ManipulatorMenu::setPlannerTolerances(float position, float orientation, float joint)
{
    auto request = std::make_shared<manipulator_interfaces::srv::ChangePlannerTolerances::Request>();
    request->position_tolerance = position;
    request->orientation_tolerance = orientation;
    request->joint_tolerance = joint;

    // Update tolerances
    params_.joint_tolerance = joint; 
    params_.tcp_position_tolerance = position; 
    params_.tcp_orientation_tolerance = orientation; 

    // Check if service is available
    while (!changePlannerTolerances_client_->wait_for_service(std::chrono::milliseconds(100)))
    {
        RCLCPP_ERROR(node_->get_logger(), "Unable to connect to set_planner_tolerances service");
        return;
    }

    auto cb = [&, this](rclcpp::Client<manipulator_interfaces::srv::ChangePlannerTolerances>::SharedFuture future){
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO(node_->get_logger(), "Planner params changed: position_tolerance = %f, orientation_tolerance = %f, joint_tolerance = %f", position, orientation, joint);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set planner params");
        }
    };

    // Send the request asynchronously
    changePlannerTolerances_client_->async_send_request(request, cb);
}

// --------------------- ADMITTANCE CONTROL ---------------------

void ManipulatorMenu::setAdmittanceControl(bool set)
{
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data  = set;

    // Check if service is available
    while (!setAdmittanceControl_client_->wait_for_service(std::chrono::milliseconds(100)))
    {
        RCLCPP_ERROR(node_->get_logger(), "Unable to connect to enable_admittance_control service");
        return;
    }

    auto cb = [&, this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future){
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO(node_->get_logger(), "Admittance control %s", set ? "ENABLED" : "DISABLED");
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set admittance control");
        }
    };

    // Send the request asynchronously
    setAdmittanceControl_client_->async_send_request(request, cb);
}

void ManipulatorMenu::setAdmittanceVelMode(bool set)
{
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data  = set;

    // Check if service is available
    while (!setAdmittanceVelMode_client_->wait_for_service(std::chrono::milliseconds(100)))
    {
        RCLCPP_ERROR(node_->get_logger(), "Unable to connect to admittance_vel_mode service");
        return;
    }

    auto cb = [&, this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future){
        auto result = future.get();
        if (result->success)
        {
            RCLCPP_INFO(node_->get_logger(), "Admittance control mode set to %s", set ? "VELOCITY" : "POSITION");
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set admittance control mode");
        }
    };

    // Send the request asynchronously
    setAdmittanceVelMode_client_->async_send_request(request, cb);
}

// --------------------- MATRIX UTILS ------------------------

void ManipulatorMenu::printMatrix(const Eigen::MatrixXd &matrix)
{
    for (long i = 0; i < matrix.rows(); i++)
    {
        for (long j = 0; j < matrix.cols(); j++)
        {
            std::cout << MenuUserInterfaceUtils::formatDouble(matrix(i, j), 2, 5) << " ";
        }
        std::cout << std::endl;
    }
}

void ManipulatorMenu::listToMatrix(const std::vector<double> &list, Eigen::MatrixXd &matrix)
{
    for (long i = 0; i < matrix.rows(); ++i)
    {
        for (long j = 0; j < 6; ++j)
        {
            matrix(i, j) = list[i * 6 + j];  // Fix the index to access correct values
        }
    }
}

// --------------------- QUATERNIONS UTILS -------------------

// Conversion from degrees euler angles to quaternion
geometry_msgs::msg::Quaternion ManipulatorMenu::quaternion_from_euler(double roll, double pitch, double yaw)
{
    // Declaration of empty quaternion
    geometry_msgs::msg::Quaternion quaternion;

    // Conversion from euler rotation to pose quaternion
    tf2::Quaternion quat;
    quat.setRPY(roll * M_PI / 180., pitch * M_PI / 180., yaw * M_PI / 180.);
    quat.normalize();
    quaternion.x = quat.getX();
    quaternion.y = quat.getY();
    quaternion.z = quat.getZ();
    quaternion.w = quat.getW();

    return quaternion;
}
// Conversion from quaternion to degrees euler angles
std::vector<double> ManipulatorMenu::euler_from_quaternion(const geometry_msgs::msg::Quaternion quaternion)
{
    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quaternion, tf_quaternion);

    // Get Euler angles
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quaternion).getRPY(roll, pitch, yaw);

    // Store the angles in a vector
    std::vector<double> euler_angles = {roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI};

    // Check if angles are in the interval (-180,180]
    for (unsigned long k = 0; k < 3; k++)
    {
        if (euler_angles[k] < -179.9999999999)
        {
            euler_angles[k] += 360.;
        }
        else if (euler_angles[k] > +180.)
        {
            euler_angles[k] -= 360.;
        }
    }

    return euler_angles;
}

//Multiply two quaternions (apply rotation of q2 to q1)
geometry_msgs::msg::Quaternion ManipulatorMenu::quaternion_multiply(const geometry_msgs::msg::Quaternion& q1, const geometry_msgs::msg::Quaternion& q2)
{
    // Convert geometry_msgs::msg::Quaternion to Eigen::Quaterniond
    Eigen::Quaterniond quat1(q1.w, q1.x, q1.y, q1.z);
    Eigen::Quaterniond quat2(q2.w, q2.x, q2.y, q2.z);   
    // Perform quaternion multiplication
    Eigen::Quaterniond result = quat1 * quat2;
    // Convert back to geometry_msgs::msg::Quaternion
    geometry_msgs::msg::Quaternion result_quat;
    result_quat.w = result.w();
    result_quat.x = result.x();
    result_quat.y = result.y();
    result_quat.z = result.z();
    return result_quat;
}

// --------------------- DEG-RADIANS UTILS -------------------
std::vector<double> ManipulatorMenu::deg_from_rad(const std::vector<double> joint_rad)
{
    std::vector<double> joint_deg(joint_rad.size(), 0);
    // Iterate over input vector
    for (unsigned long k = 0; k < joint_deg.size(); k++)
    {
        joint_deg[k] = joint_rad[k] * 180 / M_PI;
    }
    // Return result
    return joint_deg;
}

std::vector<double> ManipulatorMenu::rad_from_deg(const std::vector<double> joint_deg)
{
    std::vector<double> joint_rad(joint_deg.size(), 0);
    // Iterate over input vector
    for (unsigned long k = 0; k < joint_rad.size(); k++)
    {
        joint_rad[k] = joint_deg[k] / 180 * M_PI;
    }
    // Return result
    return joint_rad;
}

sensor_msgs::msg::JointState ManipulatorMenu::joint_state_from_vector(const std::vector<double> positions)
{
    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = node_->get_clock()->now();
    joint_state.header.frame_id = params_.base_link_name;
    joint_state.name = params_.joint_names;
    joint_state.position = rad_from_deg(positions);
    return joint_state;
}

std::vector<double> ManipulatorMenu::vector_from_joint_state(const sensor_msgs::msg::JointState joint_state)
{
    return deg_from_rad(joint_state.position);
}

geometry_msgs::msg::Pose ManipulatorMenu::pose_from_vector(const std::vector<double> position)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = position[0];
    pose.position.y = position[1];
    pose.position.z = position[2];

    pose.orientation = quaternion_from_euler(position[3], position[4], position[5]);
    return pose;
}

std::vector<double> ManipulatorMenu::vector_from_pose(const geometry_msgs::msg::Pose pose)
{
    std::vector<double> vector = std::vector<double>(6, 0);
    vector[0] = pose.position.x;
    vector[1] = pose.position.y;
    vector[2] = pose.position.z;

    std::vector<double> rpy = euler_from_quaternion(pose.orientation);
    vector[3] = rpy[0];
    vector[4] = rpy[1];
    vector[5] = rpy[2];

    return vector;
}

// --------------------- GENERIC UTILS -------------------

double ManipulatorMenu::euclidean_distance(const std::vector<double>& a, const std::vector<double>& b)
{   
    if (a.size() != b.size())
    {
        RCLCPP_ERROR(node_->get_logger(), "Vectors must have the same size to compute euclidean distance.");
        return -1;
    }

    double sum = 0;
    for (size_t i{0}; i < a.size(); i++)
    {
        sum += std::pow(a[i] - b[i], 2);
    }

    return sqrt(sum);
}

double ManipulatorMenu::euclidean_distance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)
{
    return euclidean_distance({a.x, a.y, a.z}, {b.x, b.y, b.z});
}

double ManipulatorMenu::angular_distance(const geometry_msgs::msg::Quaternion& q1, const geometry_msgs::msg::Quaternion& q2)
{
    // Convert geometry_msgs::msg::Quaternion to Eigen::Quaterniond
    Eigen::Quaterniond quat1(q1.w, q1.x, q1.y, q1.z);
    Eigen::Quaterniond quat2(q2.w, q2.x, q2.y, q2.z);

    // Compute the relative rotation
    Eigen::Quaterniond relative_rotation = quat1.inverse() * quat2;

    // Extract the angle of rotation
    double angle = 2 * std::acos(relative_rotation.w());

    // Normalize the angle to the range [0, pi]
    if (angle > M_PI)
    {
        angle = 2 * M_PI - angle;
    }

    return angle;
}

/*
    ================================================================
    ===================== PRIVATE FUNCTIONS  =======================
    ================================================================
*/

void ManipulatorMenu::loadKnownPoses(){

    std::string path = params_.known_poses_path;
    try {
        // Check file exists by trying to open it.

        std::ifstream fh(path.c_str());
        if (!fh.good()) {
            RCLCPP_ERROR(node_->get_logger(), "Known poses file not found: %s", path.c_str());
        }
        fh.close();

        YAML::Node config = YAML::LoadFile(path);
        RCLCPP_INFO(node_->get_logger(), "Knwon poses file loaded successfully.");
        for (const auto& pose : config) {
            std::string name = pose.first.as<std::string>();
            std::vector<double> position = pose.second.as<std::vector<double>>();
            known_poses_[name] = position;
            RCLCPP_DEBUG(node_->get_logger(), "Pose %s loaded: (%f, %f, %f, %f, %f, %f)", name.c_str(), position[0], position[1], position[2], position[3], position[4], position[5]);
        }
    } catch (const YAML::Exception & e) {
        RCLCPP_ERROR(node_->get_logger(), "Error parsing YAML file %s: %s", path.c_str(), e.what());
    }
}

void ManipulatorMenu::waitManipulatorParameters(){    
    params_.tcp_position_tolerance = getManipulatorParameter<double>("position_tolerance");
    params_.tcp_orientation_tolerance = getManipulatorParameter<double>("orientation_tolerance");
    params_.joint_tolerance = getManipulatorParameter<double>("joint_tolerance");
    params_.joint_names = getManipulatorParameter<std::vector<std::string>>("joint_names");
    params_.manipulator_name = getManipulatorParameter<std::string>("manipulator_name");
    params_.base_link_name = getManipulatorParameter<std::string>("world_frame");
    params_.planning_group = getManipulatorParameter<std::string>("planning_group");
}

void ManipulatorMenu::shutdown_handler()
{
    RCLCPP_DEBUG(node_->get_logger(), "Shutting down manipulator menu.");
}

void ManipulatorMenu::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr &joints_state)
{

    // Map to store couples joint name - joint values
    std::unordered_map<std::string, double>::iterator it;
    uint counter_group = 0;

    RCLCPP_INFO_ONCE(node_->get_logger(), "Listening to joints states.");

    for (uint i = 0; i < joints_state->name.size(); i++)
    {
        // Look for joints group names within joints current state
        it = joints_map_group_.find(joints_state->name[i]);
        // Exclude last link (gripper) from the search
        if (it != joints_map_group_.end())
        {
            // At the second position of the iteration, insert current joint position and velocity
            it->second = joints_state->position[i];

            // Increment the number of joints received from the joints state subscriber
            counter_group++;

            // If we have reached the last joint of the group
            if (counter_group == params_.joint_names.size())
            {
                // Iterate over the joints
                for (unsigned long k = 0; k < params_.joint_names.size(); k++)
                {
                    // Store the joints values from the joints map
                    joints_values_group_[k] = joints_map_group_[params_.joint_names[k]];
                }

                // Log gripper planning group
                RCLCPP_INFO_ONCE(node_->get_logger(), "%s joints values received by the menu interface.", params_.manipulator_name.c_str());
            }
        }
    }

    // Store the value into the global (public) class variable
    current_joint_pose_.position = joints_values_group_;
}

void ManipulatorMenu::trajectoryCallback(const manipulator_interfaces::msg::TrajectoryResult::SharedPtr& msg)
{
    //std::lock_guard<std::mutex> lock(traj_mtx_);
    if(msg->success)
    {
        RCLCPP_INFO(node_->get_logger(), "Trajectory planned successfully.");
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Trajectory planning failed, error code: %d, error msg: %s", msg->error_code, msg->message.c_str());
    }

    traj_result_ = *msg.get();
    traj_received_ = true;
}


/*
    ================================================================
    ================== USER INTERFACE FUNCTIONS ====================
    ================================================================
    Each one of the following functions are called by the user menu
    inteface after being chosen by the user.
    At the end of the file lies the initializeMenu function which 
    will bind each user function to a specific code in the menu.

    NOTE: For ease of recognition these functions should always have 
          the prefix "user".
*/

// --------------------- JOINT GOALS HANDLER ---------------------

void ManipulatorMenu::userJointGoal()
{
    // Declare the empty vector of joints goals
    std::vector<double> joints;

    // Take user degree angle for each joint
    std::cout << "Enter the values of the joint goal in degrees: \n";

    for (unsigned long k = 0; k < params_.joint_names.size(); k++)
    {
        double new_joint_value = 0.;
        std::cout << "Joint " << k + 1 << " : ";
        std::cin >> new_joint_value;
        joints.push_back(new_joint_value);
    }

    manipulator_interfaces::msg::TrajectoryResult traj = planAndWait(joint_state_from_vector(joints));
    if(traj.success)
    {
        int execute = 0;
        std::cout << "Trajectory planned successfully." << std::endl;
        std::cout << "Trajectory points count: " << traj.trajectory.joint_trajectory.points.size() << std::endl;
        std::cout << "Trajectory duration: " << traj.trajectory.joint_trajectory.points.back().time_from_start.sec << "s" << std::endl;
        std::cout << "Do you want to execute the trajectory? 1 for yes: ";
        std::cin >> execute;

        if (execute == 1)
        {
            std::cout << "Executing trajectory..." << std::endl;
            bool success = executeAndWait(traj.trajectory);
            if (success)
            {
                std::cout << "Trajectory executed successfully." << std::endl;
            }
            else
            {
                std::cout << "Trajectory execution failed." << std::endl;
            }
        }
    }
}

void ManipulatorMenu::userOneJointMove()
{
    int num = 0;
    double joint_rot = 0.0;
    std::cout << "Enter the joint to move in [0, " << params_.joint_names.size() - 1 << "]: \n";
    std::cin >> num;
    std::cout << "Enter the rotation of the joint in deg: \n";
    std::cin >> joint_rot;
    oneJointMove(num, joint_rot);
}

// --------------------- TCP GOALS HANDLER ---------------------

void ManipulatorMenu::userTcpGoal()
{
    // Declare the empty vector of joints goals
    std::vector<double> position = {0., 0., 0., 0., 0., 0.};

    // Take user degree angle for each joint
    std::cout << "Enter the values of the tcp goal, with rotation angles in degrees:\n";

    // X position input
    std::cout << "X position:  ";
    std::cin >> position[0];
    // Y position input
    std::cout << "Y position:  ";
    std::cin >> position[1];
    // Z position input
    std::cout << "Z position:  ";
    std::cin >> position[2];

    // Deg RPY angles input
    std::cout << "Rx: ";
    std::cin >> position[3];
    std::cout << "Ry: ";
    std::cin >> position[4];
    std::cout << "Rz: ";
    std::cin >> position[5];

    geometry_msgs::msg::Pose goal;
    goal.position.x = position[0];
    goal.position.y = position[1];
    goal.position.z = position[2];
    goal.orientation = quaternion_from_euler(position[3], position[4], position[5]);

    manipulator_interfaces::msg::TrajectoryResult traj = planAndWait(goal);
    if(traj.success)
    {
        int execute = 0;
        std::cout << "Trajectory planned successfully." << std::endl;
        std::cout << "Trajectory points count: " << traj.trajectory.joint_trajectory.points.size() << std::endl;
        std::cout << "Trajectory duration: " << traj.trajectory.joint_trajectory.points.back().time_from_start.sec << "s" << std::endl;
        std::cout << "Do you want to execute the trajectory? 1 for yes: ";
        std::cin >> execute;

        if (execute == 1)
        {
            std::cout << "Executing trajectory..." << std::endl;
            bool success = executeAndWait(traj.trajectory);
            if (success)
            {
                std::cout << "Trajectory executed successfully." << std::endl;
            }
            else
            {
                std::cout << "Trajectory execution failed." << std::endl;
            }
        }
    }
}

// --------------------- CARTESIAN PLAN HANDLER ---------------------

void ManipulatorMenu::userCartesianGoal(){
    std::cout << "Enter the number of waypoints for the Cartesian path: ";
    size_t num_waypoints = 0;
    std::cin >> num_waypoints;
    
    std::vector<geometry_msgs::msg::Pose> waypoints;

    for (size_t i = 0; i < num_waypoints; i++){
        std::cout << "Waypoint " << i+1 << ":\n";
        // Declare the empty vector of joints goals
        std::vector<double> position = {0., 0., 0., 0., 0., 0.};

        // Take user degree angle for each joint
        std::cout << "Enter the values of the tcp goal, with rotation angles in degrees:\n";

        // X position input
        std::cout << "X position:  ";
        std::cin >> position[0];
        // Y position input
        std::cout << "Y position:  ";
        std::cin >> position[1];
        // Z position input
        std::cout << "Z position:  ";
        std::cin >> position[2];

        // Deg RPY angles input
        std::cout << "Rx: ";
        std::cin >> position[3];
        std::cout << "Ry: ";
        std::cin >> position[4];
        std::cout << "Rz: ";
        std::cin >> position[5];

        geometry_msgs::msg::Pose goal;
        goal.position.x = position[0];
        goal.position.y = position[1];
        goal.position.z = position[2];
        goal.orientation = quaternion_from_euler(position[3], position[4], position[5]);
        waypoints.push_back(goal);
    }

    manipulator_interfaces::msg::TrajectoryResult traj = cartesianPlanAndWait(waypoints);

    if(traj.success)
    {
        int execute = 0;
        std::cout << "Trajectory planned successfully." << std::endl;
        std::cout << "Trajectory points count: " << traj.trajectory.joint_trajectory.points.size() << std::endl;
        std::cout << "Trajectory duration: " << traj.trajectory.joint_trajectory.points.back().time_from_start.sec << "s" << std::endl;
        std::cout << "Do you want to execute the trajectory? 1 for yes: ";
        std::cin >> execute;

        if (execute == 1)
        {
            std::cout << "Executing trajectory..." << std::endl;
            bool success = executeAndWait(traj.trajectory);
            if (success)
            {
                std::cout << "Trajectory executed successfully." << std::endl;
            }
            else
            {
                std::cout << "Trajectory execution failed." << std::endl;
            }
        }
    }
}

// --------------------- LINEAR MOVEMENTS HANDLER ---------------------
//For now the ee will move to the corresponding path but it won't follow a linear path

void ManipulatorMenu::userMoveAlongX()
{
    double x_step = 0.0;
    std::cout << "Enter the step along X axis in meters: \n";
    std::cin >> x_step;
    move_along_x(x_step, true); 
}

void ManipulatorMenu::userMoveAlongY()
{
    double y_step = 0.0;
    std::cout << "Enter the step along Y axis in meters: \n";
    std::cin >> y_step;
    move_along_y(y_step, true);
}

void ManipulatorMenu::userMoveAlongZ()
{
    double z_step = 0.0;
    std::cout << "Enter the step along Z axis in meters: \n";
    std::cin >> z_step;
    move_along_z(z_step, true);
}

// --------------------- ROTATIONS HANDLER ---------------------

void ManipulatorMenu::userMakeTcpRot()
{
    std::vector<double> rot_vec = {0., 0., 0.};
    std::cout << "Enter the rotation angles in degrees for the tcp goal: \n";
    std::cout << "Rx: ";
    std::cin >> rot_vec[0];
    std::cout << "Ry: ";
    std::cin >> rot_vec[1];
    std::cout << "Rz: ";
    std::cin >> rot_vec[2];
    make_tcp_rot(rot_vec);
}

void ManipulatorMenu::userRotateAroundX()
{
    double x_rot_step = 0.0;
    std::cout << "Enter the rotation around X axis in degrees: \n";
    std::cin >> x_rot_step;
    rotate_around_x(x_rot_step);
}

void ManipulatorMenu::userRotateAroundY()
{
    double y_rot_step = 0.0;
    std::cout << "Enter the rotation around Y axis in degrees: \n";
    std::cin >> y_rot_step;
    rotate_around_y(y_rot_step);
}

void ManipulatorMenu::userRotateAroundZ()
{
    double z_rot_step = 0.0;
    std::cout << "Enter the rotation around Z axis in degrees: \n";
    std::cin >> z_rot_step;
    rotate_around_z(z_rot_step);
}


// --------------------- KNOWN POSITIONS HANDLERS ---------------------

void ManipulatorMenu::userGoHomeDown()
{
    planExecuteAndWait("home_gripper_down");
}

void ManipulatorMenu::userGoHomeFront()
{
    planExecuteAndWait("home_gripper_front");
}

void ManipulatorMenu::userGoToKnownPose()
{
    size_t choice = 0;

    std::cout << "Enter the known pose you want to move to:" << std::endl;

    auto it = known_poses_.begin();
    size_t index = 0;
    for (auto it_print = it; it_print != known_poses_.end(); it_print++){
        std::cout << index << " - " << it_print->first << std::endl;
        index++;
    }

    std::cout << "Your choice:";
    std::cin >> choice;

    if (choice < known_poses_.size()){
        std::advance(it, choice);
        std::vector<double> pose = it->second;
        publishJointGoal(pose);
    } else {
        std::cout << "Invalid choice!" << std::endl;
    }
}

// --------------------- VISUALIZATION HANDLERS ---------------------

void ManipulatorMenu::userJointStateVisualizer()
{
    for (unsigned long k = 0; k < params_.joint_names.size(); k++)
    {
        std::cout << "Joint " << k << " : " << current_joint_pose_.position[k] * 180 / M_PI << std::endl;
    }
}

void ManipulatorMenu::userEEPoseVisualizer()
{
    std::vector<double> tcp_pose_rpy = getEEpos_rpy();
    std::cout << "X: " << tcp_pose_rpy[0] << "m - Y: " << tcp_pose_rpy[1] << "m - Z: " << tcp_pose_rpy[2] << "m" << std::endl;
    std::cout << "Rx: " << tcp_pose_rpy[3] << "deg - Ry: " << tcp_pose_rpy[4] << "deg - Rz: " << tcp_pose_rpy[5] << "deg" << std::endl;
}

// --------------------- COLLISION OBJECTS PRIVATE MENU HANDLERS ---------------------

// Function to add a collision object from the user menu
void ManipulatorMenu::userAddCollObj()
{
    std::string name;
    int obj_type;
    std::vector<double> obj_dims;
    double obj_pos[] = {0., 0., 0.};
    double rot_pos[] = {0., 0., 0.};
    std::cout << "Insert following infomation about the obj.\n";
    std::cout << "Name: ";
    std::cin >> name;
    std::cout << "Object type: 1 for BOX, 2 for SPHERE, 3 for CYLINDER, 4 for CONE.\n";
    std::cin >> obj_type;
    // If box chosen
    if (obj_type == 1)
    {
        obj_dims.resize(3, 0.0);
        std::cout << "X dim: ";
        std::cin >> obj_dims[0];
        std::cout << "Y dim: ";
        std::cin >> obj_dims[1];
        std::cout << "Z dim: ";
        std::cin >> obj_dims[2];
    }
    // If sphere chosen
    else if (obj_type == 2)
    {
        obj_dims.resize(1, 0.0);
        std::cout << "Radius: ";
        std::cin >> obj_dims[0];
    }
    // Else
    else if (obj_type == 3 || obj_type == 4)
    {
        obj_dims.resize(2, 0.0);
        std::cout << "Height: ";
        std::cin >> obj_dims[0];
        std::cout << "Radius: ";
        std::cin >> obj_dims[1];
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Invalid object type. Please choose 1, 2, 3 or 4.");
        return;
    }

    std::cout << "Insert position\n";
    std::cout << "X position: ";
    std::cin >> obj_pos[0];
    std::cout << "Y position: ";
    std::cin >> obj_pos[1];
    std::cout << "Z position: ";
    std::cin >> obj_pos[2];
    std::cout << "Insert orientation\n";
    std::cout << "RX rotation: ";
    std::cin >> rot_pos[0];
    std::cout << "RY rotation: ";
    std::cin >> rot_pos[1];
    std::cout << "RZ rotation: ";
    std::cin >> rot_pos[2];

    geometry_msgs::msg::Quaternion rot_quat = quaternion_from_euler(rot_pos[0], rot_pos[1], rot_pos[2]);
    double rot_pos_quat[4] = {rot_quat.x, rot_quat.y, rot_quat.z, rot_quat.w};

    addObj(name, obj_type, obj_dims, obj_pos, rot_pos_quat, moveit_msgs::msg::CollisionObject::ADD);
}

// Function to add a collision object from the user menu
void ManipulatorMenu::userAddAttachedObj()
{
    std::string name;
    int obj_type;
    std::vector<double> obj_dims;
    double obj_pos[] = {0., 0., 0.};
    double rot_pos[] = {0., 0., 0.};
    std::cout << "Insert following infomation about the obj.\n";
    std::cout << "Name: ";
    std::cin >> name;
    std::cout << "Object type: 1 for BOX, 2 for SPHERE, 3 for CYLINDER, 4 for CONE.\n";
    std::cin >> obj_type;
    // If box chosen
    if (obj_type == 1)
    {
        obj_dims = {
            0.,
            0.,
            0.,
        };
        std::cout << "X dim: ";
        std::cin >> obj_dims[0];
        std::cout << "Y dim: ";
        std::cin >> obj_dims[1];
        std::cout << "Z dim: ";
        std::cin >> obj_dims[2];
    }
    // If sphere chosen
    else if (obj_type == 2)
    {
        obj_dims = {0.};
        std::cout << "Radius: ";
        std::cin >> obj_dims[0];
    }
    // Else
    else
    {
        obj_dims = {0., 0.};
        std::cout << "Height: ";
        std::cin >> obj_dims[0];
        std::cout << "Radius: ";
        std::cin >> obj_dims[1];
    }

    std::cout << "Insert position\n";
    std::cout << "X position: ";
    std::cin >> obj_pos[0];
    std::cout << "Y position: ";
    std::cin >> obj_pos[1];
    std::cout << "Z position: ";
    std::cin >> obj_pos[2];
    std::cout << "Insert orientation\n";
    std::cout << "RX rotation: ";
    std::cin >> rot_pos[0];
    std::cout << "RY rotation: ";
    std::cin >> rot_pos[1];
    std::cout << "RZ rotation: ";
    std::cin >> rot_pos[2];

    geometry_msgs::msg::Quaternion rot_quat = quaternion_from_euler(rot_pos[0], rot_pos[1], rot_pos[2]);
    double rot_pos_quat[4] = {rot_quat.x, rot_quat.y, rot_quat.z, rot_quat.w};

    addAttachedObj(name, obj_type, obj_dims, obj_pos, rot_pos_quat, 0);
}

// Function to delete a given collision object from the user menu
void ManipulatorMenu::userDeleteCollObj()
{
    std::string obj_name_loc;
    std::cout << "Insert the name of the object you want to delete:" << std::endl;
    std::cin >> obj_name_loc;
    std::cout << "Insert 1 if the object is attached to the robot, 0 otherwise:" << std::endl;
    int attached;
    std::cin >> attached;
    if (attached == 1)
    {
        removeAttachedObj(obj_name_loc);
    }
    else {
        removeObj(obj_name_loc);
    }
}

// --------------------- CONSTRAINTS HANDLERS ---------------------

void ManipulatorMenu::userAddJointConstraint(){

    uint joint_index = 0;
    double min_position = 0.0;
    double max_position = 0.0;
    double weight = 0.0;

    std::cout << "Enter the joint index [0-5]:";
    std::cin >> joint_index;
    std::cout << "Enter the minimum position (degrees): ";
    std::cin >> min_position;
    std::cout << "Enter the max_position (degrees): ";
    std::cin >> max_position;
    std::cout << "Enter the weight [0-1]: ";
    std::cin >> weight;

    publishJointConstraint(joint_index, min_position, max_position, weight);
}

void ManipulatorMenu::userAddPositionConstraint(){
    std::string link_name;
    uint shape_type;
    geometry_msgs::msg::Pose position;
    std::vector<double> dimensions;
    std::vector<double> rotation_euler(3, 0.0);
    double weight = 0.0;

    std::cout << "Enter the link name: ";
    std::cin >> link_name;
    std::cout << "Shape type type: 1 for BOX, 2 for SPHERE, 3 for CYLINDER, 4 for CONE.\n";
    std::cin >> shape_type;

    if (shape_type == 1)
    {
        dimensions.resize(3, 0.0);
        std::cout << "X dim: ";
        std::cin >> dimensions[0];
        std::cout << "Y dim: ";
        std::cin >> dimensions[1];
        std::cout << "Z dim: ";
        std::cin >> dimensions[2];
    }
    // If sphere chosen
    else if (shape_type == 2)
    {
        dimensions.resize(1, 0.0);
        std::cout << "Radius: ";
        std::cin >> dimensions[0];
    }
    // Else
    else if (shape_type == 3 || shape_type == 4)
    {
        dimensions.resize(2, 0.0);
        std::cout << "Height: ";
        std::cin >> dimensions[0];
        std::cout << "Radius: ";
        std::cin >> dimensions[1];
    } else {
        RCLCPP_ERROR(node_->get_logger(), "Invalid object type. Please choose 1, 2, 3 or 4.");
        return;
    }

    std::cout << "Insert position\n";
    std::cout << "X position: ";
    std::cin >> position.position.x;
    std::cout << "Y position: ";
    std::cin >> position.position.y;
    std::cout << "Z position: ";
    std::cin >> position.position.z;
    std::cout << "Insert orientation\n";
    std::cout << "RX rotation: ";
    std::cin >> rotation_euler[0];
    std::cout << "RY rotation: ";
    std::cin >> rotation_euler[1];
    std::cout << "RZ rotation: ";
    std::cin >> rotation_euler[2];

    position.orientation = quaternion_from_euler(rotation_euler[0], rotation_euler[1], rotation_euler[2]);

    std::cout << "Enter the weight: ";
    std::cin >> weight;

    publishPositionConstraint(link_name, position, shape_type, dimensions, weight);
}

void ManipulatorMenu::userAddOrientationConstraint(){
    std::string link_name;
    std::vector<double> rotation_euler(3, 0.0);
    std::vector<double> tolerances(3, 0.0);
    double weight = 0.0;

    std::cout << "Enter the link name: ";
    std::cin >> link_name;
    std::cout << "Insert orientation\n";
    std::cout << "RX rotation: ";
    std::cin >> rotation_euler[0];
    std::cout << "RY rotation: ";
    std::cin >> rotation_euler[1];
    std::cout << "RZ rotation: ";
    std::cin >> rotation_euler[2];

    std::cout << "Insert tolerances\n";
    std::cout << "RX tolerance ";
    std::cin >> tolerances[0];
    std::cout << "RY tolerance ";
    std::cin >> tolerances[1];
    std::cout << "RZ tolerance ";
    std::cin >> tolerances[2];

    geometry_msgs::msg::Quaternion quaternion = quaternion_from_euler(rotation_euler[0], rotation_euler[1], rotation_euler[2]);

    std::cout << "Enter the weight: ";
    std::cin >> weight;

    publishOrientationConstraint(link_name, quaternion, tolerances, weight);
}

void ManipulatorMenu::userClearConstraints()
{
    publishClearConstraints();
    RCLCPP_INFO(node_->get_logger(), "All constraints cleared.");
}

// --------------------- KINEMATICS QUERIES HANDLERS ---------------------

void ManipulatorMenu::userGetInvKine()
{
    std::vector<double> position = {0., 0., 0., 0., 0., 0.};

    std::cout << "Enter the values of the tcp goal, with rotation angles in degrees:\n";

    // X position input
    std::cout << "X position:  ";
    std::cin >> position[0];
    // Y position input
    std::cout << "Y position:  ";
    std::cin >> position[1];
    // Z position input
    std::cout << "Z position:  ";
    std::cin >> position[2];

    std::cout << "Rx: ";
    std::cin >> position[3];
    std::cout << "Ry: ";
    std::cin >> position[4];
    std::cout << "Rz: ";
    std::cin >> position[5];

    // Get the inverse kinematics
    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = position[0];
    goal_pose.position.y = position[1];
    goal_pose.position.z = position[2];
    goal_pose.orientation = quaternion_from_euler(position[3], position[4], position[5]);

    std::vector<double> inv_kine = invKineClient(goal_pose);

    // Print the result
    std::cout << "The inverse kinematics solution is: \n";
    for (unsigned long k = 0; k < inv_kine.size(); k++)
    {
        std::cout << "Joint " << k << " : " << inv_kine[k] * 180 / M_PI << std::endl;
    }
}

void ManipulatorMenu::userGetJacobian()
{
    Eigen::MatrixXd jacobian = getJacobianClient();
    std::cout << "The Jacobian matrix is: \n";
    printMatrix(jacobian);
}

void ManipulatorMenu::userGetPseudoInv()
{
    Eigen::MatrixXd jacobian = pseudoInverseClient();
    std::cout << "The pseudo-inverse Jacobian matrix is: \n";
    printMatrix(jacobian);
}

// --------------------- PARAMS SETTERS HANDLERS ------------------------

void ManipulatorMenu::userSetJacobianSpeedControl()
{
    bool set;
    std::cout << "Enter 1 to set Jacobian speed control, 0 to unset: \n";
    std::cin >> set;
    setJacobianSpeedControl(set);
}

void ManipulatorMenu::userSetRealTimeControl()
{
    bool set;
    std::cout << "Enter 1 to set joints real time control, 0 to unset: \n";
    std::cin >> set;
    setJsRealTimeControl(set);
}

void ManipulatorMenu::userSetRealTimeConstraints(){
    bool limit_joints, limit_jacobian;

    std::cout << "Enter 1 to enable real time joints constraints, 0 to unset: \n";
    std::cin >> limit_joints;
    std::cout << "Enter 1 to enable real time jacobian constraints, 0 to unset: \n";
    std::cin >> limit_jacobian;

    setRealTimeConstraints(limit_joints, limit_jacobian);
}

void ManipulatorMenu::userSetPlannerScalingFactors()
{
    float new_vel, new_acc;
    std::cout << "Enter the new velocity factor: \n";
    std::cin >> new_vel;
    std::cout << "Enter the new acceleration factor: \n";
    std::cin >> new_acc;
    setPlannerScalingFactors(new_vel, new_acc);
}

void ManipulatorMenu::userSetPlannerTolerances()
{
    float positon, orientation, joint;
    std::cout << "Enter the new position tolerance: \n";
    std::cin >> positon;
    std::cout << "Enter the new orientation tolerance: \n";
    std::cin >> orientation;
    std::cout << "Enter the new joint tolerance: \n";
    std::cin >> joint;
    setPlannerTolerances(positon, orientation, joint);
}

// --------------------- ADMITTANCE CONTROL ---------------------

void ManipulatorMenu::userSetAdmittanceControl()
{
    bool set;
    std::cout << "Enter 1 to set admittance control, 0 to unset: \n";
    std::cin >> set;
    setAdmittanceControl(set);
}

void ManipulatorMenu::userSetAdmittanceControlMode()
{
    bool vel_mode;
    std::cout << "Enter 1 to set velocity mode, 0 to set position mode: \n";
    std::cin >> vel_mode;
    setAdmittanceVelMode(vel_mode);
}

// --------------------- GRIPPER HANDLERS ------------------------

void ManipulatorMenu::userGripperMove()
{
    bool close;
    std::cout << "Enter 1 to close the gripper, 0 to open: \n";
    std::cin >> close;
    moveGripper(close);
}

void ManipulatorMenu::userRunTest(){
    // Test the planner
    std::vector<double> position = {0.5, 0.0, 0.2, 180., 0., 90.};
    moveit_msgs::msg::RobotTrajectory traj = planAndWait(pose_from_vector(position)).trajectory;

    RCLCPP_INFO(node_->get_logger(), "Trajectory planned, executing...");

    if (traj.joint_trajectory.points.size() == 0)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to plan trajectory");
        return;
    }

    bool success = executeAndWait(traj);

    if (!success)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to execute trajectory");
    }

    gripperMoveClient(true); // Grab obj

    rclcpp::sleep_for(std::chrono::seconds(2));

    position = {0.0, 0.5, 0.4, 180., 0., 0.};
    traj = planAndWait(pose_from_vector(position)).trajectory;

    if (traj.joint_trajectory.points.size() == 0)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to plan trajectory");
        return;
    }

    success = executeAndWait(traj);

    if (!success)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to execute trajectory");
    }

    gripperMoveClient(false); // Release obj
}

// --------------------- MENU INITIALIZER ------------------------

void ManipulatorMenu::initializeMenu(){
    menu_ = new MenuUserInterface<ManipulatorMenu>(this);

    int section_start = 0; //Temporary variable to hold the last section start point

    // Fake section to shutdwon the menu
    menu_->addChoice("Shutdown manipulator menu", &ManipulatorMenu::shutdown_handler);
    menu_->addSection("Shutdown", section_start, menu_->last_);
    section_start = menu_->last_ + 1;

    //Joint/TCP Goals
    menu_->addChoice("Plan and execute joint goal", &ManipulatorMenu::userJointGoal);
    menu_->addChoice("Plan and execute one joint move", &ManipulatorMenu::userOneJointMove);
    menu_->addChoice("Plan and execute TCP goal", &ManipulatorMenu::userTcpGoal);
    menu_->addChoice("Plan and execute Cartesian path", &ManipulatorMenu::userCartesianGoal);
    menu_->addSection("Joint/TCP Goals", section_start, menu_->last_);
    section_start = menu_->last_ + 1;

    //Linear movements
    menu_->addChoice("Move along X axis", &ManipulatorMenu::userMoveAlongX);
    menu_->addChoice("Move along Y axis", &ManipulatorMenu::userMoveAlongY);
    menu_->addChoice("Move along Z axis", &ManipulatorMenu::userMoveAlongZ);
    menu_->addSection("Linear movements", section_start, menu_->last_);
    section_start = menu_->last_ + 1;

    //Rotations
    menu_->addChoice("Make TCP rotation", &ManipulatorMenu::userMakeTcpRot);
    menu_->addChoice("Rotate around X axis", &ManipulatorMenu::userRotateAroundX);
    menu_->addChoice("Rotate around Y axis", &ManipulatorMenu::userRotateAroundY);
    menu_->addChoice("Rotate around Z axis", &ManipulatorMenu::userRotateAroundZ);
    menu_->addSection("Rotations", section_start, menu_->last_);
    section_start = menu_->last_ + 1;

    //Known positions
    menu_->addChoice("Go to home position with gripper facing down", &ManipulatorMenu::userGoHomeDown);
    menu_->addChoice("Go to home position with gripper facing front", &ManipulatorMenu::userGoHomeFront);
    menu_->addChoice("Move to known position by id", &ManipulatorMenu::userGoToKnownPose);
    menu_->addSection("Known positions", section_start, menu_->last_);
    section_start = menu_->last_ + 1;

    //Visualization
    menu_->addChoice("Visualize current joint state", &ManipulatorMenu::userJointStateVisualizer);
    menu_->addChoice("Visualize current EE pose", &ManipulatorMenu::userEEPoseVisualizer);
    menu_->addSection("Visualization", section_start, menu_->last_);
    section_start = menu_->last_ + 1;

    //Collision objects
    menu_->addChoice("Add a collision object", &ManipulatorMenu::userAddCollObj);
    menu_->addChoice("Add an attached object", &ManipulatorMenu::userAddAttachedObj);
    menu_->addChoice("Delete a collision object", &ManipulatorMenu::userDeleteCollObj);
    menu_->addSection("Collision objects", section_start, menu_->last_);
    section_start = menu_->last_ + 1;

    //Constraints
    menu_->addChoice("Add joint constraint", &ManipulatorMenu::userAddJointConstraint);
    menu_->addChoice("Add position constraint", &ManipulatorMenu::userAddPositionConstraint);
    menu_->addChoice("Add orientation constraint", &ManipulatorMenu::userAddOrientationConstraint);
    menu_->addChoice("Remove all constraints", &ManipulatorMenu::userClearConstraints);
    menu_->addSection("Constraints", section_start, menu_->last_);
    section_start = menu_->last_ + 1;

    //Kinematics queries
    menu_->addChoice("Get inverse kinematics of a given pose", &ManipulatorMenu::userGetInvKine);
    menu_->addChoice("Get current Jacobian", &ManipulatorMenu::userGetJacobian);
    menu_->addChoice("Get current pseudo-inverse Jacobian", &ManipulatorMenu::userGetPseudoInv);
    menu_->addSection("Kinematics queries", section_start, menu_->last_);
    section_start = menu_->last_ + 1;

    //Setters
    menu_->addChoice("Set Jacobian speed control", &ManipulatorMenu::userSetJacobianSpeedControl);
    menu_->addChoice("Set joints real time control", &ManipulatorMenu::userSetRealTimeControl);
    menu_->addChoice("Set real time constraints", &ManipulatorMenu::userSetRealTimeConstraints);
    menu_->addChoice("Set new planner velocity and acceleration factors", &ManipulatorMenu::userSetPlannerScalingFactors);
    menu_->addChoice("Set new planner tolerances", &ManipulatorMenu::userSetPlannerTolerances);
    menu_->addSection("Setters", section_start, menu_->last_);
    section_start = menu_->last_ + 1;

    //Admittance control
    menu_->addChoice("Enable/disable admittance control", &ManipulatorMenu::userSetAdmittanceControl);
    menu_->addChoice("Set admittance control mode", &ManipulatorMenu::userSetAdmittanceControlMode);
    menu_->addSection("Admittance control", section_start, menu_->last_);
    section_start = menu_->last_ + 1;

    //Gripper
    menu_->addChoice("Move gripper", &ManipulatorMenu::userGripperMove);
    menu_->addSection("Gripper", section_start, menu_->last_);
    section_start = menu_->last_ + 1;

}

// Menu constructor to retrieve parameters from the node
ManipulatorMenuParams::ManipulatorMenuParams(const rclcpp::Node::SharedPtr& node){
    std::string ns = node->get_namespace();
    std::string prefix = ns.empty() ? "" : ns + "_";
  
    //Declareation
    node->declare_parameter("ros_freq", 10.0);
    node->declare_parameter("manipulator_name", "manipulator");
    node->declare_parameter("planning_group", prefix + "ur_manipulator");
    node->declare_parameter("joint_names", std::vector<std::string>{
        prefix + "shoulder_pan_joint", 
        prefix + "shoulder_lift_joint", 
        prefix + "elbow_joint",
        prefix + "wrist_1_joint", 
        prefix + "wrist_2_joint", 
        prefix + "wrist_3_joint"
    });
    node->declare_parameter("base_link_name", prefix + "base_link");
    node->declare_parameter("tcp_position_tolerance", 0.01);
    node->declare_parameter("tcp_orientation_tolerance", 0.01);
    node->declare_parameter("joint_tolerance", 0.01);
    node->declare_parameter("known_poses_path", std::string(""));
    node->declare_parameter("gripper", std::string("no_gripper"));
    node->declare_parameter("gripper_group", std::string("robotiq_85_gripper"));
    node->declare_parameter("gripper_IO_cmds", std::vector<int64_t>{0, 0});

    // Get parameters
    node->get_parameter("ros_freq", ros_freq);
    node->get_parameter("manipulator_name", manipulator_name);
    node->get_parameter("planning_group", planning_group);
    node->get_parameter("joint_names", joint_names);
    node->get_parameter("base_link_name", base_link_name);
    node->get_parameter("tcp_position_tolerance", tcp_position_tolerance);
    node->get_parameter("tcp_orientation_tolerance", tcp_orientation_tolerance);
    node->get_parameter("joint_tolerance", joint_tolerance);
    node->get_parameter("gripper", gripper);
    node->get_parameter("gripper_group", gripper_group);
    node->get_parameter("gripper_IO_cmds", gripper_IO_cmds);

    std::string known_poses_path_rel;
    std::string share_dir = ament_index_cpp::get_package_share_directory("manipulators");
    node->get_parameter("known_poses_path", known_poses_path_rel);
    known_poses_path = share_dir + "/" + known_poses_path_rel;
}
