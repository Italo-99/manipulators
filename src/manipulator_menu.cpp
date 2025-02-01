// IMPORT LIBRARIES
#include "manipulators/ManipulatorMenu.h"

// --------------------- PUBLIC CONSTRUCTOR ---------------------

ManipulatorMenu::ManipulatorMenu(const std::string node_name, const rclcpp::NodeOptions &options) : rclcpp::Node(node_name, options)
{

    declareParameters();

    std::vector<std::string> joint_names = this->get_parameter("joint_names").as_string_array();
    std::string manipulator_name = this->get_parameter("manipulator_name").as_string();
    std::string ee_joint_name = this->get_parameter("ee_joint_name").as_string();

    // Display Manipulator
    RCLCPP_INFO(get_logger(), "Manipulator menu initialized with the following setup:");
    RCLCPP_INFO(get_logger(), "Manipulator name: %s", manipulator_name.c_str());

    for (unsigned long k = 0; k< joint_names.size(); k++)
    {
        RCLCPP_INFO(get_logger(), "Joint %ld name: %s", k, joint_names[k].c_str());
    }

    // Init arrays
    for (const std::string& name : joint_names) {
        joints_map_group_[name] = 0.;
    }

    joints_values_group_.resize(joint_names.size());
    current_joint_pose_.name      = joint_names;

    // --------------------- PUBS & SUBS DELCARATIONS ---------------------
    jointGoalPublisher_           = this->create_publisher<sensor_msgs::msg::JointState>(manipulator_name+"/desired_joint_pose", 1);
    tcpPosePublisher_             = this->create_publisher<geometry_msgs::msg::Pose>(manipulator_name+"/desired_tcp_pose", 1);
    tcpPoseIKPublisher_           = this->create_publisher<geometry_msgs::msg::Pose>(manipulator_name+"/desired_tcpIK_pose", 1);
    carthesianMovePublisher_      = this->create_publisher<geometry_msgs::msg::PoseArray>(manipulator_name+"/desired_cartesian_move", 1);
    display_goal_pub_             = this->create_publisher<geometry_msgs::msg::PoseStamped>(manipulator_name+"/display_robot_goal", 1);
    eepose_pub_                   = this->create_publisher<geometry_msgs::msg::PoseStamped>(manipulator_name+"/display_ee_pose", 1);
    collisionObjectPublisher_     = this->create_publisher<moveit_msgs::msg::CollisionObject>(manipulator_name+"/add_collision_object", 1);
    collisionAttObjectPublisher_  = this->create_publisher<moveit_msgs::msg::AttachedCollisionObject>(manipulator_name+"/add_attached_object", 1);
    moveGripperPublisher_         = this->create_publisher<std_msgs::msg::Float64>(ee_joint_name+"/motor_control", 1);

    jointStateSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 1, 
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->jointStateCallback(msg);
        }
    );

    // --------------------- Kinematics client init ---------------------
    invKineClient_                = this->create_client<manipulator_interfaces::srv::InvKine>(manipulator_name+"/invKine");
    pseudoInvClient_              = this->create_client<manipulator_interfaces::srv::PseudoInverse>(manipulator_name+"/pseudoInverse");
    fKineClient_                  = this->create_client<manipulator_interfaces::srv::FKine>(manipulator_name+"/FKine");
    jacobianClient_               = this->create_client<manipulator_interfaces::srv::Jacobian>(manipulator_name+"/Jacobian");

    setInstKineClient_            = this->create_client<std_srvs::srv::SetBool>(manipulator_name+"/instKine_setter");
    setJacobianControlClient_     = this->create_client<std_srvs::srv::SetBool>(manipulator_name+"/jacobian_control_setter");
    setRealTimeControlClient_     = this->create_client<std_srvs::srv::SetBool>(manipulator_name+"/joints_real_time_setter");
    plannerParamsClient_          = this->create_client<manipulator_interfaces::srv::ChangePlannerParameters>(manipulator_name+"/change_planner_params");

    // --------------------- CoppeliaSim client init ---------------------
    if (this->get_parameter("enable_coppelia").as_bool())
    {
        coppeliaClient_ = this->create_client<manipulator_interfaces::srv::CoppeliaMenu>("coppelia_menu");
    }   

    // --------------------- Gripper client init ---------------------
    if (ee_joint_name != "" && get_parameter("enable_sim_gripper").as_bool() == true)
    {
        grab_client_    = this->create_client<std_srvs::srv::SetBool>(ee_joint_name+"/grabbing_gripper");
        gripper_client_ = this->create_client<std_srvs::srv::SetBool>(ee_joint_name+"/move_gripper");

        if (get_parameter("enable_real_gripper").as_bool()){
            real_gripper_client_ = this->create_client<motors_trajectory::srv::RobotiQGripperControl>(get_parameter("gripper_topic").as_string());
        }
    }
}

// --------------------- PUBLIC FUNCTIONS ---------------------

std::vector<double> ManipulatorMenu::invKineClient(const geometry_msgs::msg::Pose pose)
{
    std::vector<double> joint_values;

    // Set target pose
    manipulator_interfaces::srv::InvKine::Request::SharedPtr request;
    request->target_pose = pose;

    while (!invKineClient_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return joint_values;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "invKine service not available, waiting again...");
    }

    auto response = invKineClient_->async_send_request(request);

    // Call the srv
    if (rclcpp::spin_until_future_complete(shared_from_this(), response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        for (unsigned long k = 0; k < response.get()->joint_values.size(); k++)
        {
            joint_values.push_back(response.get()->joint_values[k]);
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service invKine");
    }

    return joint_values;
}

Eigen::MatrixXd ManipulatorMenu::pseudoInverseClient()
{
    std::vector<std::string> joint_names = this->get_parameter("joint_names").as_string_array(); 
    Eigen::MatrixXd matrix(joint_names.size(), 6);

    manipulator_interfaces::srv::PseudoInverse::Request::SharedPtr request;

    while (!pseudoInvClient_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return matrix;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pseudoInverse service not available, waiting again...");
    }

    auto response = pseudoInvClient_->async_send_request(request);


    if (rclcpp::spin_until_future_complete(shared_from_this(), response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        // RCLCPP_INFO(get_logger(), "Pseudoinverse matrix received:");
        // Assign data from Float64[] to Eigen::MatrixXd
        for (unsigned long i = 0; i < joint_names.size(); ++i)
        {
            for (int j = 0; j < 6; ++j)
            {
                matrix(i, j) = response.get()->matrix_values[i * joint_names.size() + j];
            }
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service pseudoInverse");
    }

    return matrix;
}

geometry_msgs::msg::Pose ManipulatorMenu::getCurrentFKineClient()
{
    geometry_msgs::msg::Pose pose;

    manipulator_interfaces::srv::FKine::Request::SharedPtr request;

    while (!fKineClient_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return pose;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "fKine service not available, waiting again...");
    }

    auto response = fKineClient_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(shared_from_this(), response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        // RCLCPP_INFO(get_logger(), "Forward Kinematics Pose received:");
        // ROS_INFO_STREAM(fKine_srv_.response.tcp_pose);
        pose = response.get()->tcp_pose.pose;
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service getCurrentFKine");
    }
    return pose;
}

Eigen::MatrixXd ManipulatorMenu::getJacobianClient()
{
    std::vector<std::string> joint_names = this->get_parameter("joint_names").as_string_array(); 
    Eigen::MatrixXd matrix(joint_names.size(), 6);

    manipulator_interfaces::srv::Jacobian::Request::SharedPtr request;

    while (!jacobianClient_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return matrix;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "jacobian service not available, waiting again...");
    }

    auto response = jacobianClient_->async_send_request(request);


    if (rclcpp::spin_until_future_complete(shared_from_this(), response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        // RCLCPP_INFO(get_logger(), "jacobian matrix received:");
        // Assign data from Float64[] to Eigen::MatrixXd
        for (unsigned long i = 0; i < joint_names.size(); ++i)
        {
            for (int j = 0; j < 6; ++j)
            {
                matrix(i, j) = response.get()->matrix_values[i * joint_names.size() + j];
            }
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service jacobian");
    }

    return matrix;
}

// --------------------- ROS HANDLER ---------------------

// Asynchronous spinner for ROS routines without user menu
void ManipulatorMenu::spinner()
{
    // Setup a rate for ROS loop execution
    rclcpp::Rate r(this->get_parameter("spinner_rate").as_int());

    // ROS loop
    while (rclcpp::ok())
    {
        // ROS spinner
        rclcpp::spin_some(shared_from_this());
        getEEpose();

        // Test funtion for InvKine computations time measurement: about 10-20 ms, not acceptable
        // ros::Time start = ros::Time::now();
        // invKineClient(getCurrentFKineClient());
        // getCurrentFKineClient();
        // pseudoInverseClient();
        // RCLCPP_INFO(get_logger(), "Total duration of the computations: %f", ros::Time::now().toSec()-start.toSec());

        // Wait for next loop time
        r.sleep();
    }

    // Shutdown ROS if Ctrl+C or Ctrl+D are pressed
    rclcpp::shutdown();
}

// Asynchronous spinner for ROS routines with user menu
void ManipulatorMenu::spinnerMenu()
{
    // Initialize user choice variable
    int userChoice = 0;

    while (rclcpp::ok())
    {
        rclcpp::spin_some(shared_from_this());          // ROS Once spinner
        getEEpos_rpy();                                 // Update current robot pose
        printMenu();                                    // Print choice menu
        userChoice = getUserChoice();                   // Get user choice from the terminal
        processChoice(userChoice);                      // Execute the command
        rclcpp::sleep_for(std::chrono::seconds(1));     // Wait 1s until next command
    }

    RCLCPP_INFO(get_logger(), "Closing the menu!\n");
    rclcpp::shutdown();
}

// --------------------- COPPELIASIM HANDLER ---------------------

// Open Coppelia simulation
void ManipulatorMenu::startCoppeliaSim()
{
    coppelia_req_->command = 0;
    wait_for_response();
}

// Close Coppelia simulation
void ManipulatorMenu::stopCoppeliaSim()
{
    coppelia_req_->command = 1;
    wait_for_response();
}

// Save Coppelia scene
void ManipulatorMenu::saveCoppeliaScene()
{
    coppelia_req_->command = 2;
    wait_for_response();
}

// --------------------- MOVEMENTS HANDLER ---------------------

// Publish a joint goal by passing a vector of joints in deg
sensor_msgs::msg::JointState ManipulatorMenu::publishJointGoal(const std::vector<double> joints)
{
    // Fill the joint msg with degToRad conversion
    sensor_msgs::msg::JointState jointStateMsg;
    jointStateMsg.header.stamp = get_clock()->now();
    for (unsigned long k = 0; k < joints.size(); k++)
    {
        jointStateMsg.position.push_back(joints[k] * M_PI / 180);
    }

    return publishJointGoal(jointStateMsg);
}

// Publish a joint goal by passing a JointState msg
sensor_msgs::msg::JointState ManipulatorMenu::publishJointGoal(const sensor_msgs::msg::JointState jointStateMsg)
{
    // Publish the JointState message
    jointGoalPublisher_->publish(jointStateMsg);
    return jointStateMsg;
}

// Publish a Tcp goal by passing a vector (rotations must be expressed in deg)
geometry_msgs::msg::Pose ManipulatorMenu::publishTcpGoal(const std::vector<double> position)
{
    geometry_msgs::msg::Pose tcpPoseMsg;

    tcpPoseMsg.position.x = position[0];
    tcpPoseMsg.position.y = position[1];
    tcpPoseMsg.position.z = position[2];

    // Conversion from euler rotation to pose quaternion
    tcpPoseMsg.orientation = quaternion_from_euler(position[3], position[4], position[5]);

    return publishTcpGoal(tcpPoseMsg);
}

// Publish a Tcp goal by passing a geometry_msgs::msg::Pose
geometry_msgs::msg::Pose ManipulatorMenu::publishTcpGoal(const geometry_msgs::msg::Pose tcpPoseMsg)
{
    tcpPosePublisher_->publish(tcpPoseMsg);

    // Display the goal on RViz
    geometry_msgs::msg::PoseStamped robot_goal_msg;
    robot_goal_msg.header.frame_id = get_parameter("base_link_name").as_string();
    robot_goal_msg.header.stamp = get_clock()->now();
    robot_goal_msg.pose = tcpPoseMsg,

    display_goal_pub_->publish(robot_goal_msg);

    return tcpPoseMsg;
}

// Publish a TcpIK goal by passing a vector (rotations must be expressed in deg)
geometry_msgs::msg::Pose ManipulatorMenu::publishTcpIKGoal(const std::vector<double> position)
{
    // TCP pose
    geometry_msgs::msg::Pose tcpPoseMsg;
    tcpPoseMsg.position.x = position[0];
    tcpPoseMsg.position.y = position[1];
    tcpPoseMsg.position.z = position[2];

    tcpPoseMsg.orientation = quaternion_from_euler(position[3], position[4], position[5]);

    return publishTcpIKGoal(tcpPoseMsg);
}

// Publish a TcpIK goal by passing a geometry_msgs::msg::Pose
geometry_msgs::msg::Pose ManipulatorMenu::publishTcpIKGoal(const geometry_msgs::msg::Pose tcpPoseMsg)
{
    // Plan trajectory through inverse kinematics
    tcpPoseIKPublisher_->publish(tcpPoseMsg);

    // Display the goal on RViz
    geometry_msgs::msg::PoseStamped robot_goal_msg;
    robot_goal_msg.header.frame_id = get_parameter("base_link_name").as_string();
    robot_goal_msg.header.stamp = get_clock()->now();
    robot_goal_msg.pose = tcpPoseMsg;

    display_goal_pub_->publish(robot_goal_msg);

    return tcpPoseMsg;
}

// Publish a cartesian goal of poses sequence along the same line
// Specify axis1 and axis2 as 0 for x, 1 for y and 2 for z
// Define pos1 and pos2 the final poses along those axis (the 3rd won't change)
// steps value will define how many waypoints to put in
geometry_msgs::msg::Pose ManipulatorMenu::publishCartesianMove(const uint axis1,
                                                               const uint axis2,
                                                               const double pos1,
                                                               const double pos2,
                                                               const uint steps)
{
    // Initialize starting and waypoints variables
    geometry_msgs::msg::Pose current_pose = getCurrentFKineClient();
    geometry_msgs::msg::PoseArray waypoints;
    geometry_msgs::msg::Pose final_pose;

    waypoints.header.frame_id = get_parameter("base_link_name").as_string();
    double step_axisX = 0.;
    double step_axisY = 0.;
    double step_axisZ = 0.;
    // Compute axis step
    if (axis1 == axis2)
    {
        RCLCPP_WARN(get_logger(), "Error in axis input!");
        return final_pose;
    }
    else
    {
        if (axis1 == 0)
        {
            step_axisX = pos1 - current_pose.position.x;
        }
        else if (axis1 == 1)
        {
            step_axisY = pos1 - current_pose.position.y;
        }
        else if (axis1 == 2)
        {
            step_axisZ = pos1 - current_pose.position.z;
        }
        if (axis2 == 0)
        {
            step_axisX = pos2 - current_pose.position.x;
        }
        else if (axis2 == 1)
        {
            step_axisY = pos2 - current_pose.position.y;
        }
        else if (axis2 == 2)
        {
            step_axisZ = pos2 - current_pose.position.z;
        }
    }
    // Compute common step
    step_axisX = step_axisX / static_cast<double>(steps);
    step_axisY = step_axisY / static_cast<double>(steps);
    step_axisZ = step_axisZ / static_cast<double>(steps);
    // Fill waypoints msgs
    for (unsigned long k = 0; k < steps; k++)
    {
        geometry_msgs::msg::Pose wp;
        // Same orientation for every waypoint
        wp.orientation = current_pose.orientation;
        // Fill the position
        wp.position.x = current_pose.position.x + (k + 1) * step_axisX;
        wp.position.y = current_pose.position.y + (k + 1) * step_axisY;
        wp.position.z = current_pose.position.z + (k + 1) * step_axisZ;
        // Last position should be accurate
        if (k == steps - 1)
        {
            if (axis1 == 0)
            {
                wp.position.x = pos1;
            }
            else if (axis2 == 0)
            {
                wp.position.x = pos2;
            }
            if (axis1 == 1)
            {
                wp.position.y = pos1;
            }
            else if (axis2 == 1)
            {
                wp.position.y = pos2;
            }
            if (axis1 == 2)
            {
                wp.position.z = pos1;
            }
            else if (axis2 == 2)
            {
                wp.position.z = pos2;
            }
            final_pose = wp;
        }
        // Add current computed waypoints to the vector
        waypoints.poses.push_back(wp);
    }
    // Publish the msg
    carthesianMovePublisher_->publish(waypoints);

    return final_pose;
}

// Move a single joint, joint rotation must be in deg
sensor_msgs::msg::JointState ManipulatorMenu::oneJointMove(const int num, const double joint_rot)
{
    // Read from subscribers the current joints state
    rclcpp::spin_some(shared_from_this());
    // Fill current joints pose as target
    std::vector<double> joint_target;
    for (unsigned long k = 0; k < get_parameter("joint_names").as_string_array().size(); k++)
    {
        joint_target.push_back(current_joint_pose_.position[k] * 180 / M_PI);
    }
    // Change the joint target position
    joint_target[num] = joint_target[num] + joint_rot;
    return publishJointGoal(joint_target);
}

// Go to pre configured home position
sensor_msgs::msg::JointState ManipulatorMenu::goHome(const bool ee_orient)
{
    std::vector<double> start_joint_pose = {0., 0., 0., 0., 0., 0};
    if (!ee_orient) // gripper down
    {
        start_joint_pose = {0., -90., -90., -90., +90., 0.};
    }
    else // gripper at the front
    {
        start_joint_pose = {0., -90., -90., 0., +90., 0.};
    }
    if (get_parameter("joint_names").as_string_array().size() != 6)
    {
        for (unsigned long k = 0; k < get_parameter("joint_names").as_string_array().size() - 6; k++)
        {
            start_joint_pose.push_back(0.);
        }
    }

    // Publish home joint goal
    return publishJointGoal(start_joint_pose);
}

// -------------------- TF END EFFECTOR LISTENER -----------------------//

// Listen a TF between two given frames
geometry_msgs::msg::PoseStamped ManipulatorMenu::getTf(const std::string &source_frame, const std::string &target_frame)
{
    // Create a TF2 buffer and listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // Wait for the transformation to be available
    try
    {
        tf_buffer->canTransform(source_frame, target_frame, rclcpp::Time(0), rclcpp::Duration(std::chrono::milliseconds(200)));
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(get_logger(), "%s", ex.what());
    }

    // Get the transformation
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
        transformStamped = tf_buffer->lookupTransform(source_frame, target_frame, rclcpp::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(get_logger(), "%s", ex.what());
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    // Convert the tf msg into a PoseStampedrclcp
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = source_frame;
    target_pose.header.stamp = get_clock()->now();
    target_pose.pose.position.x = transformStamped.transform.translation.x;
    target_pose.pose.position.y = transformStamped.transform.translation.y;
    target_pose.pose.position.z = transformStamped.transform.translation.z;
    target_pose.pose.orientation = transformStamped.transform.rotation;

    return target_pose;
}

// Get current EE pose
geometry_msgs::msg::Pose ManipulatorMenu::getEEpose()
{
    // Compute the FKine between base_link and end-effector
    current_tcp_pose_.header.frame_id = get_parameter("base_link_name").as_string();
    current_tcp_pose_.pose = getCurrentFKineClient();
    eepose_pub_->publish(current_tcp_pose_);
    return current_tcp_pose_.pose;
}

// Get EE pose as vector with RPY euler angles
std::vector<double> ManipulatorMenu::getEEpos_rpy()
{
    // Read current EE pose by FKine
    getEEpose();

    // Fill the rotation vector
    std::vector<double> tcp_rpy = euler_from_quaternion(current_tcp_pose_.pose.orientation);

    // Declaration of the pose vector
    std::vector<double> tcp_pose_rpy = {current_tcp_pose_.pose.position.x, current_tcp_pose_.pose.position.y, current_tcp_pose_.pose.position.z, tcp_rpy[0], tcp_rpy[1], tcp_rpy[2]};
    return tcp_pose_rpy;
}

// -------------------- SIMPLE MOVES ALONG CARTHESIAN AXES -----------------------//

// Set a carthesian move along x axis in metres
geometry_msgs::msg::Pose ManipulatorMenu::move_along_x(const double x_step, bool cartesian)
{
    // Get current EE pose
    std::vector<double> goal_pose = getEEpos_rpy();
    // Update position along X
    goal_pose[0] = goal_pose[0] + x_step;
    if (cartesian)
    {
        uint8_t n_steps = std::max(int(x_step / 0.1), 1);
        return publishCartesianMove(0, 1, goal_pose[0], goal_pose[1], n_steps);
    }
    else
    {
        return publishTcpGoal(goal_pose);
    }
}

// Set a carthesian move along x axis in metres
geometry_msgs::msg::Pose ManipulatorMenu::move_along_y(const double y_step, bool cartesian)
{
    // Get current EE pose
    std::vector<double> goal_pose = getEEpos_rpy();
    // Update position along Y
    goal_pose[1] = goal_pose[1] + y_step;
    if (cartesian)
    {
        uint8_t n_steps = std::max(int(y_step / 0.1), 1);
        return publishCartesianMove(0, 1, goal_pose[0], goal_pose[1], n_steps);
    }
    else
    {
        return publishTcpGoal(goal_pose);
    }
}

// Set a carthesian move along x axis in metres
geometry_msgs::msg::Pose ManipulatorMenu::move_along_z(const double z_step, bool cartesian)
{
    // Get current EE pose
    std::vector<double> goal_pose = getEEpos_rpy();
    goal_pose[2] = goal_pose[2] + z_step;
    // Update position along Z
    if (cartesian)
    {
        uint8_t n_steps = std::max(int(z_step / 0.1), 1);
        return publishCartesianMove(0, 2, goal_pose[0], goal_pose[2], n_steps);
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

// --------------------- COLLISION OBJECTS HANDLER ---------------------

// Create a collision object from a selected primitive
void ManipulatorMenu::addObj(const std::string &name,
                             const int obj_type,
                             std::vector<double> obj_dims,
                             double obj_pos[],
                             double rot_pos[],
                             uint operation)
{
    // Creation of the obj
    moveit_msgs::msg::CollisionObject obj;

    obj.header.frame_id = get_parameter("base_link_name").as_string();
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
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "obj_dims array is not compatible with obj_type");
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
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "obj_dims array is not compatible with obj_type");
        }
        else
        { // Set the sphere radius
            obj.primitives[0].dimensions[0] = obj_dims[0];
        }
        break;

    default: // CYLINDER OR CONE
        if (size_obj_dims != 2)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "obj_dims array is not compatible with obj_type");
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
    obj.primitive_poses.resize(1);
    obj.primitive_poses[0].position.x = obj_pos[0];
    obj.primitive_poses[0].position.y = obj_pos[1];
    obj.primitive_poses[0].position.z = obj_pos[2];

    // Set obj orientation
    obj.primitive_poses[0].orientation.x = rot_pos[0];
    obj.primitive_poses[0].orientation.y = rot_pos[1];
    obj.primitive_poses[0].orientation.z = rot_pos[2];
    obj.primitive_poses[0].orientation.w = rot_pos[3];

    publishCollisionObject(obj);
}

// Collision object publisher
void ManipulatorMenu::publishCollisionObject(const moveit_msgs::msg::CollisionObject collisionObjectMsg)
{
    collisionObjectPublisher_->publish(collisionObjectMsg);
}

// Create a collision object from a selected primitive
void ManipulatorMenu::addAttachedObj(const std::string &name,
                                     const int obj_type,
                                     std::vector<double> obj_dims,
                                     double obj_pos[],
                                     double rot_pos[],
                                     uint operation)
{
    // Creation of the obj
    moveit_msgs::msg::CollisionObject obj;

    obj.header.frame_id = get_parameter("base_link_name").as_string();
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
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "obj_dims array is not compatible with obj_type");
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
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "obj_dims array is not compatible with obj_type");
        }
        else
        { // Set the sphere radius
            obj.primitives[0].dimensions[0] = obj_dims[0];
        }
        break;

    default: // CYLINDER OR CONE
        if (size_obj_dims != 2)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "obj_dims array is not compatible with obj_type");
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
    obj.primitive_poses.resize(1);
    obj.primitive_poses[0].position.x = obj_pos[0];
    obj.primitive_poses[0].position.y = obj_pos[1];
    obj.primitive_poses[0].position.z = obj_pos[2];

    // Set obj orientation
    obj.primitive_poses[0].orientation.x = rot_pos[0];
    obj.primitive_poses[0].orientation.y = rot_pos[1];
    obj.primitive_poses[0].orientation.z = rot_pos[2];
    obj.primitive_poses[0].orientation.w = rot_pos[3];

    // Set the attached object
    moveit_msgs::msg::AttachedCollisionObject attachedObj;
    attachedObj.object = obj;
    publishAttachedCollisionObject(attachedObj);
}

// Collision Attached object publisher
void ManipulatorMenu::publishAttachedCollisionObject(const moveit_msgs::msg::AttachedCollisionObject collisionAttachedObjectMsg)
{
    collisionAttObjectPublisher_->publish(collisionAttachedObjectMsg);
}

// --------------------- GRIPPER CONTROL ---------------------

// Open the gripper
void ManipulatorMenu::openGripper()
{
    callGripperSrv(false);
}

// Close the gripper
void ManipulatorMenu::closeGripper()
{
    callGripperSrv(true);
}

// Move the gripper
void ManipulatorMenu::moveGripper(const double gripper_position)
{
    std_msgs::msg::Float64 gripper_pos_msg;
    gripper_pos_msg.data = gripper_position;
    moveGripperPublisher_->publish(gripper_pos_msg);
}

// Grab an object at the gripper
void ManipulatorMenu::grabObjGripper()
{
    callGrabbingSrv(true);
}
// Detach an object from the gripper
void ManipulatorMenu::detachObjGripper()
{
    callGrabbingSrv(false);
}

// Open real gripper
void ManipulatorMenu::openRealGripper()
{
    callRealGripperSrv(100.);
}

// Close real gripper
void ManipulatorMenu::closeRealGripper()
{
    callRealGripperSrv(0.);
}

// Move real gripper (input is in range [0,100])
void ManipulatorMenu::moveRealGripper(const float command)
{
    callRealGripperSrv(command);
}

// ------------------- KINEMATICS PARAMS SETTERS ---------------------- //

// Set Jacobian-based speed control
void ManipulatorMenu::setJacobianSpeedControl(bool set)
{
    std_srvs::srv::SetBool::Request::SharedPtr request;
    request->data = set;
    
    //Wait for srv
    while (!setJacobianControlClient_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "jacobian_control_setter service not available, waiting again...");
    }

    auto response = setJacobianControlClient_->async_send_request(request);

    // Call the srv
    if (rclcpp::spin_until_future_complete(shared_from_this(), response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (response.get()->success){
            RCLCPP_INFO(get_logger(), "Jacobian control set to %d", set);
        }
        else{
            RCLCPP_ERROR(get_logger(), "Failed to set Jacobian control");
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service jacobian_control_setter");
    }
}

// Set Instantaneous kinematics mode
void ManipulatorMenu::setInstantKineMode(bool set)
{
    std_srvs::srv::SetBool::Request::SharedPtr request;
    request->data = set;
    
    //Wait for srv
    while (!setInstKineClient_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "instKine_setter service not available, waiting again...");
    }

    auto response = setJacobianControlClient_->async_send_request(request);

    // Call the srv
    if (rclcpp::spin_until_future_complete(shared_from_this(), response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (response.get()->success){
            RCLCPP_INFO(get_logger(), "instKine set to %d", set);
        }
        else{
            RCLCPP_ERROR(get_logger(), "Failed to set instKine");
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service instKine_setter");
    }
}

// Set new dynamic planners vel/acc params
void ManipulatorMenu::setNewPlannerParams(float new_vel, float new_acc)
{
    manipulator_interfaces::srv::ChangePlannerParameters::Request::SharedPtr request;
    request->acc_factor = new_acc;
    request->vel_factor = new_vel;
    
    //Wait for srv
    while (!plannerParamsClient_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "change_planner_params service not available, waiting again...");
    }

    auto response = plannerParamsClient_->async_send_request(request);

    // Call the srv
    if (rclcpp::spin_until_future_complete(shared_from_this(), response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (response.get()->success){
            RCLCPP_INFO(get_logger(), "Planner params changed: acc_factor = %f, vel_factor = %f", new_acc, new_vel);
        }
        else{
            RCLCPP_ERROR(get_logger(), "Failed to set planner params");
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service change_planner_params");
    }
}

// Set Joints real time speed control
void ManipulatorMenu::setJsRealTimeControl(bool set)
{
    std_srvs::srv::SetBool::Request::SharedPtr request;
    request->data = set;
    
    //Wait for srv
    while (!setRealTimeControlClient_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "joints_real_time_setter service not available, waiting again...");
    }

    auto response = setJacobianControlClient_->async_send_request(request);

    // Call the srv
    if (rclcpp::spin_until_future_complete(shared_from_this(), response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (response.get()->success){
            RCLCPP_INFO(get_logger(), "Joints real time control set to %d", set);
        }
        else{
            RCLCPP_ERROR(get_logger(), "Failed to set joints real time control");
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service joints_real_time_setter");
    }
}

// --------------------- PRIVATE FUNCTIONS ---------------------

void ManipulatorMenu::declareParameters() {
    this->declare_parameter("manipulator_name", std::string());
    this->declare_parameter("joint_names", std::vector<std::string>());
    this->declare_parameter("ee_joint_name", "");
    this->declare_parameter("base_link_name", "base_link");
    this->declare_parameter("ros_freq", 500);
    this->declare_parameter("enable_coppelia", false);
    this->declare_parameter("enable_sim_gripper", false);
    this->declare_parameter("enable_real_gripper", false);
    this->declare_parameter("gripper_topic", "/ur_rtde/robotiq_gripper/command");
}

// --------------------- COPPELIA HANDLER ---------------------

// Send the request to coppelia and wait for the response
void ManipulatorMenu::wait_for_response()
{
    while(coppeliaClient_->wait_for_service(std::chrono::seconds(1))){
        if (!rclcpp::ok()){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "coppelia service not available, waiting again...");
    }

    auto response = coppeliaClient_->async_send_request(coppelia_req_);

    // Call the srv
    if (rclcpp::spin_until_future_complete(shared_from_this(), response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(get_logger(), "Simulation status: %d", response.get()->result);
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service invKine");
    }
}

// --------------------- JOINT GOALS HANDLER ---------------------

void ManipulatorMenu::userJointGoal()
{
    // Declare the empty vector of joints goals
    std::vector<double> joints;

    // Take user degree angle for each joint
    std::cout << "Enter the values of the joint goal in degrees: \n";

    for (unsigned long k = 0; k < get_parameter("joint_names").as_string_array().size(); k++)
    {
        double new_joint_value = 0.;
        std::cout << "Joint " << k + 1 << " : ";
        std::cin >> new_joint_value;
        joints.push_back(new_joint_value);
    }

    publishJointGoal(joints);
}

void ManipulatorMenu::oneJointMove_user()
{
    int num = 0;
    double joint_rot = 0.0;
    std::cout << "Enter the joint to move in [0, " << get_parameter("joint_names").as_string_array().size() - 1 << "]: \n";
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

    publishTcpGoal(position);
}

void ManipulatorMenu::userTcpIKGoal()
{
    // Declare the empty vector of joints goals
    std::vector<double> position = {0., 0., 0., 0., 0., 0.};

    // Take user degree angle for each joint
    std::cout << "Enter the values of the tcp goal through InvKine, with rotation angles in degrees:\n";

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

    publishTcpIKGoal(position);
}

// --------------------- USER CARTESIAN MOVES HANDLER ---------------------
void ManipulatorMenu::userCartesianMove()
{
    RCLCPP_INFO(get_logger(), "Setup your cartesian move:");
    uint axis1;
    uint axis2;
    double pos1;
    double pos2;
    uint steps;
    std::cout << "Insert the first axis  (0:x, 1:y, 2:z): ";
    std::cin >> axis1;
    std::cout << "Insert the second axis (0:x, 1:y, 2:z): ";
    std::cin >> axis2;
    std::cout << "Insert the final position on axis1    : ";
    std::cin >> pos1;
    std::cout << "Insert the final position on axis2    : ";
    std::cin >> pos2;
    std::cout << "Set the number of waypoints passed    : ";
    std::cin >> steps;
    publishCartesianMove(axis1, axis2, pos1, pos2, steps);
}

// --------------------- SUBS HANDLER ---------------------

void ManipulatorMenu::jointStateVisualizer()
{
    rclcpp::spin_some(shared_from_this());
    for (unsigned long k = 0; k < get_parameter("joint_names").as_string_array().size(); k++)
    {
        std::cout << "Joint " << k << " : " << current_joint_pose_.position[k] * 180 / M_PI << std::endl;
    }
}

void ManipulatorMenu::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr &joints_state)
{

    // Map to store couples joint name - joint values
    std::vector<std::string> joint_names = get_parameter("joint_names").as_string_array();
    static std::unordered_map<std::string, double>::iterator it;
    uint counter_group = 0;

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
            if (counter_group == joint_names.size())
            {
                // Iterate over the joints
                for (unsigned long k = 0; k < joint_names.size(); k++)
                {
                    // Store the joints values from the joints map
                    joints_values_group_[k] = joints_map_group_[joint_names[k]];
                }

                // Log gripper planning group
                RCLCPP_INFO_ONCE(get_logger(), "%s joints values received by the menu interface.", get_parameter("manipulator_name").as_string().c_str());
            }
        }
    }

    // Store the value into the global (public) class variable
    current_joint_pose_.position = joints_values_group_;
}

// --------------------- COLLISION OBJECTS PRIVATE MENU HANDLER ---------------------

// Function to add a collision object from the user menu
void ManipulatorMenu::addCollObj()
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
        std::cout << "X dim: ";
        std::cin >> obj_dims[0];
    }
    // Else
    else
    {
        obj_dims = {0., 0.};
        std::cout << "X dim: ";
        std::cin >> obj_dims[0];
        std::cout << "Y dim: ";
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

    addObj(name, obj_type, obj_dims, obj_pos, rot_pos_quat, 0);
}

// Function to delete a given collision object from the user menu
void ManipulatorMenu::deleteCollObj()
{
    std::string obj_name_loc;
    std::cout << "Insert the name of the object you want to delete:" << std::endl;
    std::cin >> obj_name_loc;
    std::cout << "Insert 1 if the object is attached to the robot, 0 otherwise:" << std::endl;
    int attached;
    std::cin >> attached;
    std::vector<double> obj_dim_loc = {0., 0., 0.};
    double obj_pos_loc[] = {0., 0., 0.};
    double rot_pos_quat_loc[] = {0., 0., 0., 1.};
    if (attached == 0)
    {
        addObj(obj_name_loc, 1, obj_dim_loc, obj_pos_loc, rot_pos_quat_loc, 1);
    }
    else
    {
        addAttachedObj(obj_name_loc, 1, obj_dim_loc, obj_pos_loc, rot_pos_quat_loc, 1);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        addObj(obj_name_loc, 1, obj_dim_loc, obj_pos_loc, rot_pos_quat_loc, 1);
    }
}

// Function to add a collision object from the user menu
void ManipulatorMenu::addUserAttachedObj()
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
        std::cout << "X dim: ";
        std::cin >> obj_dims[0];
    }
    // Else
    else
    {
        obj_dims = {0., 0.};
        std::cout << "X dim: ";
        std::cin >> obj_dims[0];
        std::cout << "Y dim: ";
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

// --------------------- COLLISION OBJECTS PRIVATE MENU HANDLER ---------------------

// Gripper Moving command from the user
void ManipulatorMenu::userGripperMove()
{
    double gripper_position = 0.;

    // Take user gripper position
    std::cout << "Enter the value (in %) of gripper move :\n";

    // Gripper position input
    std::cout << "Gripper opening position: ";
    std::cin >> gripper_position;

    moveGripper(gripper_position);
}

// --------------------- GRIPPER SERVICES ---------------------
// Call the service for open/close gripper
void ManipulatorMenu::callGripperSrv(const bool command)
{
    // Create a request
    std_srvs::srv::SetBool::Request::SharedPtr request;
    request->data = command;

    //Wait for srv
    while(gripper_client_->wait_for_service(std::chrono::seconds(1))){
        if (!rclcpp::ok()){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gripper service not available, waiting again...");
    }

    auto response = gripper_client_->async_send_request(request);

    // Call the srv
    if (rclcpp::spin_until_future_complete(shared_from_this(), response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (response.get()->success){
            RCLCPP_INFO(get_logger(), "Gripper command succeeded, set to %d", command);
        }
        else{
            RCLCPP_ERROR(get_logger(), "Failed move gripper");
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to call gripper service");
    }
}

// Call the service for grab/detach an object at the gripper
void ManipulatorMenu::callGrabbingSrv(const bool command)
{
    // Create a request
    std_srvs::srv::SetBool::Request::SharedPtr request;
    request->data = command;

    //Wait for srv
    while(grab_client_->wait_for_service(std::chrono::seconds(1))){
        if (!rclcpp::ok()){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper grabing service not available, waiting again...");
    }

    auto response = grab_client_->async_send_request(request);

    // Call the srv
    if (rclcpp::spin_until_future_complete(shared_from_this(), response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (response.get()->success){
            RCLCPP_INFO(get_logger(), "Gripper grabbing request succeeded");
        }
        else{
            RCLCPP_ERROR(get_logger(), "Gripper grabbing request failed");
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service for gripper grabbing");
    }
}

// Call the service to open/close the real gripper
void ManipulatorMenu::callRealGripperSrv(const float command)
{
    // Create a request
    motors_trajectory::srv::RobotiQGripperControl::Request::SharedPtr request;
    request->position = command;
    request->speed = 50;
    request->force = 50;

    //Wait for srv
    while(real_gripper_client_->wait_for_service(std::chrono::seconds(1))){
        if (!rclcpp::ok()){
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "real gripper service not available, waiting again...");
    }

    auto response = real_gripper_client_->async_send_request(request);

    // Call the srv
    if (rclcpp::spin_until_future_complete(shared_from_this(), response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (response.get()->success){
            RCLCPP_INFO(get_logger(), "Gripper move request succeeded");
        }
        else{
            RCLCPP_ERROR(get_logger(), "Gripper move request failed");
        }
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to call real gripper service");
    }
}

// --------------------- QUATERNIONS HANDLER -------------------
// Conversion from degrees euler angles to quaternion
geometry_msgs::msg::Quaternion ManipulatorMenu::quaternion_from_euler(double roll, double pitch, double yaw)
{
    // Declaration of empty quaternion
    geometry_msgs::msg::Quaternion quaternion;

    // Conversion from euler rotation to pose quaternion
    tf2::Quaternion quat;
    quat.setRPY(roll * M_PI / 180, pitch * M_PI / 180, yaw * M_PI / 180);
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

// --------------------- DEG-RADIANS HANDLER -------------------
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

// --------------------- MENU HANDLER ---------------------

void ManipulatorMenu::printMenu()
{
    std::cout << "\n======= MANIPULATOR MENU =======\n";
    std::cout << "\n======= Joint/tcp moving test options =======\n";
    std::cout << "1. Give a TCP goal through InvKine, with fake controller\n";
    std::cout << "2. Give a joint goal, with fake controller\n";
    std::cout << "3. Give a joint goal to MoveIt\n";
    std::cout << "4. Give a TCP goal to MoveIt\n";
    std::cout << "5. Give a TCP goal through InvKine to MoveIt\n";
    std::cout << "6. Move a defined joint\n";
    std::cout << "\n======= Carthesian moves test options =======\n";
    std::cout << "7. Move the robot along x\n";
    std::cout << "8. Move the robot along y\n";
    std::cout << "9. Move the robot along z\n";
    std::cout << "\n======= Tcp orientation options =======\n";
    std::cout << "10.Change TCP orientation\n";
    std::cout << "11.Rotate the TCP around x\n";
    std::cout << "12.Rotate the TCP around y\n";
    std::cout << "13.Rotate the TCP around z\n";
    std::cout << "14.Get a fixed TCP orientation\n";
    std::cout << "\n======= Handle objects in the planning scene =======\n";
    std::cout << "15.Add an object to the scene\n";
    std::cout << "16.Delete an object from the scene\n";
    std::cout << "\n======= Visualize current robot state =======\n";
    std::cout << "17.Visualize joints state\n";
    std::cout << "18.Visualize current tcp pose\n";
    std::cout << "\n======= Home positions setting =======\n";
    std::cout << "19.Go to home position (gripper down)\n";
    std::cout << "20.Go to home position (gripper at the front)\n";
    std::cout << "\n======= Cartesian move =======\n";
    std::cout << "21.Make a cartesian move\n";
    if (get_parameter("enable_coppelia").as_bool())
    {
        std::cout << "\n======= CoppeliaSim handling =======\n";
        std::cout << "22. To start twin Coppelia simulation\n";
        std::cout << "23. To stop  twin Coppelia simulation\n";
        std::cout << "24. To save  twin CoppeliaSim scene\n";
    }
    if (get_parameter("enable_sim_gripper").as_bool())
    {
        std::cout << "\n======= Fake gripper control =======\n";
        std::cout << "25.Open the gripper\n";
        std::cout << "26.Close the gripper\n";
        std::cout << "27.Set the position of the gripper\n";
        std::cout << "28.Grab an object at the gripper\n";
        std::cout << "29.Detach an object from the gripper\n";
    }
    if (get_parameter("enable_real_gripper").as_bool())
    {
        std::cout << "\n======= Real gripper control =======\n";
        std::cout << "30.Open  real gripper\n";
        std::cout << "31.Close real gripper\n";
        std::cout << "32.Move  real gripper to a given position \n";
    }
    std::cout << "\n======= Kinematics srvs =======\n";
    std::cout << "33. Get joint values through inverse kinematics of a given pose\n";
    std::cout << "34. Get current Jacobian\n";
    std::cout << "35. Get current Inverse Jacobian\n";
    std::cout << "36. Change velocity and acceleration as planner's parameters\n";
    std::cout << "\n======= Kinematics mode setter =======\n";
    std::cout << "37. Enable  the instantaneous kinematics mode for motors\n";
    std::cout << "38. Disable the instantaneous kinematics mode for motors\n";
    std::cout << "39. Enable  the jacobian speed control mode for robot joints\n";
    std::cout << "40. Disable the jacobian speed control mode for robot joints\n";
    std::cout << "41. Enable  the joints real time speed control mode\n";
    std::cout << "42. Disable the joints real time speed control mode\n";
    std::cout << "\n======= Closing ROS menu =======\n";
    std::cout << "43.Shutdown the menu\n";
    std::cout << "=====================\n";
}

int ManipulatorMenu::getUserChoice()
{
    int choice;
    std::cout << "Enter your choice: ";
    std::cin >> choice;
    return choice;
}

void ManipulatorMenu::processChoice(int choice)
{
    double step;                // Linear move length along axis
    std::vector<double> rot;    // End effector rotation
    std::vector<double> ee_pos; // End effector position
    switch (choice)
    {
    case 1:
        RCLCPP_INFO(get_logger(), "You selected Option 1");
        RCLCPP_ERROR(get_logger(), "no_planner option is obsolete, moveit will be used instead");
        userTcpGoal();
        break;
    case 2:
        RCLCPP_INFO(get_logger(), "You selected Option 2");
        RCLCPP_ERROR(get_logger(), "no_planner option is obsolete, moveit will be used instead");
        userJointGoal();
        break;
    case 3:
        RCLCPP_INFO(get_logger(), "You selected Option 3");
        userJointGoal();
        break;
    case 4:
        RCLCPP_INFO(get_logger(), "You selected Option 4");
        userTcpGoal();
        break;
    case 5:
        RCLCPP_INFO(get_logger(), "You selected Option 5");
        userTcpIKGoal();
        break;
    case 6:
        RCLCPP_INFO(get_logger(), "You selected Option 6");
        oneJointMove_user();
        break;

    case 7:
        RCLCPP_INFO(get_logger(), "You selected Option 7");

        std::cout << "Insert how many metres you want to move along x: \n";
        std::cin >> step;
        move_along_x(step);
        break;

    case 8:
        RCLCPP_INFO(get_logger(), "You selected Option 8");

        std::cout << "Insert how many metres you want to move along y:\n";
        std::cin >> step;
        move_along_y(step);
        break;
    case 9:
        RCLCPP_INFO(get_logger(), "You selected Option 9");

        std::cout << "Insert how many metres you want to move along z:\n";
        std::cin >> step;
        move_along_z(step);
        break;

    case 10:
        RCLCPP_INFO(get_logger(), "You selected Option 10\n");
        std::cout << "Insert the rotation around the axis you want to do.\n";
        rot = {0., 0., 0.};
        std::cout << " X rotation: ";
        std::cin >> rot[0];
        std::cout << " Y rotation: ";
        std::cin >> rot[1];
        std::cout << " Z rotation: ";
        std::cin >> rot[2];
        make_tcp_rot(rot);
        break;

    case 11:
        RCLCPP_INFO(get_logger(), "You selected Option 11");
        std::cout << "Insert the rotation around X axis you want to do.\n";
        double x_rot;
        std::cout << " X rotation: ";
        std::cin >> x_rot;
        rotate_around_x(x_rot);
        break;

    case 12:
        RCLCPP_INFO(get_logger(), "You selected Option 12");
        std::cout << "Insert the rotation around Y axis you want to do.\n";
        double y_rot;
        std::cout << " Y rotation: ";
        std::cin >> y_rot;
        rotate_around_y(y_rot);
        break;

    case 13:
        RCLCPP_INFO(get_logger(), "You selected Option 13");
        std::cout << "Insert the rotation around Z axis you want to do.\n";
        double z_rot;
        std::cout << " Z rotation: ";
        std::cin >> z_rot;
        rotate_around_z(z_rot);
        break;

    case 14:
        RCLCPP_INFO(get_logger(), "You selected Option 14");
        std::cout << "Insert the FIXED orientation of the EE you want to have.\n";
        rot = {0., 0., 0.};
        std::cout << " X rotation: ";
        std::cin >> rot[0];
        std::cout << " Y rotation: ";
        std::cin >> rot[1];
        std::cout << " Z rotation: ";
        std::cin >> rot[2];
        change_tcp_orient(rot);
        break;

    case 15:
    {
        RCLCPP_INFO(get_logger(), "You selected Option 15");
        std::cout << "Insert 1 if the object has to be attached to the tcp, 2 otherwise.\n";
        int val;
        std::cin >> val;
        if (val == 1)
        {
            addUserAttachedObj();
        }
        else if (val == 2)
        {
            addCollObj();
        }
    }
    break;
    case 16:
        RCLCPP_INFO(get_logger(), "You selected Option 16");
        deleteCollObj();
        break;
    case 17:
        RCLCPP_INFO(get_logger(), "You selected Option 17");
        jointStateVisualizer();
        break;

    case 18:
    {
        RCLCPP_INFO(get_logger(), "You selected Option 18");
        std::vector<double> ee_pose = getEEpos_rpy();
        std::cout << " EE - X position: " << ee_pose[0] << std::endl;
        std::cout << " EE - Y position: " << ee_pose[1] << std::endl;
        std::cout << " EE - Z position: " << ee_pose[2] << std::endl;
        std::cout << " EE - X rotation: " << ee_pose[3] << std::endl;
        std::cout << " EE - Y rotation: " << ee_pose[4] << std::endl;
        std::cout << " EE - Z rotation: " << ee_pose[5] << std::endl;
    }
    break;
    case 19:
        RCLCPP_INFO(get_logger(), "You selected Option 19");
        RCLCPP_INFO(get_logger(), "Go to home position, gripper down ...");
        goHome(0);
        break;
    case 20:
        RCLCPP_INFO(get_logger(), "You selected Option 20");
        RCLCPP_INFO(get_logger(), "Go to home position, gripper at the front ...");
        goHome(1);
        break;
    case 21:
        RCLCPP_INFO(get_logger(), "You selected Option 21");
        userCartesianMove();
        break;
    case 22:
        RCLCPP_INFO(get_logger(), "You selected Option 22");
        RCLCPP_INFO(get_logger(), "Start Coppelia simulation");
        startCoppeliaSim();
        break;
    case 23:
        RCLCPP_INFO(get_logger(), "You selected Option 23");
        RCLCPP_INFO(get_logger(), "Stop Coppelia simulation");
        stopCoppeliaSim();
        break;
    case 24:
        RCLCPP_INFO(get_logger(), "You selected Option 24");
        RCLCPP_INFO(get_logger(), "Save current Coppelia scene");
        saveCoppeliaScene();
        break;
    case 25:
        RCLCPP_INFO(get_logger(), "You selected Option 25");
        RCLCPP_INFO(get_logger(), "Opening the gripper ...");
        openGripper();
        break;
    case 26:
        RCLCPP_INFO(get_logger(), "You selected Option 26");
        RCLCPP_INFO(get_logger(), "Closing the gripper ...");
        closeGripper();
        break;
    case 27:
        RCLCPP_INFO(get_logger(), "You selected Option 27");
        RCLCPP_INFO(get_logger(), "Gripper moving setting");
        userGripperMove();
        break;
    case 28:
        RCLCPP_INFO(get_logger(), "You selected Option 28");
        RCLCPP_INFO(get_logger(), "Grab an object to the gripper");
        grabObjGripper();
        break;
    case 29:
        RCLCPP_INFO(get_logger(), "You selected Option 29");
        RCLCPP_INFO(get_logger(), "Detach an object from the gripper");
        detachObjGripper();
        break;
    case 30:
        RCLCPP_INFO(get_logger(), "You selected Option 30");
        RCLCPP_INFO(get_logger(), "Opening real gripper");
        openRealGripper();
        break;
    case 31:
        RCLCPP_INFO(get_logger(), "You selected Option 31");
        RCLCPP_INFO(get_logger(), "Closing real gripper");
        closeRealGripper();
        break;
    case 32:
        RCLCPP_INFO(get_logger(), "You selected Option 32");
        RCLCPP_INFO(get_logger(), "Set a real gripper position");
        float gripper_pos;
        std::cin >> gripper_pos;
        moveRealGripper(gripper_pos);
        break;
    case 33:
    {
        RCLCPP_INFO(get_logger(), "You selected Option 33");
        RCLCPP_INFO(get_logger(), "Set the pose you want to compute inverse kinematics.");
        geometry_msgs::msg::Pose pose;
        float rx, ry, rz;
        std::cout << "X position: ";
        std::cin >> pose.position.x;
        std::cout << "Y position: ";
        std::cin >> pose.position.y;
        std::cout << "Z position: ";
        std::cin >> pose.position.z;
        std::cout << "X rotation (in degrees): ";
        std::cin >> rx;
        std::cout << "Y rotation (in degrees): ";
        std::cin >> ry;
        std::cout << "Z rotation (in degrees): ";
        std::cin >> rz;
        pose.orientation = quaternion_from_euler(rx, ry, rz);
        std::vector<double> joints = invKineClient(pose);
        for (unsigned long k = 0; k < joints.size(); k++)
        {
            RCLCPP_INFO(get_logger(), "Joint %ld: %f", k, joints[k]);
        }
    }
    break;
    case 34:
    {
        RCLCPP_INFO(get_logger(), "You selected Option 34");
        Eigen::MatrixXd jac = getJacobianClient();
        RCLCPP_INFO(get_logger(), "Jacobian computed:\n");
        std::cout << jac << std::endl;
    }
    break;
    case 35:
    {
        RCLCPP_INFO(get_logger(), "You selected Option 35");
        Eigen::MatrixXd inv_jac = pseudoInverseClient();
        RCLCPP_INFO(get_logger(), "Inverse Jacobian computed:\n");
        std::cout << inv_jac << std::endl;
    }
    break;
    case 36:
    {
        RCLCPP_INFO(get_logger(), "You selected Option 36");
        float vel, acc;
        std::cout << "Insert new vel factor: ";
        std::cin >> vel;
        std::cout << "Insert new acc factor: ";
        std::cin >> acc;
        setNewPlannerParams(vel, acc);
    }
    break;
    case 37:
    {
        RCLCPP_INFO(get_logger(), "You selected Option 37");
        setInstantKineMode(true);
    }
    break;
    case 38:
    {
        RCLCPP_INFO(get_logger(), "You selected Option 38");
        setInstantKineMode(false);
    }
    break;
    case 39:
    {
        RCLCPP_INFO(get_logger(), "You selected Option 39");
        setJacobianSpeedControl(true);
    }
    break;
    case 40:
    {
        RCLCPP_INFO(get_logger(), "You selected Option 40");
        setJacobianSpeedControl(false);
    }
    break;
    case 41:
    {
        RCLCPP_INFO(get_logger(), "You selected Option 41");
        setJsRealTimeControl(true);
    }
    break;
    case 42:
    {
        RCLCPP_INFO(get_logger(), "You selected Option 42");
        setJsRealTimeControl(false);
    }
    break;
    case 43:
        RCLCPP_INFO(get_logger(), "You selected Option 41");
        RCLCPP_INFO(get_logger(), "Exiting...\n");
        rclcpp::shutdown();
        break;
    default:
        RCLCPP_WARN(get_logger(), "Invalid choice. Please choose a valid option.");
        break;
    }
}
