#include "manipulators/ManipulatorPlanner.h"

// ------------------------------------- PUBLIC METHODS -------------------------------------

ManipulatorPlannerNode::ManipulatorPlannerNode(const std::string node_name, const rclcpp::NodeOptions &options) 
: rclcpp::Node(node_name, options), node_name_(node_name) {

    checkParams();

    //Initialize velocity variables
    NUM_JOINTS = joint_names_.size();

    if (NUM_JOINTS == 0) {
        RCLCPP_ERROR(this->get_logger(), "No joint names provided");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Joints number: %ld", NUM_JOINTS);

    //Initialize the velocity command vectors
    current_js_vel_.resize(NUM_JOINTS, 1);
    current_js_vel_.setZero();
    js_vel_cmd_.resize(NUM_JOINTS, 1);
    js_vel_cmd_.setZero();
    current_ee_vel_.resize(6, 1);
    current_ee_vel_.setZero();
    ee_vel_cmd_.resize(6, 1);
    ee_vel_cmd_.setZero();

    // services_cb_group_ = this->create_callback_group(
    //     rclcpp::CallbackGroupType::MutuallyExclusive
    // );

    // Initialize service servers
    fkine_service_ = this->create_service<manipulator_interfaces::srv::FKine>(
        manipulator_name_ + "/get_fkine", 
        [this](const std::shared_ptr<manipulator_interfaces::srv::FKine::Request> request,
               std::shared_ptr<manipulator_interfaces::srv::FKine::Response> response) {
            this->getFKine_callback(request, response);
        },
        rmw_qos_profile_services_default,
        main_cb_group_
    );

    invkine_service_ = this->create_service<manipulator_interfaces::srv::InvKine>(
        manipulator_name_ + "/get_invkine", 
        [this](const std::shared_ptr<manipulator_interfaces::srv::InvKine::Request> request,
               std::shared_ptr<manipulator_interfaces::srv::InvKine::Response> response) {
            this->getInvKine_callback(request, response);
        },
        rmw_qos_profile_services_default,
        main_cb_group_
    );

    jacobian_service_ = this->create_service<manipulator_interfaces::srv::Jacobian>(
        manipulator_name_ + "/get_jacobian", 
        [this](const std::shared_ptr<manipulator_interfaces::srv::Jacobian::Request> request,
               std::shared_ptr<manipulator_interfaces::srv::Jacobian::Response> response) {
            this->getJacobian_callback(request, response);
        },
        rmw_qos_profile_services_default,
        main_cb_group_
    );

    pseudoInverse_service_ = this->create_service<manipulator_interfaces::srv::PseudoInverse>(
        manipulator_name_ + "/get_pseudo_inverse", 
        [this](const std::shared_ptr<manipulator_interfaces::srv::PseudoInverse::Request> request,
               std::shared_ptr<manipulator_interfaces::srv::PseudoInverse::Response> response) {
            this->getPseudoInverseJacobian_callback(request, response);
        },
        rmw_qos_profile_services_default,
        main_cb_group_
    );

    changePlannerScalingFactors_service_ = this->create_service<manipulator_interfaces::srv::ChangePlannerScalingFactors>(
        manipulator_name_ + "/change_planner_scaling_factors",
        [this](const std::shared_ptr<manipulator_interfaces::srv::ChangePlannerScalingFactors::Request> request,
               std::shared_ptr<manipulator_interfaces::srv::ChangePlannerScalingFactors::Response> response) {
            this->changePlannerScalingFactors_callback(request, response);
        },
        rmw_qos_profile_services_default,
        main_cb_group_
    );

    changePlannerTolerances_service_ = this->create_service<manipulator_interfaces::srv::ChangePlannerTolerances>(
        manipulator_name_ + "/change_planner_tolerances",
        [this](const std::shared_ptr<manipulator_interfaces::srv::ChangePlannerTolerances::Request> request,
               std::shared_ptr<manipulator_interfaces::srv::ChangePlannerTolerances::Response> response) {
            this->changePlannerTolerances_callback(request, response);
        },
        rmw_qos_profile_services_default,
        main_cb_group_
    );

    realTimeConstraintsSetter_service_ = this->create_service<manipulator_interfaces::srv::EnableRealTimeConstraints>(
        manipulator_name_ + "/enable_real_time_constraints",
        [this](const std::shared_ptr<manipulator_interfaces::srv::EnableRealTimeConstraints::Request> request,
               std::shared_ptr<manipulator_interfaces::srv::EnableRealTimeConstraints::Response> response) {
            this->realTimeConstraintsSetter_callback(request, response);
        },
        rmw_qos_profile_services_default,
        main_cb_group_
    );

    jointsRealTimeSetter_service_ = this->create_service<std_srvs::srv::SetBool>(
        manipulator_name_ + "/joints_real_time_setter",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            this->jointsRealTimeSetter_callback(request, response);
        },
        rmw_qos_profile_services_default,
        main_cb_group_
    );

    jacobianControlSetter_service_ = this->create_service<std_srvs::srv::SetBool>(
        manipulator_name_ + "/jacobian_control_setter",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            this->jacobianControlSetter_callback(request, response);
        },
        rmw_qos_profile_services_default,
        main_cb_group_
    );

    rclcpp::SubscriptionOptions sub_options;
    subscribers_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );
    sub_options.callback_group = subscribers_cb_group_;

    // Initialize subscribers
    tcpGoal_sub_ = this->create_subscription<manipulator_interfaces::msg::TcpGoal>(
        manipulator_name_ + "/tcp_goal", 1, 
        [this](const manipulator_interfaces::msg::TcpGoal::SharedPtr msg) {
            this->tcpGoal_callback(msg);
        },
        sub_options
    );

    jointGoal_sub_ = this->create_subscription<manipulator_interfaces::msg::JointGoal>(
        manipulator_name_ + "/joint_goal", 1, 
        [this](const manipulator_interfaces::msg::JointGoal::SharedPtr msg) {
            this->jointGoal_callback(msg);
        },
        sub_options
    );

    collisionObject_sub_ = this->create_subscription<moveit_msgs::msg::CollisionObject>(
        manipulator_name_ + "/collision_object", 1, 
        [this](const moveit_msgs::msg::CollisionObject::SharedPtr msg) {
            this->collisionObject_callback(msg);
        },
        sub_options
    );

    attachedcollisionObject_sub_ = this->create_subscription<moveit_msgs::msg::AttachedCollisionObject>(
        manipulator_name_ + "/attached_collision_object", 1, 
        [this](const moveit_msgs::msg::AttachedCollisionObject::SharedPtr msg) {
            this->attachedCollisionObject_callback(msg);
        },
        sub_options
    );

    cartesianPlan_sub_ = this->create_subscription<manipulator_interfaces::msg::CartesianGoal>(
        manipulator_name_ + "/cartesian_plan", 1, 
        [this](const manipulator_interfaces::msg::CartesianGoal::SharedPtr msg) {
            this->cartesianPlan_callback(msg);
        },
        sub_options
    );


    jointConstraint_sub_ = this->create_subscription<moveit_msgs::msg::JointConstraint>(
        manipulator_name_ + "/joint_constraint", 1, 
        [this](const moveit_msgs::msg::JointConstraint::SharedPtr msg) {
            this->jointConstraint_callback(msg);
        },
        sub_options
    );

    positionConstraint_sub_ = this->create_subscription<moveit_msgs::msg::PositionConstraint>(
        manipulator_name_ + "/position_constraint", 1, 
        [this](const moveit_msgs::msg::PositionConstraint::SharedPtr msg) {
            this->positionConstraint_callback(msg);
        },
        sub_options
    );
    

    orientationConstraint_sub_ = this->create_subscription<moveit_msgs::msg::OrientationConstraint>(
        manipulator_name_ + "/orientation_constraint", 1, 
        [this](const moveit_msgs::msg::OrientationConstraint::SharedPtr msg) {
            this->orientationConstraint_callback(msg);
        },
        sub_options
    );

    clearConstraints_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        manipulator_name_ + "/clear_constraints", 1, 
        [this](const std_msgs::msg::Empty::SharedPtr msg) {
            msg.get();
            dynamic_planner_->clearPathConstraints();
            constraints_poses_.clear();
            constraints_primitives_.clear();
            // dynamic_planner_->clearJointConstraints();
        },
        sub_options
    );

    // Set callback group for real time subs to the main one, this avoids race conditions on shared memory
    rclcpp::SubscriptionOptions rt_sub_options;
    rt_sub_options.callback_group = main_cb_group_;

    velJacSetpoint_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        manipulator_name_ + "/cmd_vel", 1, 
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            this->velJacSetpoint_callback(msg);
        },
        rt_sub_options
    );

    realTimeSetpoint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        manipulator_name_ + "/js_cmd_vel", 1, 
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->realTimeSetpoint_callback(msg);
        },
        rt_sub_options
    );

    tcpPose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(manipulator_name_ + "/tcp_pose", 1);
    tcpVel_pub_  = this->create_publisher<geometry_msgs::msg::Twist>(manipulator_name_ + "/tcp_vel", 1);

    rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
        std::bind(&ManipulatorPlannerNode::shutdown_handler, this) // Register shutdown handler
    );
}

ManipulatorPlannerNode::~ManipulatorPlannerNode() {
    RCLCPP_INFO(get_logger(), "Destroying ManipulatorPlannerNode...");
    // Stop the spinner thread
    if (mainloop_timer_) {
        mainloop_timer_->cancel();
    }
    if(tcpPose_timer_) {
        tcpPose_timer_->cancel();
    }
    if(tcpVel_timer_) {
        tcpVel_timer_->cancel();
    }
    executor_.cancel();
    dynamic_planner_.reset(); // Reset the dynamic planner
}

void ManipulatorPlannerNode::spinner() {
    unsigned long long int num_samples = 0; // Number of samples for the mean time calculation

    rclcpp::Rate rate(ros_freq_);

    initializePlanner(); //Initialize dynamic_planner_

    rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    main_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );

    mainloop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / ros_freq_)),
        [&, this]() {
            if (spinner_mean_ > 0.9 / ros_freq_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20000, "Spinner mean time is too high: %f s", spinner_mean_);
            }

            if (dynamic_planner_->isReady() == false) {
                return;
            }

            jacobian_var_ = getJacobian(); // Assign jacobian

            if (jac_control_) {

                // This is the main loop for the node
                auto start_time = steady_clock.now();

                // Jacobian control
                jacobianControl();    

                // Calculate the mean time for each iteration of the spinner
                double elapsed_time = (steady_clock.now() - start_time).seconds();
                spinner_mean_ = (spinner_mean_ * static_cast<double>(num_samples) + elapsed_time) / static_cast<double>(num_samples + 1);
                num_samples++;
            }
            else if (js_rt_control_) {
                // This is the main loop for the node
                auto start_time = steady_clock.now();

                // Real-time joint speed control
                jointsRealTimeControl();

                // Calculate the mean time for each iteration of the spinner
                double elapsed_time = (steady_clock.now() - start_time).seconds();
                spinner_mean_ = (spinner_mean_ * static_cast<double>(num_samples) + elapsed_time) / static_cast<double>(num_samples + 1);
                num_samples++;
            }

            tcpPose_pub_->publish(getFKine());
            tcpVel_pub_->publish(getTcpVel());

            rate.sleep();
        },
        main_cb_group_
    );

    executor_.add_node(this->shared_from_this()); //Add the dynamic planner node to the executor
    executor_.spin(); // Start the executor
}

// ------------------------------------- PRIVATE METHODS -------------------------------------

void ManipulatorPlannerNode::shutdown_handler(){
    dynamic_planner_->stop(); // Stop the robot
    RCLCPP_INFO(get_logger(), "Spinner mean time: %f s", spinner_mean_);
}

//KINEMATICS FUNCTIONS

// Get the current tcp pose through forward kinematics
const geometry_msgs::msg::Pose ManipulatorPlannerNode::getFKine() {
    return dynamic_planner_->getFKine(ee_name_).pose;
}

// Get the tcp twist by multiplying joints vels with the jacobian
const geometry_msgs::msg::Twist ManipulatorPlannerNode::getTcpVel()
{
    // Initialize dq with the appropriate size and assign values
    Eigen::VectorXd dq(NUM_JOINTS);
    std::vector<double> current_joint_speeds = dynamic_planner_->getJointSpeeds();

    for (unsigned int k = 0; k < NUM_JOINTS; k++)
    {
        dq(k) = current_joint_speeds[k];
    }

    // Compute the end-effector twist (linear and angular velocities) using the Jacobian
    Eigen::VectorXd twist = jacobian_var_ * dq;

    // Create a Twist message to hold the result
    geometry_msgs::msg::Twist tcp_twist;

    // Assign linear velocity components
    tcp_twist.linear.x = twist(0);
    tcp_twist.linear.y = twist(1);
    tcp_twist.linear.z = twist(2);

    // Assign angular velocity components
    tcp_twist.angular.x = twist(3);
    tcp_twist.angular.y = twist(4);
    tcp_twist.angular.z = twist(5);

    // Return the resulting TCP velocity
    return tcp_twist;
}

const Eigen::MatrixXd ManipulatorPlannerNode::getJacobian(const std::vector<double> &joint_positions, const std::string &end_effector_link) {
    return dynamic_planner_->getJacobian(joint_positions, end_effector_link);
}

const Eigen::MatrixXd ManipulatorPlannerNode::getJacobian(const std::string &end_effector_link) {
    return dynamic_planner_->getJacobian(end_effector_link);
}

const Eigen::MatrixXd ManipulatorPlannerNode::getJacobian() {
    return getJacobian(ee_name_);
}

const Eigen::MatrixXd ManipulatorPlannerNode::getPseudoInverseJacobian(const std::vector<double> &joint_positions, const std::string &end_effector_link) {
    return dynamic_planner_->getPseudoInverseJacobian(joint_positions, end_effector_link);
}

const Eigen::MatrixXd ManipulatorPlannerNode::getPseudoInverseJacobian(const std::string &end_effector_link) {
    return dynamic_planner_->getPseudoInverseJacobian(end_effector_link);
}

const Eigen::MatrixXd ManipulatorPlannerNode::getPseudoInverseJacobian() {
    return getPseudoInverseJacobian(ee_name_);
}

//CALLBACK FUNCTIONS

void ManipulatorPlannerNode::getFKine_callback(
    const std::shared_ptr<manipulator_interfaces::srv::FKine::Request> request, 
    std::shared_ptr<manipulator_interfaces::srv::FKine::Response> response
) {
    sensor_msgs::msg::JointState joint_state = request->joint_state;

    if (joint_state.position.empty()) {
        response->tcp_pose = dynamic_planner_->getFKine(ee_name_); //Return current position fkine
    } else {
        response->tcp_pose = dynamic_planner_->getFKine(joint_state.position, ee_name_);
    }

}


void ManipulatorPlannerNode::getInvKine_callback(
    const std::shared_ptr<manipulator_interfaces::srv::InvKine::Request> request,
    std::shared_ptr<manipulator_interfaces::srv::InvKine::Response> response
) {
    response->joint_values = dynamic_planner_->invKine(
        request->target_pose, 
        ee_name_
    );
}

void ManipulatorPlannerNode::getJacobian_callback(
    const std::shared_ptr<manipulator_interfaces::srv::Jacobian::Request> request,
    std::shared_ptr<manipulator_interfaces::srv::Jacobian::Response> response
) {
    request.get(); //Suppress unused var warning

    Eigen::MatrixXd jacobian = dynamic_planner_->getJacobian(ee_name_);
    std::vector<double> jacobian_values(jacobian.data(), jacobian.data() + jacobian.size()); //Flattens the matrix into a vector

    response->matrix_values = jacobian_values;
}

void ManipulatorPlannerNode::getPseudoInverseJacobian_callback(
    const std::shared_ptr<manipulator_interfaces::srv::PseudoInverse::Request> request,
    std::shared_ptr<manipulator_interfaces::srv::PseudoInverse::Response> response
) {
    request.get(); //Suppress unused var warning

    Eigen::MatrixXd pseudo_inv = getPseudoInverseJacobian();
    std::vector<double> pseudo_inv_values(pseudo_inv.data(), pseudo_inv.data() + pseudo_inv.size()); //Flattens the matrix into a vector

    response->matrix_values = pseudo_inv_values;
}

void ManipulatorPlannerNode::changePlannerScalingFactors_callback(
    const std::shared_ptr<manipulator_interfaces::srv::ChangePlannerScalingFactors::Request> request,
    std::shared_ptr<manipulator_interfaces::srv::ChangePlannerScalingFactors::Response> response
) {
    DynamicPlannerParams params = dynamic_planner_->getParams();
    params.acc_factor = request->acc_factor;
    params.vel_factor = request->vel_factor;
    dynamic_planner_->setParams(params);

    //Also apply changes to the actual parameters of the node
    rclcpp::Parameter new_acc("acc_factor", params.acc_factor);
    rclcpp::Parameter new_vel("vel_factor", params.vel_factor);
    this->set_parameter(new_acc);
    this->set_parameter(new_vel);

    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Planner parameters changed successfully (vel_factor: %f, acc_factor: %f)", params.vel_factor, params.acc_factor);
}

void ManipulatorPlannerNode::changePlannerTolerances_callback(
    const std::shared_ptr<manipulator_interfaces::srv::ChangePlannerTolerances::Request> request,
    std::shared_ptr<manipulator_interfaces::srv::ChangePlannerTolerances::Response> response
) {
    DynamicPlannerParams params = dynamic_planner_->getParams();
    params.position_tolerance = request->position_tolerance;
    params.orientation_tolerance = request->orientation_tolerance;
    params.joint_tolerance = request->joint_tolerance;
    dynamic_planner_->setParams(params);

    //Also apply changes to the actual parameters of the node
    rclcpp::Parameter new_position("position_tolerance", params.position_tolerance);
    rclcpp::Parameter new_orientation("orientation_tolerance", params.orientation_tolerance);
    rclcpp::Parameter new_joint("joint_tolerance", params.joint_tolerance);

    this->set_parameter(new_position);
    this->set_parameter(new_orientation);
    this->set_parameter(new_joint);

    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Planner parameters changed successfully (position_tolerance: %f, orientation_tolerance: %f, joint_tolerance: %f)", 
                params.position_tolerance, params.orientation_tolerance, params.joint_tolerance);
}

void ManipulatorPlannerNode::tcpGoal_callback(const manipulator_interfaces::msg::TcpGoal::SharedPtr msg) 
{
    RCLCPP_INFO(this->get_logger(), "Received TCP goal");

    std::string ee_link_name;
    std::string ref_frame;

    // RCLCPP_INFO(this->get_logger(), "Frame: %s, EE: %s", msg->frame.c_str(), msg->end_effector.c_str());

    if(msg->end_effector == manipulator_interfaces::msg::TcpGoal::DEFAULT){
        ee_link_name = ee_name_;
    } else {
        ee_link_name = msg->end_effector;
    }

    if(msg->frame == manipulator_interfaces::msg::TcpGoal::DEFAULT){
        ref_frame = world_frame_;
    } else {
        ref_frame = msg->frame;
    }

    if(!msg->start_state.position.empty()){ //If a start state is provided set the robot to that state
        moveit::core::RobotStatePtr robot_state = dynamic_planner_->getRobotState();
        robot_state->setJointGroupPositions(planning_group_, msg->start_state.position);
    }

    dynamic_planner_->plan(msg->target_pose, ee_link_name, ref_frame);

    if(msg->execute){
        dynamic_planner_->executeTrajectory();
    }
}

void ManipulatorPlannerNode::jointGoal_callback(const manipulator_interfaces::msg::JointGoal::SharedPtr msg) 
{
    RCLCPP_INFO(this->get_logger(), "Received joint goal");

    if(!msg->start_state.position.empty()){ //If a start state is provided set the robot to that state
        moveit::core::RobotStatePtr robot_state = dynamic_planner_->getRobotState();
        robot_state->setJointGroupPositions(planning_group_, msg->start_state.position);
    }
    
    dynamic_planner_->plan(msg->joint_goal.position);

    if (msg->execute){
        dynamic_planner_->executeTrajectory();
    }
}

void ManipulatorPlannerNode::collisionObject_callback(const moveit_msgs::msg::CollisionObject::SharedPtr collision_object) 
{
    RCLCPP_INFO(this->get_logger(), "Received collision object: %s", collision_object->id.c_str());

    //Set defaults
    if (collision_object->header.frame_id == "")
    {
        collision_object->header.frame_id = world_frame_;
    }

    dynamic_planner_->processCollisionObject(*collision_object);
}

void ManipulatorPlannerNode::attachedCollisionObject_callback(const moveit_msgs::msg::AttachedCollisionObject::SharedPtr collision_object) 
{
    RCLCPP_INFO(this->get_logger(), "Received attached collision object: %s", collision_object->object.id.c_str());

    //Set defaults
    if (collision_object->link_name == "")
    {
        collision_object->link_name = ee_name_;
    }
    if (collision_object->object.header.frame_id == "")
    {
        collision_object->object.header.frame_id = ee_name_;
    }

    // Gripper links are always in touch with the object, this will ignore collisions between them
    for (const std::string& link_name : gripper_links_)
    {
        collision_object->touch_links.push_back(link_name);
    }

    dynamic_planner_->processAttachedCollisionObject(*collision_object);
}

// Callback function for goals in the 3D cartesian space for the robot TCP
// Joint positions are computed through InvKine of inputs
void ManipulatorPlannerNode::cartesianPlan_callback(const manipulator_interfaces::msg::CartesianGoal::SharedPtr msg)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    std::string ee_link_name;
    std::string ref_frame;

    for (geometry_msgs::msg::Pose point : msg->waypoints.poses)
    {
        waypoints.push_back(point);
    }

    if(msg->end_effector == manipulator_interfaces::msg::TcpGoal::DEFAULT){
        ee_link_name = ee_name_;
    } else {
        ee_link_name = msg->end_effector;
    }

    if(msg->frame == manipulator_interfaces::msg::TcpGoal::DEFAULT){
        ref_frame = world_frame_;
    } else {
        ref_frame = msg->frame;
    }

    if(!msg->start_state.position.empty()){ //If a start state is provided set the robot to that state
        moveit::core::RobotStatePtr robot_state = dynamic_planner_->getRobotState();
        robot_state->setJointGroupPositions(planning_group_, msg->start_state.position);
    }

    dynamic_planner_->cartesianPlan(waypoints, ee_link_name, ref_frame);
    
    if (msg->execute)
    {
        dynamic_planner_->executeTrajectory();
    }
}

// Enable or disable the real time constraints for the planner
// (limit_joints_control_ and limit_jacobian_control_ parameters)
void ManipulatorPlannerNode::realTimeConstraintsSetter_callback(const std::shared_ptr<manipulator_interfaces::srv::EnableRealTimeConstraints::Request> req, 
                                                                std::shared_ptr<manipulator_interfaces::srv::EnableRealTimeConstraints::Response> res)
{
    // Set the real time constraints
    limit_joints_control_ = req->limit_joints_control;
    limit_jacobian_control_ = req->limit_jacobian_control;
    RCLCPP_INFO(get_logger(), "Real time constraints set as: limit_joints_control = %s, limit_jacobian_control = %s", 
                limit_joints_control_ ? "True" : "False", limit_jacobian_control_ ? "True" : "False");

    // Return success
    res->success = true;
}

// Set the jacobian speed based control
void ManipulatorPlannerNode::jointsRealTimeSetter_callback(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res)
{

    // Stop the robot to prevent bad behaviours during mode switch
    for (unsigned int k = 0; k < joint_names_.size(); k++)
    {
        current_js_vel_[k]  = 0.;
        js_vel_cmd_[k] = 0.;
    }

    double timeout = max_spd_jnts_ / max_acc_jnts_ * 1.2;
    auto start_time = this->now();
    
    while (isRobotMoving() && (this->now() - start_time).seconds() < timeout) {
        rclcpp::sleep_for(std::chrono::milliseconds((long)(1000 / ros_freq_)));
    }

    if (isRobotMoving()) {
        // ERROR: Robot is still moving, something failed, send last stop command and throw fatal error
        sensor_msgs::msg::JointState stop_state = dynamic_planner_->getJointState();
        stop_state.velocity = std::vector<double>(NUM_JOINTS, 0.0);
        dynamic_planner_->moveRobot(stop_state);

        throw std::runtime_error("FATAL: Robot received stop command but is still moving.");
    }

    // Set robot real time joints speed control
    js_rt_control_ = req->data;
    RCLCPP_INFO(get_logger(), "Joints real time control mode set as %s", js_rt_control_ ? "True":"False");
    
    // Return success
    res->success = true;
    res->message = js_rt_control_ ? "Joints real time control mode enabled":"Joints real time control mode disabled";
}

// Set the jacobian speed based control
void ManipulatorPlannerNode::jacobianControlSetter_callback(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res)
{
    // Stop the robot to prevent bad behaviours during mode switch
    for (unsigned int k = 0; k<6; k++) {
        current_ee_vel_(k) = 0.;
        ee_vel_cmd_(k) = 0.;
    }

    double timeout = std::max(max_speed_ee_ / max_accel_ee_, max_rot_speed_ee_ / max_rot_accel_ee_) * 1.2;
    auto start_time = this->now();
    
    while (isRobotMoving() && (this->now() - start_time).seconds() < timeout) {
        rclcpp::sleep_for(std::chrono::milliseconds((long)(1000 / ros_freq_)));
    }

    if (isRobotMoving()) {
        // ERROR: Robot is still moving, something failed, send last stop command and throw fatal error
        sensor_msgs::msg::JointState stop_state = dynamic_planner_->getJointState();
        stop_state.velocity = std::vector<double>(NUM_JOINTS, 0.0);
        dynamic_planner_->moveRobot(stop_state);

        throw std::runtime_error("FATAL: Robot received stop command but is still moving.");
    }

    // Set robot jacobian control
    jac_control_ = req->data;
    RCLCPP_INFO(get_logger(), "Jacobian control mode set as %s", jac_control_ ? "True":"False");

    // Return success
    res->success = true;
    res->message = jac_control_ ? "Jacobian control mode enabled":"Jacobian control mode disabled";
}

// Update speed setpoint of the arm for the real time joints speed based control
void ManipulatorPlannerNode::realTimeSetpoint_callback(const sensor_msgs::msg::JointState::SharedPtr& msg)
{
    if (js_rt_control_ == false) {
        RCLCPP_WARN(get_logger(), "Received joint speed command but joints real time control mode is disabled");
        return;
    }

    for (unsigned int k = 0; k < NUM_JOINTS; k++)
    {
        // Update setpoint of the k-th joint
        js_vel_cmd_[k] = msg->velocity[k];
        // Check if the vel cmds exceed the maximum acceptable speed
        if (abs(msg->velocity[k]) > max_spd_jnts_)
        {
            js_vel_cmd_[k] = sign(msg->velocity[k])*max_spd_jnts_;
        }
    }
}

// Update the velocity setpoint of the arm for the jacobian speed based control
void ManipulatorPlannerNode::velJacSetpoint_callback(const geometry_msgs::msg::Twist::SharedPtr& msg)
{
    if (jac_control_ == false) {
        RCLCPP_WARN(get_logger(), "Received velocity command but jacobian control mode is disabled");
        return;
    }

    // Map the linear velocity
    ee_vel_cmd_[0] = msg->linear.x;
    ee_vel_cmd_[1] = msg->linear.y;
    ee_vel_cmd_[2] = msg->linear.z;
    
    // Map the angular velocity components from the Twist message
    ee_vel_cmd_[3] = msg->angular.x; // X component of angular velocity
    ee_vel_cmd_[4] = msg->angular.y; // Y component of angular velocity
    ee_vel_cmd_[5] = msg->angular.z; // Z component of angular velocity
    
    // VELOCITY CAPPING
    double norm_linear = ee_vel_cmd_.head<3>().norm();
    if (norm_linear > max_speed_ee_) {ee_vel_cmd_ *= (max_speed_ee_/norm_linear);}

    double norm_angular = ee_vel_cmd_.tail<3>().norm();
    if (norm_angular > max_speed_ee_) {ee_vel_cmd_.tail<3>() *= (max_rot_speed_ee_/norm_angular);}
}

void ManipulatorPlannerNode::jointConstraint_callback(const moveit_msgs::msg::JointConstraint::SharedPtr msg)
{
    // Add the joint constraint to the planning scene
    RCLCPP_INFO(this->get_logger(), "Received joint constraint");
    moveit_msgs::msg::Constraints current_constraints = dynamic_planner_->getPathConstraints();
    current_constraints.joint_constraints.push_back(*msg.get());
    dynamic_planner_->setPathConstraints(current_constraints);
}

void ManipulatorPlannerNode::positionConstraint_callback(const moveit_msgs::msg::PositionConstraint::SharedPtr msg)
{
    // Add the position constraint to the planning scene
    RCLCPP_INFO(this->get_logger(), "Received position constraint");
    moveit_msgs::msg::Constraints current_constraints = dynamic_planner_->getPathConstraints();

    moveit_msgs::msg::PositionConstraint position_constraint = *msg.get();
    if (position_constraint.header.frame_id.empty()) {
        // If the frame_id is empty, set it to the world frame
        position_constraint.header.frame_id = world_frame_;
    }

    // Validate that primitive_poses match primitives in size
    if (position_constraint.constraint_region.primitives.size() != 
        position_constraint.constraint_region.primitive_poses.size()) {
        RCLCPP_ERROR(get_logger(), "Error: Number of primitives (%zu) doesn't match number of poses (%zu)",
            position_constraint.constraint_region.primitives.size(),
            position_constraint.constraint_region.primitive_poses.size());
        return;
    }

    current_constraints.position_constraints.push_back(position_constraint);
    dynamic_planner_->setPathConstraints(current_constraints);

    updateJacobianConstraintPrimitives();
}

void ManipulatorPlannerNode::orientationConstraint_callback(const moveit_msgs::msg::OrientationConstraint::SharedPtr msg)
{
    // Add the orientation constraint to the planning scene
    RCLCPP_INFO(this->get_logger(), "Received orientation constraint");
    moveit_msgs::msg::Constraints current_constraints = dynamic_planner_->getPathConstraints();
    current_constraints.orientation_constraints.push_back(*msg.get());
    dynamic_planner_->setPathConstraints(current_constraints);
}

//CONTROL FUNCTIONS

void setToZeroIfSmall(double &value)
{
    if (std::abs(value) < 1e-6) {value = 0.0;}
}

double ManipulatorPlannerNode::sign(double val)
{
    if      (val > 0) {return +1.;}
    else if (val < 0) {return -1.;}
    else              {return  0.;}
}

// Execute the jacobian based control
void ManipulatorPlannerNode::jacobianControl()
{
    updateJacobianSpeedCmd(); // Update the speed setpoint of the arm

    // Compute the speed
    Eigen::VectorXd qdot(NUM_JOINTS);

    // Scale jacobian
    // The process of normalization for the velocity setpoint is done to have a common
    // scale between the translational and rotational components of the velocity.
    // Jacobian rows are scaled accordingly.
    
    Eigen::MatrixXd jacobian_normalized = jacobian_var_;
    for (unsigned int i = 0; i < 3; i++) {
        jacobian_normalized.row(i) /= max_speed_ee_;
    }
    for (unsigned int i = 3; i < 6; i++) {
        jacobian_normalized.row(i) /= max_rot_speed_ee_;
    }

    Eigen::VectorXd v_scaled = current_ee_vel_;
    v_scaled.head<3>() /= max_speed_ee_;
    v_scaled.tail<3>() /= max_rot_speed_ee_;

    // Compute the SVD of the jacobian then use adaptive damped least squares to compute the joint velocities, 
    // this will help to mitigate the effect of singularities
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian_normalized, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXd sigma = svd.singularValues();
    double sigma_min = sigma.minCoeff();
    double lambda = 0.0;

    if (sigma_min < jac_sigma_threshold_) {
        // If the smallest singular value is below the threshold, we are close to a singularity
        // damping is applied
        lambda = (1 - sigma_min / jac_sigma_threshold_) * jac_max_damping_factor_;
    }

    Eigen::VectorXd sigma_damped(sigma.size());
    for(int i=0; i<sigma.size(); ++i)
        sigma_damped(i) = sigma(i) / (sigma(i)*sigma(i) + lambda*lambda);
   
    Eigen::MatrixXd invJ_damped =
        svd.matrixV() *
        sigma_damped.asDiagonal() *
        svd.matrixU().transpose();

    qdot = invJ_damped * v_scaled;
    
    // Set a lower limit to joint velocities to avoid noise
    for (unsigned int k = 0; k < NUM_JOINTS; k++) {setToZeroIfSmall(qdot[k]);}
    
    // Convert joints state into Eigen::VectorXd
    Eigen::VectorXd q(NUM_JOINTS);
    std::vector<double> current_joint_values = dynamic_planner_->getJointValues();
    for (unsigned int k = 0; k < NUM_JOINTS; k++)
    {
        q(k) = current_joint_values[k];
    }
     
    // Update joint position setpoint
    Eigen::VectorXd dq = q + qdot / ros_freq_;

    // Build the msg for the joints setpoint
    sensor_msgs::msg::JointState js;
    js.name     = joint_names_;
    js.position.resize(dq.size());
    js.velocity.resize(qdot.size());
    
    // Insert positions and velocity setpoints
    for (unsigned int k = 0; k < NUM_JOINTS; k++)
    {
        js.position[k] = dq[k];
        js.velocity[k] = qdot[k];
    }

    bool jac_check = true;
    bool pos_constraints_check = true;
    bool joint_constraints_check = true;

    // if (min_jacobian_determinant_ > 0.0){
    //     Eigen::MatrixXd jacobian = getJacobian(js.position, ee_name_); // Compute the jacobian matrix for the new position
    //     jac_check = abs(jacobian.determinant()) >= min_jacobian_determinant_; // Jacobian check failed
    // }

    if (limit_jacobian_control_) {
        // If the jacobian control is limited, check if the joint constraints are violated
        for (unsigned int k = 0; k < constraints_primitives_.size(); k++) {
            // Check if the position constraints are violated
            if (!isPoseInsidePrimitive(
                dynamic_planner_->getFKine(js.position, ee_name_).pose,
                constraints_primitives_[k], 
                constraints_poses_[k]
            )) {
                pos_constraints_check = false;
                break; // Exit the loop if a constraint is violated
            }
        }
    }

    if (limit_joints_control_) {
        // If the joint control is limited, check if the joint constraints are violated
        if (!dynamic_planner_->checkJointConstraints(js.position)) {
            RCLCPP_WARN(get_logger(), "Joint constraints violated, stopping the robot");
            joint_constraints_check = false;
        }
    }


    if (!jac_check || !pos_constraints_check || !joint_constraints_check) {
        js.position = dynamic_planner_->getJointValues(); // Set the position to the current one
        js.velocity = std::vector<double>(NUM_JOINTS, 0.0); // Set the velocity to zero
    }
    
    // Send the goal to the move it fake controller as trajectory point
    dynamic_planner_->moveRobot(js);
}

void ManipulatorPlannerNode::updateJacobianSpeedCmd(){
    // Compute the difference between the new and old velocity commands
    Eigen::VectorXd delta = ee_vel_cmd_ - current_ee_vel_;
    for (unsigned int k = 0; k < 6; k++) {setToZeroIfSmall(delta[k]);}

    double delta_linear_norm = delta.head<3>().norm();
    double delta_angular_norm = delta.tail<3>().norm();

    if (delta_linear_norm * ros_freq_ < max_accel_ee_){
        // Delta is small enough to set the new velocity directly (or the robot is not moving)
        current_ee_vel_.head<3>() = ee_vel_cmd_.head<3>();
    }
    else {
        for (unsigned int k = 0; k < 3; k++)
        {
            // Compute the acceleration for each component mutiplying acceleration max by the component of the delta versor
            double acc = max_accel_ee_ * abs(delta[k]) / delta_linear_norm;
            current_ee_vel_(k) = current_ee_vel_(k) + sign(delta[k]) * acc / ros_freq_;
        }
    }

    if (delta_angular_norm * ros_freq_ < max_rot_accel_ee_){
        // Delta is small enough to set the new velocity directly (or the robot is not moving)
        current_ee_vel_.tail<3>() = ee_vel_cmd_.tail<3>();
    }
    else {
        for (unsigned int k = 3; k < 6; k++)
        {
            // Compute the acceleration for each component mutiplying acceleration max by the component of the delta versor
            double acc = max_rot_accel_ee_ * abs(delta[k]) / delta_angular_norm;
            current_ee_vel_(k) = current_ee_vel_(k) + sign(delta[k]) * acc / ros_freq_;
        }
    }

    // Set a lower limit to velocities to avoid noises
    for (unsigned int k = 0; k < 6; k++) {setToZeroIfSmall(current_ee_vel_[k]);}
}

void ManipulatorPlannerNode::updateJacobianConstraintPrimitives()
{
    constraints_primitives_.clear(); // Clear the constraints if the jacobian control is enabled
    constraints_poses_.clear(); // Clear the poses if the jacobian control is enabled
    moveit_msgs::msg::Constraints current_constraints = dynamic_planner_->getPathConstraints();
    for (auto &constraint : current_constraints.position_constraints) {
        for (size_t i {0}; i < constraint.constraint_region.primitives.size(); ++i) {
            // Store the primitives for later use
            constraints_primitives_.push_back(constraint.constraint_region.primitives[i]);
            // Store the poses for later use
            constraints_poses_.push_back(constraint.constraint_region.primitive_poses[i]);
        }
    }

}

// Execute the jacobian based control
void ManipulatorPlannerNode::jointsRealTimeControl()
{
    // Check if the accelerations are acceptable and map the joints speed from the JointState message
    for (unsigned int k = 0; k < NUM_JOINTS; k++)
    {
        if (abs(js_vel_cmd_[k]-current_js_vel_[k])*ros_freq_ > max_acc_jnts_)
            {current_js_vel_[k] = current_js_vel_[k] + sign(js_vel_cmd_[k]-current_js_vel_[k])*max_acc_jnts_/ros_freq_;}
        else  {current_js_vel_[k] = js_vel_cmd_[k];}
    }

    // Convert joints state into Eigen::MatrixXd
    Eigen::VectorXd q(NUM_JOINTS);

    std::vector<double> current_joint_values = dynamic_planner_->getJointValues();
    for (unsigned int k = 0; k < NUM_JOINTS; k++)
    {
        q[k] = current_joint_values[k];
    }
        
    // Set a lower limit to joint velocities to avoid noises
    for (unsigned int k = 0; k < 6; k++) {setToZeroIfSmall(current_js_vel_[k]);}

    // Update joint position setpoint
    Eigen::VectorXd dq(NUM_JOINTS);
    dq = q + current_js_vel_ / ros_freq_;

    // Build the msg for the joints setpoint
    sensor_msgs::msg::JointState js;
    js.name = joint_names_;
    js.position.resize(dq.size());
    js.velocity.resize(current_js_vel_.size());

    // Insert positions and velocity setpoints
    for (unsigned int k = 0; k < NUM_JOINTS; k++)
    {
        js.position[k] = dq[k];
        js.velocity[k] = current_js_vel_[k];
    }

    if (limit_joints_control_) {
        // If the joint control is limited, check if the joint constraints are violated
        if (!dynamic_planner_->checkJointConstraints(js.position)) {
            RCLCPP_WARN(get_logger(), "Joint constraints violated, stopping the robot");
            js.position = dynamic_planner_->getJointValues(); // Set the position to the current one
            js.velocity = std::vector<double>(NUM_JOINTS, 0.0); // Set the velocity to zero
        }
    }

    // Send the goal to the move it fake controller as trajectory point
    dynamic_planner_->moveRobot(js);
}

bool ManipulatorPlannerNode::isRobotMoving(){
    std::vector<double> current_joint_speeds = dynamic_planner_->getJointSpeeds();
    for (unsigned int k = 0; k < NUM_JOINTS; k++)
    {
        if (abs(current_joint_speeds[k]) > dynamic_planner_->getParams().joint_tolerance) {return true;}
    }
    return false;
}

//HELPER FUNCTIONS

void ManipulatorPlannerNode::addPrefix(const std::string &prefix, std::vector<std::string> &names) const {
    /*
    Adds a prefix to each name in the vector
    Args:
        prefix: Prefix to add
        names: Vector of names to modify
    */
    for (auto &name : names) {
        name = prefix + name;
    }
}

void ManipulatorPlannerNode::checkParams() {
    std::string prefix = this->get_parameter_or("prefix", std::string());

    manipulator_name_         = this->get_parameter_or("manipulator_name", std::string());
    planning_group_           = prefix + this->get_parameter_or("planning_group", std::string());
    joint_names_              = this->get_parameter_or("joint_names", std::vector<std::string>());
    ee_name_                  = prefix + this->get_parameter_or("ee_name", std::string());
    ros_freq_                 = this->get_parameter_or("ros_freq", 500);
    max_speed_ee_             = this->get_parameter_or("max_speed_ee", 1.0);
    max_accel_ee_             = this->get_parameter_or("max_accel_ee", 1.0);
    max_rot_speed_ee_         = this->get_parameter_or("max_rot_speed_ee", 1.0);
    max_rot_accel_ee_         = this->get_parameter_or("max_rot_accel_ee", 1.0);
    max_spd_jnts_             = this->get_parameter_or("max_spd_jnts", 1.0);
    max_acc_jnts_             = this->get_parameter_or("max_acc_jnts", 1.0);
    gripper_links_            = this->get_parameter_or("gripper_links", std::vector<std::string>());
    world_frame_              = prefix + this->get_parameter_or("world_frame", std::string("base_link"));
    min_jacobian_determinant_ = this->get_parameter_or("min_jacobian_determinant", 0.0);
    limit_joints_control_     = this->get_parameter_or("limit_joints_control", false);
    limit_jacobian_control_   = this->get_parameter_or("limit_jacobian_control", false);
    jac_max_damping_factor_   = this->get_parameter_or("jacobian_max_damping_factor", 0.1);
    jac_sigma_threshold_      = this->get_parameter_or("jacobian_sigma_threshold", 0.1);

    addPrefix(prefix, joint_names_);
    addPrefix(prefix, gripper_links_);
}
 
void ManipulatorPlannerNode::initializePlanner() {
    //Initialize the dynamic planner
    dynamic_planner_ = std::make_shared<DynamicPlanner>(
        shared_from_this(),
        planning_group_);
}

bool ManipulatorPlannerNode::isPoseInsidePrimitive(
    const geometry_msgs::msg::Pose &pose,
    const shape_msgs::msg::SolidPrimitive &primitive,
    const geometry_msgs::msg::Pose &primitive_pose
) {
    // Convert ROS poses to Eigen
    Eigen::Vector3d point(pose.position.x, pose.position.y, pose.position.z);

    tf2::Transform tf_primitive;
    tf2::fromMsg(primitive_pose, tf_primitive);

    tf2::Vector3 tf_point(point.x(), point.y(), point.z());

    // Transform the point into the primitive's local frame
    tf2::Vector3 local_point = tf_primitive.inverse() * tf_point;
    Eigen::Vector3d p_local(local_point.x(), local_point.y(), local_point.z());

    switch (primitive.type) {
        case shape_msgs::msg::SolidPrimitive::SPHERE: {
            double radius = primitive.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS];
            return p_local.norm() <= radius;
        }

        case shape_msgs::msg::SolidPrimitive::BOX: {
            double hx = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] / 2.0;
            double hy = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] / 2.0;
            double hz = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] / 2.0;

            return (std::abs(p_local.x()) <= hx &&
                    std::abs(p_local.y()) <= hy &&
                    std::abs(p_local.z()) <= hz);
        }

        case shape_msgs::msg::SolidPrimitive::CYLINDER: {
            double height = primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT];
            double radius = primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS];

            double r_xy = std::sqrt(p_local.x() * p_local.x() + p_local.y() * p_local.y());
            return (r_xy <= radius && std::abs(p_local.z()) <= height / 2.0);
        }

        default:
            RCLCPP_WARN(this->get_logger(), "Unsupported primitive type.");
            return false;
    }
}
