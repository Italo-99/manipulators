#include "manipulators/ManipulatorPlanner.h"

// ------------------------------------- PUBLIC METHODS -------------------------------------

ManipulatorPlannerNode::ManipulatorPlannerNode(const std::string node_name, const rclcpp::NodeOptions &options) 
: rclcpp::Node(node_name, options), node_name_(node_name) {

    declareParameters();

    std::string prefix = this->get_parameter("prefix").as_string();    

    manipulator_name_ = this->get_parameter("manipulator_name").as_string();
    planning_group_ = prefix + this->get_parameter("planning_group").as_string();
    joint_names_ = this->get_parameter("joint_names").as_string_array();
    ee_name_ = prefix + this->get_parameter("ee_name").as_string();
    ros_freq_ = this->get_parameter("ros_freq").as_int();
    max_speed_ee_ = this->get_parameter("max_speed_ee").as_double();
    max_accel_ee_ = this->get_parameter("max_accel_ee").as_double();
    max_rot_speed_ee_ = this->get_parameter("max_rot_speed_ee").as_double();
    max_rot_accel_ee_ = this->get_parameter("max_rot_accel_ee").as_double();
    max_spd_jnts_ = this->get_parameter("max_spd_jnts").as_double();
    max_acc_jnts_ = this->get_parameter("max_acc_jnts").as_double();
    gripper_links_ = this->get_parameter("gripper_links").as_string_array();
    world_frame_ = prefix + this->get_parameter("world_frame").as_string();
    min_jacobian_determinant_ = this->get_parameter("min_jacobian_determinant").as_double();
    limit_joints_control_ = this->get_parameter("limit_joints_control").as_bool();
    limit_jacobian_control_ = this->get_parameter("limit_jacobian_control").as_bool();

    addPrefix(prefix, joint_names_);
    addPrefix(prefix, gripper_links_);

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

    auto cb_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );

    // Initialize service servers
    fkine_service_ = this->create_service<manipulator_interfaces::srv::FKine>(
        manipulator_name_ + "/get_fkine", 
        [this](const std::shared_ptr<manipulator_interfaces::srv::FKine::Request> request,
               std::shared_ptr<manipulator_interfaces::srv::FKine::Response> response) {
            this->getFKine_callback(request, response);
        },
        rmw_qos_profile_services_default,
        cb_group
    );

    invkine_service_ = this->create_service<manipulator_interfaces::srv::InvKine>(
        manipulator_name_ + "/get_invkine", 
        [this](const std::shared_ptr<manipulator_interfaces::srv::InvKine::Request> request,
               std::shared_ptr<manipulator_interfaces::srv::InvKine::Response> response) {
            this->getInvKine_callback(request, response);
        },
        rmw_qos_profile_services_default,
        cb_group
    );

    jacobian_service_ = this->create_service<manipulator_interfaces::srv::Jacobian>(
        manipulator_name_ + "/get_jacobian", 
        [this](const std::shared_ptr<manipulator_interfaces::srv::Jacobian::Request> request,
               std::shared_ptr<manipulator_interfaces::srv::Jacobian::Response> response) {
            this->getJacobian_callback(request, response);
        },
        rmw_qos_profile_services_default,
        cb_group
    );

    pseudoInverse_service_ = this->create_service<manipulator_interfaces::srv::PseudoInverse>(
        manipulator_name_ + "/get_pseudo_inverse", 
        [this](const std::shared_ptr<manipulator_interfaces::srv::PseudoInverse::Request> request,
               std::shared_ptr<manipulator_interfaces::srv::PseudoInverse::Response> response) {
            this->getPseudoInverseJacobian_callback(request, response);
        },
        rmw_qos_profile_services_default,
        cb_group
    );

    changePlannerScalingFactors_service_ = this->create_service<manipulator_interfaces::srv::ChangePlannerScalingFactors>(
        manipulator_name_ + "/change_planner_scaling_factors",
        [this](const std::shared_ptr<manipulator_interfaces::srv::ChangePlannerScalingFactors::Request> request,
               std::shared_ptr<manipulator_interfaces::srv::ChangePlannerScalingFactors::Response> response) {
            this->changePlannerScalingFactors_callback(request, response);
        },
        rmw_qos_profile_services_default,
        cb_group
    );

    changePlannerTolerances_service_ = this->create_service<manipulator_interfaces::srv::ChangePlannerTolerances>(
        manipulator_name_ + "/change_planner_tolerances",
        [this](const std::shared_ptr<manipulator_interfaces::srv::ChangePlannerTolerances::Request> request,
               std::shared_ptr<manipulator_interfaces::srv::ChangePlannerTolerances::Response> response) {
            this->changePlannerTolerances_callback(request, response);
        },
        rmw_qos_profile_services_default,
        cb_group
    );

    jointsRealTimeSetter_service_ = this->create_service<std_srvs::srv::SetBool>(
        manipulator_name_ + "/joints_real_time_setter",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            this->jointsRealTimeSetter_callback(request, response);
        },
        rmw_qos_profile_services_default,
        cb_group
    );

    jacobianControlSetter_service_ = this->create_service<std_srvs::srv::SetBool>(
        manipulator_name_ + "/jacobian_control_setter",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            this->jacobianControlSetter_callback(request, response);
        },
        rmw_qos_profile_services_default,
        cb_group
    );

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group;

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

    execution_ctrl_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        planning_group_ + "/execution_control", 1, 
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            this->executionControl_callback(msg);
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

    velJacSetpoint_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        manipulator_name_ + "/cmd_vel", 1, 
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            this->velJacSetpoint_callback(msg);
        },
        sub_options
    );

    realTimeSetpoint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        manipulator_name_ + "/js_cmd_vel", 1, 
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->realTimeSetpoint_callback(msg);
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
        },
        sub_options
    );

    tcpPose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>(manipulator_name_ + "/tcp_pose", 1);
    tcpVel_pub_  = this->create_publisher<geometry_msgs::msg::Twist>(manipulator_name_ + "/tcp_vel", 1);

    rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
        std::bind(&ManipulatorPlannerNode::shutdown_handler, this) // Register shutdown handler
    );
}

ManipulatorPlannerNode::~ManipulatorPlannerNode() {
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

    executor_.add_node(this->get_node_base_interface()); //Add the dynamic planner node to the executor

    rclcpp::Clock steady_clock(RCL_STEADY_TIME);

    auto main_cb_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );

    mainloop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / ros_freq_)),
        [&, this]() {

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

            rate.sleep();
        },
        main_cb_group
    );

    auto tcp_pose_cb_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );
    
    tcpPose_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / ros_freq_)),
        [&, this]() {
            // Publish tcp pose
            tcpPose_pub_->publish(getFKine());
        },
        tcp_pose_cb_group
    );

    auto tcp_vel_cb_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );

    tcpVel_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / ros_freq_)),
        [&, this]() {
            // Publish tcp vel
            tcpVel_pub_->publish(getTcpVel());
        },
        tcp_vel_cb_group
    );

    executor_.spin(); // Start the executor

    rclcpp::shutdown();
}

// ------------------------------------- PRIVATE METHODS -------------------------------------

void ManipulatorPlannerNode::shutdown_handler(){
    dynamic_planner_->stop(); // Stop the robot
    RCLCPP_INFO(get_logger(), "Spinner mean time: %f s", spinner_mean_);
}

//COLLISION OBJECTS

void ManipulatorPlannerNode::addCollisionObject(
    const std::string &object_name,
    const std::string &object_frame,
    const shape_msgs::msg::SolidPrimitive &object_primitive,
    const geometry_msgs::msg::Pose &object_pose
) {
    /*
    Adds a collision object to the planning scene
    Args:
        object_name: Name of the object
        object_frame: Frame in which the object must be placed
        object_primitive: Object primitive (shape)
        object_pose: Position of the object
    */
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = object_frame;
    collision_object.id = object_name;

    collision_object.primitives.push_back(object_primitive);
    collision_object.primitive_poses.push_back(object_pose);
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    dynamic_planner_->getPlanningScene()->applyCollisionObjects({collision_object});
}

void ManipulatorPlannerNode::addCollisionObject(
    const std::string &object_name,
    const std::string &object_frame,
    const ShapeType object_type,
    const std::vector<double> &object_dims,
    const geometry_msgs::msg::Pose &object_pose
) {
    /*
    Adds a collision object to the planning scene
    Args:
        object_name: Name of the object
        object_frame: Frame in which the object must be placed
        object_type: Type of the object (see ShapeType enum)
        object_dims: Dimensions for the object, depends on the object type
        object_pose: Position of the object
    */
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = object_type;

    setPrimitiveDimensions(object_type, object_dims, primitive);

    addCollisionObject(object_name, object_frame, primitive, object_pose); //Add the collision object to the planning scene
}

//ATTACHED COLLISION OBJECTS

void ManipulatorPlannerNode::addAttachedCollisionObject(
    const std::string &object_name,
    const shape_msgs::msg::SolidPrimitive &object_primitive,
    const geometry_msgs::msg::Pose &object_pose,
    const std::string &link_name,
    const std::vector<std::string> &disabled_collisions
) {
    /*
    Args:
        object_name: Name of the object
        object_primitive: Object primitive (shape)
        object_pose: Position of the object
        link_name: Name of the link to attach the object (if left empty, the 'ee_name' parameter is used)
        disabled_collisions: Array of links to disable collisions with the attached object
    */
    moveit_msgs::msg::CollisionObject attached_object;
    attached_object.id = object_name;

    std::string attached_link; //The link the object is attached to (end effector link by default, otherwise link_name)

    if (link_name.empty()) {
        attached_link = ee_name_;
    } else {
        attached_link = link_name;
    }

    //This is used so the pose of the object is relative to the link it is attached to
    attached_object.header.frame_id = attached_link;

    //Add primitive and pose
    attached_object.primitives.push_back(object_primitive);
    attached_object.primitive_poses.push_back(object_pose);
    attached_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    dynamic_planner_->getPlanningScene()->applyCollisionObject(attached_object); //Add to planning scene
    dynamic_planner_->getMoveGroup()->attachObject(object_name, attached_link, disabled_collisions); //Attach the object to the link
}

void ManipulatorPlannerNode::addAttachedCollisionObject(
    const std::string &object_name,
    const ShapeType object_type,
    const std::vector<double> &object_dims,
    const geometry_msgs::msg::Pose &object_pose,
    const std::string &link_name,
    const std::vector<std::string> &disabled_collisions
) {
    /*
    Args:
        object_name: Name of the object
        object_type: Type of the object (see ShapeType enum)
        object_dims: Dimensions for the object, depends on the object type
        object_pose: Position of the object
        link_name: Name of the link to attach the object (if left empty, the 'ee_name' parameter is used)
        disabled_collisions: Array of links to disable collisions with the attached object
    */
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = object_type;

    setPrimitiveDimensions(object_type, object_dims, primitive);

    addAttachedCollisionObject(object_name, primitive, object_pose, link_name, disabled_collisions);
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
    for (unsigned int k = 0; k < NUM_JOINTS; k++)
    {
        dq(k) = dynamic_planner_->joints_speed_group_[k];
    }

    // Compute the end-effector twist (linear and angular velocities) using the Jacobian
    jacobian_var_ = getJacobian();
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
        dynamic_planner_->moveRobot();
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
        dynamic_planner_->moveRobot();
    }
}

void ManipulatorPlannerNode::executionControl_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    //If true move the robot, otherwise stop
    if (msg->data)
    {
        dynamic_planner_->moveRobot();
    } else {
        dynamic_planner_->stop();
    }
}

void ManipulatorPlannerNode::collisionObject_callback(const moveit_msgs::msg::CollisionObject::SharedPtr collision_object) 
{
    // Add the collision object to the planning scene
    RCLCPP_INFO(this->get_logger(), "Received collision object: %s, operation: %d", collision_object->id.c_str(), collision_object->operation);
    moveit_msgs::msg::CollisionObject object = *collision_object.get();
    object.header.frame_id = world_frame_; // Set the frame id to the world frame
    
    dynamic_planner_->getPlanningScene()->applyCollisionObjects({object});
}

void ManipulatorPlannerNode::attachedCollisionObject_callback(const moveit_msgs::msg::AttachedCollisionObject::SharedPtr collision_object) 
{
    // Add the collision object to the planning scene and attach it to a link
    // If link_name is empty, the 'ee_name' parameter is used
    RCLCPP_INFO(this->get_logger(), "Received attached collision object: %s, operation: %d", collision_object->object.id.c_str(), collision_object->object.operation);
    std::string link_name = collision_object->link_name.empty() ? ee_name_ : collision_object->link_name;

    RCLCPP_INFO(get_logger(), "Attaching object to link: %s", link_name.c_str());
    
    collision_object->object.header.frame_id = link_name;

    dynamic_planner_->getPlanningScene()->applyCollisionObject(collision_object->object);
    dynamic_planner_->getMoveGroup()->attachObject(
        collision_object->object.id, 
        link_name, 
        gripper_links_
    );
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

    double fraction = dynamic_planner_->cartesianPlan(waypoints, ee_link_name, ref_frame);
    if (fraction < 0.01) 
    {
        RCLCPP_WARN(get_logger(), "Cartesian trajectory unfeasible");
    }
    else if (msg->execute)
    {
        dynamic_planner_->moveRobot();
    }
}

// Set the jacobian speed based control
void ManipulatorPlannerNode::jointsRealTimeSetter_callback(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res)
{
    // Set robot real time joints speed control
    js_rt_control_ = req->data;
    RCLCPP_INFO(get_logger(), "Joints real time control mode set as %s", js_rt_control_ ? "True":"False");

    // Stop the robot to prevent bad behaviours during mode switch
    for (unsigned int k = 0; k < joint_names_.size(); k++)
    {
        current_js_vel_[k]  = 0.;
    }
    for (unsigned int k = 0; k<6; k++)
    {
        current_ee_vel_[k] = 0.;
    }
    // Publish the msg to the robot
    jacobianControl();
    // Return success
    res->success = true;
    res->message = js_rt_control_ ? "Joints real time control mode enabled":"Joints real time control mode disabled";
}

// Set the jacobian speed based control
void ManipulatorPlannerNode::jacobianControlSetter_callback(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res)
{
    // Set robot jacobian control
    jac_control_ = req->data;
    RCLCPP_INFO(get_logger(), "Jacobian control mode set as %s", jac_control_ ? "True":"False");

    if (limit_jacobian_control_){
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

    // Stop the robot to prevent bad behaviours during mode switch
    for (unsigned int k = 0; k<6; k++) {current_ee_vel_(k) = 0.;}
    // Publish the msg to the robot
    jacobianControl();

    // Return success
    res->success = true;
    res->message = jac_control_ ? "Jacobian control mode enabled":"Jacobian control mode disabled";
}

// Update speed setpoint of the arm for the real time joints speed based control
void ManipulatorPlannerNode::realTimeSetpoint_callback(const sensor_msgs::msg::JointState::SharedPtr& msg)
{
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
    // Map the linear velocity
    ee_vel_cmd_[0] = msg->linear.x;
    ee_vel_cmd_[1] = msg->linear.y;
    ee_vel_cmd_[2] = msg->linear.z;
    
    // Map the angular velocity components from the Twist message
    ee_vel_cmd_[3] = msg->angular.x; // X component of angular velocity
    ee_vel_cmd_[4] = msg->angular.y; // Y component of angular velocity
    ee_vel_cmd_[5] = msg->angular.z; // Z component of angular velocity

    // Check if linear speed is below the maximum, otherwise scale velocity to fit
    double norm_linear = ee_vel_cmd_.head<3>().norm();
    if (norm_linear > max_speed_ee_) {ee_vel_cmd_ *= (max_speed_ee_/norm_linear);}

    // Check if angular speed is below the maximum, otherwise scale velocity to fit
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
    position_constraint.header.frame_id = world_frame_; // Set the frame id to the world frame

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
    Eigen::VectorXd dq(NUM_JOINTS);
    dq = jacobian_var_.completeOrthogonalDecomposition().pseudoInverse() * current_ee_vel_;
    
    // Set a lower limit to joint velocities to avoid noises
    for (unsigned int k = 0; k < NUM_JOINTS; k++) {setToZeroIfSmall(dq[k]);}
    
    // Convert joints state into Eigen::VectorXd
    Eigen::VectorXd q(NUM_JOINTS);
    for (unsigned int k = 0; k < NUM_JOINTS; k++)
    {
        q(k) = dynamic_planner_->joints_values_group_[k];
    }

    // Update joint position setpoint
    Eigen::VectorXd qd(NUM_JOINTS);
    qd = q + dq / ros_freq_;

    // Build the msg for the joints setpoint
    sensor_msgs::msg::JointState js;
    js.name     = joint_names_;
    js.position.resize(qd.size());
    js.velocity.resize(dq.size());
    
    // Insert positions and velocity setpoints
    for (unsigned int k = 0; k < NUM_JOINTS; k++)
    {
        js.position[k] = qd[k];
        js.velocity[k] = dq[k];
    }

    bool jac_check = true;
    bool constraints_check = true;

    if (min_jacobian_determinant_ > 0.0){
        Eigen::MatrixXd jacobian = getJacobian(js.position, ee_name_); // Compute the jacobian matrix for the new position
        jac_check = abs(jacobian.determinant()) >= min_jacobian_determinant_; // Jacobian check failed
    }

    if (limit_jacobian_control_) {
        // If the jacobian control is limited, check if the joint constraints are violated
        for (unsigned int k = 0; k < constraints_primitives_.size(); k++) {
            // Check if the position constraints are violated
            if (!isPoseInsidePrimitive(
                dynamic_planner_->getFKine(js.position, ee_name_).pose,
                constraints_primitives_[k], 
                constraints_poses_[k]
            )) {
                constraints_check = false;
                break; // Exit the loop if a constraint is violated
            }
        }
    }

    if (!jac_check || !constraints_check) {
        js.position = dynamic_planner_->joints_values_group_; // Set the position to the current one
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
    for (unsigned int k = 0; k < NUM_JOINTS; k++)
    {
        q[k] = dynamic_planner_->joints_values_group_[k];
    }
        
    // Set a lower limit to joint velocities to avoid noises
    for (unsigned int k = 0; k < 6; k++) {setToZeroIfSmall(current_js_vel_[k]);}

    // Update joint position setpoint
    Eigen::VectorXd qd(NUM_JOINTS);
    qd = q + current_js_vel_ / ros_freq_;

    // Build the msg for the joints setpoint
    sensor_msgs::msg::JointState js;
    js.name = joint_names_;
    js.position.resize(qd.size());
    js.velocity.resize(current_js_vel_.size());

    // Insert positions and velocity setpoints
    for (unsigned int k = 0; k < NUM_JOINTS; k++)
    {
        js.position[k] = qd[k];
        js.velocity[k] = current_js_vel_[k];
    }

    if (limit_joints_control_) {
        // If the joint control is limited, check if the joint constraints are violated
        if (!dynamic_planner_->checkJointConstraints(js.position)) {
            RCLCPP_WARN(get_logger(), "Joint constraints violated, stopping the robot");
            js.position = dynamic_planner_->joints_values_group_; // Set the position to the current one
            js.velocity = std::vector<double>(NUM_JOINTS, 0.0); // Set the velocity to zero
        }
    }

    // Send the goal to the move it fake controller as trajectory point
    dynamic_planner_->moveRobot(js);
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

void ManipulatorPlannerNode::declareParameters() {
    this->declare_parameter("manipulator_name", std::string());
    this->declare_parameter("planning_group", std::string());
    this->declare_parameter("joint_names", std::vector<std::string>());
    this->declare_parameter("ee_name", std::string());
    this->declare_parameter("world_frame", "base_link");
    this->declare_parameter("ros_freq", 500);
    this->declare_parameter("max_speed_ee", 1.0);
    this->declare_parameter("max_accel_ee", 1.0);
    this->declare_parameter("max_rot_speed_ee", 1.0);
    this->declare_parameter("max_rot_accel_ee", 1.0);
    this->declare_parameter("max_spd_jnts", 1.0);
    this->declare_parameter("max_acc_jnts", 1.0);
    this->declare_parameter("gripper_links", std::vector<std::string>()); //This is used to disable collision with the fingers when attaching objects
    this->declare_parameter("prefix", std::string()); //Prefix for the joint and link names
    this->declare_parameter("min_jacobian_determinant", 0.0); //Minimum determinant for the inverse jacobian (0 is disabled)
    this->declare_parameter("limit_joints_control", false);
    this->declare_parameter("limit_jacobian_control", false);

    //Dynamic planner params
    this->declare_parameter("planner_id", "geometric::RRTConnect");
    this->declare_parameter("vel_factor", 0.1); //MUTABLE
    this->declare_parameter("acc_factor", 0.1); //MUTABLE
    this->declare_parameter("max_planning_time", 2.0);
    this->declare_parameter("max_planning_attempts", 2);
    this->declare_parameter("position_tolerance", 0.01);
    this->declare_parameter("orientation_tolerance", 0.01);
    this->declare_parameter("joint_tolerance", 0.01);
}
 
void ManipulatorPlannerNode::initializePlanner() {
    //Initialize the dynamic planner
    DynamicPlannerParams params;

    params.vel_factor = this->get_parameter("vel_factor").as_double();
    params.acc_factor = this->get_parameter("acc_factor").as_double();
    params.planning_time = this->get_parameter("max_planning_time").as_double();
    params.num_attempts = this->get_parameter("max_planning_attempts").as_int();
    params.position_tolerance = this->get_parameter("position_tolerance").as_double();
    params.orientation_tolerance = this->get_parameter("orientation_tolerance").as_double();
    params.joint_tolerance = this->get_parameter("joint_tolerance").as_double();
    params.planner_id = this->get_parameter("planner_id").as_string();
    params.sample_time = 1 / ros_freq_;
    params.max_velocity = max_speed_ee_;
    params.world_frame = world_frame_;
    params.end_effector_link = ee_name_;
    
    auto sub_node = rclcpp::Node::make_shared("dynamic_planner_node");
    executor_.add_node(sub_node->get_node_base_interface());
    dynamic_planner_ = std::make_shared<DynamicPlanner>(sub_node,
                                                        planning_group_,
                                                        params,
                                                        false);
}

void ManipulatorPlannerNode::setPrimitiveDimensions(const ShapeType object_type, const std::vector<double> &object_dims, shape_msgs::msg::SolidPrimitive &primitive)
{
    /*
    Sets the appropriate dimensions for the primitive object depending on the object type
    Args:
        object_type: Type of the object (see ShapeType enum)
        object_dims: Dimensions for the object, depends on the object type
        primitive: SolidPrimitive object to set the dimensions
    */
    switch(object_type) {
        case ShapeType::BOX:
            if (object_dims.size() != 3) {
                RCLCPP_WARN(this->get_logger(), "Object dimensions array is not compatible with object type");
                return;
            }

            primitive.dimensions.resize(3);
            primitive.dimensions[0] = object_dims[0];
            primitive.dimensions[1] = object_dims[1];
            primitive.dimensions[2] = object_dims[2];
            break;

        case ShapeType::SPHERE:
            if (object_dims.size() != 1) {
                RCLCPP_WARN(this->get_logger(), "Object dimensions array is not compatible with object type");
                return;
            }

            primitive.dimensions.resize(1);
            primitive.dimensions[0] = object_dims[0];
            break;

        case ShapeType::CYLINDER:
            if (object_dims.size() != 2) {
                RCLCPP_WARN(this->get_logger(), "Object dimensions array is not compatible with object type");
                return;
            }

            primitive.dimensions.resize(2);
            primitive.dimensions[0] = object_dims[0];
            primitive.dimensions[1] = object_dims[1];
            break;

        case ShapeType::CONE:
            if (object_dims.size() != 2) {
                RCLCPP_WARN(this->get_logger(), "Object dimensions array is not compatible with object type");
                return;
            }

            primitive.dimensions.resize(2);
            primitive.dimensions[0] = object_dims[0];
            primitive.dimensions[1] = object_dims[1];
            break;

        default:
            RCLCPP_WARN(this->get_logger(), "Invalid object type, check the ShapeType enum");
            return;
    }
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