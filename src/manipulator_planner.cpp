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
    max_spd_jnts_ = this->get_parameter("max_spd_jnts").as_double();
    max_acc_jnts_ = this->get_parameter("max_acc_jnts").as_double();
    gripper_links_ = this->get_parameter("gripper_links").as_string_array();
    world_frame_ = prefix + this->get_parameter("world_frame").as_string();

    addPrefix(prefix, joint_names_);
    addPrefix(prefix, gripper_links_);

    //Initialize velocity variables
    const size_t NUM_JOINTS = joint_names_.size();

    if (NUM_JOINTS == 0) {
        RCLCPP_ERROR(this->get_logger(), "No joint names provided");
        return;
    }

    //Initialize the velocity command vectors
    js_vel_cmd_.resize(NUM_JOINTS, 1);
    js_vel_cmd_.setZero();
    js_msg_new_.resize(NUM_JOINTS, 1);
    js_msg_new_.setZero();
    arm_vel_cmd_.resize(6, 1);
    arm_vel_cmd_.setZero();
    arm_msg_new_.resize(3, 1);
    arm_msg_new_.setZero();

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

    cartesianPlan_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        manipulator_name_ + "/cartesian_plan", 1, 
        [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
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

    // Initialize publishers
    j0_pub_ = this->create_publisher<std_msgs::msg::Float64>(manipulator_name_ + "/" + joint_names_[0] + "/motor_control", 1);
    j1_pub_ = this->create_publisher<std_msgs::msg::Float64>(manipulator_name_ + "/" + joint_names_[1] + "/motor_control", 1);
    j2_pub_ = this->create_publisher<std_msgs::msg::Float64>(manipulator_name_ + "/" + joint_names_[2] + "/motor_control", 1);
    j3_pub_ = this->create_publisher<std_msgs::msg::Float64>(manipulator_name_ + "/" + joint_names_[3] + "/motor_control", 1);
    j4_pub_ = this->create_publisher<std_msgs::msg::Float64>(manipulator_name_ + "/" + joint_names_[4] + "/motor_control", 1);
    j5_pub_ = this->create_publisher<std_msgs::msg::Float64>(manipulator_name_ + "/" + joint_names_[5] + "/motor_control", 1);

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
        rclcpp::CallbackGroupType::Reentrant
    );

    mainloop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / ros_freq_)),
        [&, this]() {
            // This is the main loop for the node
            auto start_time = steady_clock.now();

            if (jac_control_) {
                // Jacobian control
                jacobianControl();    
            }
            else if (js_rt_control_) {
                // Real-time joint speed control
                jointsRealTimeControl();
            }
            
            // Calculate the mean time for each iteration of the spinner
            double elapsed_time = (steady_clock.now() - start_time).seconds();
            spinner_mean_ = (spinner_mean_ * static_cast<double>(num_samples) + elapsed_time) / static_cast<double>(num_samples + 1);
            num_samples++;

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
    const unsigned int NUM_JOINTS = joint_names_.size();
    Eigen::VectorXd dq(NUM_JOINTS);
    for (unsigned int k = 0; k < NUM_JOINTS; k++)
    {
        dq(k) = dynamic_planner_->joints_speed_group_[k];
    }

    // Compute the end-effector twist (linear and angular velocities) using the Jacobian
    Eigen::VectorXd twist = getJacobian() * dq;

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

const Eigen::MatrixXd ManipulatorPlannerNode::getJacobian(const std::string &end_effector_link) {
    /*
    Computes the jacobian matrix
    Args:
        end_effector_link: Name of the end effector link to which the jacobian is referred
    */
    return dynamic_planner_->getJacobian(end_effector_link);
}

const Eigen::MatrixXd ManipulatorPlannerNode::getJacobian() {
    /*
    Computes the jacobian matrix
    "ee_name" parameter is used as the end effector link
    */
    return getJacobian(ee_name_);
}

const Eigen::MatrixXd ManipulatorPlannerNode::getPseudoInverseJacobian(const std::string &end_effector_link) {
    /*
    Computes the Moore-Penrose pseudo-inverse of the jacobian matrix
    Args:
        end_effector_link: Name of the end effector link to which the jacobian is referred
    */
    return dynamic_planner_->getPseudoInverseJacobian(end_effector_link);
}

const Eigen::MatrixXd ManipulatorPlannerNode::getPseudoInverseJacobian() {
    /*
    Computes the Moore-Penrose pseudo-inverse of the jacobian matrix
    "ee_name" parameter is used as the end effector link
    */
    return getPseudoInverseJacobian(ee_name_);
}

//CALLBACK FUNCTIONS
void ManipulatorPlannerNode::getFKine_callback(
    const std::shared_ptr<manipulator_interfaces::srv::FKine::Request> request, 
    std::shared_ptr<manipulator_interfaces::srv::FKine::Response> response
) {
    /*
    Callback function for the forward kinematics service
    Interface:
        request: 
            joint_state (sensor_msgs/JointState): Joint state to compute the forward kinematics, 
                                                  if empty the current joint state is used
        response: 
            tcp_pose (geometry_msgs/PoseStamped): End effector pose
    */
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
    /*
    Callback function for the inverse kinematics service
    Interface:
        request: 
            target_pose: (geometry_msgs/Pose): End effector pose
        response: 
            joint_values           (float64[]): inverse kinematics result
    */
    response->joint_values = dynamic_planner_->invKine(
        request->target_pose, 
        ee_name_
    );
}

void ManipulatorPlannerNode::getJacobian_callback(
    const std::shared_ptr<manipulator_interfaces::srv::Jacobian::Request> request,
    std::shared_ptr<manipulator_interfaces::srv::Jacobian::Response> response
) {
    /*
    Callback function for the jacobian service
    Interface:
        request: None
        response: 
            matrix_values (float64[]): Flattened jacobian matrix
    */
    request.get(); //Suppress unused var warning

    Eigen::MatrixXd jacobian = dynamic_planner_->getJacobian(ee_name_);
    std::vector<double> jacobian_values(jacobian.data(), jacobian.data() + jacobian.size()); //Flattens the matrix into a vector

    response->matrix_values = jacobian_values;
}

void ManipulatorPlannerNode::getPseudoInverseJacobian_callback(
    const std::shared_ptr<manipulator_interfaces::srv::PseudoInverse::Request> request,
    std::shared_ptr<manipulator_interfaces::srv::PseudoInverse::Response> response
) {
    /*
    Callback function for the pseudo-inverse jacobian service
    Interface:
        request: None
        response: 
            matrix_values (float64[]): Flattened pseudo-inverse jacobian matrix
    */
    request.get(); //Suppress unused var warning

    Eigen::MatrixXd pseudo_inv = getPseudoInverseJacobian();
    std::vector<double> pseudo_inv_values(pseudo_inv.data(), pseudo_inv.data() + pseudo_inv.size()); //Flattens the matrix into a vector

    response->matrix_values = pseudo_inv_values;
}

void ManipulatorPlannerNode::changePlannerScalingFactors_callback(
    const std::shared_ptr<manipulator_interfaces::srv::ChangePlannerScalingFactors::Request> request,
    std::shared_ptr<manipulator_interfaces::srv::ChangePlannerScalingFactors::Response> response
) {
    /*
    Callback function for the change planner parameters service
    Interface:
        request: 
            vel_factor (float64): Velocity factor
            acc_factor (float64): Acceleration factor
        response: 
            success       (bool): True if the parameters were changed successfully
    */

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
    /*
    Callback function for the change planner parameters service
    Interface:
        request: 
            position_tolerance      (float64): Tolerance for tcp position
            orientation_tolerance   (float64): Tolerance for tcp orientation
            joint_tolerance         (float64): Tolerance for joint positions
        response: 
            success                    (bool): True if the parameters were changed successfully
    */

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
    /*
    Callback function for the TCP goal subscriber, plans and moves the robot to the goal pose
    Interface TcpGoal:
        start_state (sensor_msgs/JointState): start joints positions
        target_pose     (geometry_msgs/Pose): target position for the end effector
        end_effector                (string): the end effector which will reach the target_pose
        frame                       (string): the reference frame for the pose
        execute                       (bool): wether to execute the planned trajectory rightaway
    */
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
    /*
    Callback function for the joint goal subscriber, plans and moves the robot to the goal joint positions
    Interface JointGoal:
        start_state     (sensor_msgs/JointState): start joints positions
        joint_goal      (sensor_msgs/JointState): joints positions to reach
        execute         (bool): wether to execute the planned trajectory rightaway
    */

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
void ManipulatorPlannerNode::cartesianPlan_callback(const geometry_msgs::msg::PoseArray::SharedPtr p_seq)
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for (geometry_msgs::msg::Pose point : p_seq->poses)
    {
        waypoints.push_back(point);
    }

    // Send to joint goal dynamic planner V4
    double fraction = dynamic_planner_->cartesianPlan(waypoints);
    if (fraction < 0.01) {RCLCPP_WARN(get_logger(), "Cartesian trajectory unfeasible");}
    else { dynamic_planner_->moveRobot(); }
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
        js_vel_cmd_[k]  = 0.;
    }
    for (unsigned int k = 0; k<6; k++)
    {
        arm_vel_cmd_[k] = 0.;
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
    // Stop the robot to prevent bad behaviours during mode switch
    for (unsigned int k = 0; k<6; k++) {arm_vel_cmd_(k) = 0.;}
    // Publish the msg to the robot
    jacobianControl();
    // Return success
    res->success = true;
    res->message = jac_control_ ? "Jacobian control mode enabled":"Jacobian control mode disabled";
}

// Update speed setpoint of the arm for the real time joints speed based control
void ManipulatorPlannerNode::realTimeSetpoint_callback(const sensor_msgs::msg::JointState::SharedPtr& msg)
{
    for (unsigned int k = 0; k < joint_names_.size(); k++)
    {
        // Update setpoint of the k-th joint
        js_msg_new_[k] = msg->velocity[k];
        // Check if the vel cmds exceed the maximum acceptable speed
        if (abs(msg->velocity[k]) > max_spd_jnts_)
        {
            js_msg_new_[k] = sign(msg->velocity[k])*max_spd_jnts_;
        }
    }
}

// Update the velocity setpoint of the arm for the jacobian speed based control
void ManipulatorPlannerNode::velJacSetpoint_callback(const geometry_msgs::msg::Twist::SharedPtr& msg)
{
    // Compute the norm of the linear vels components of the new msg
    arm_msg_new_[0] = msg->linear.x;
    arm_msg_new_[1] = msg->linear.y;
    arm_msg_new_[2] = msg->linear.z;
    // arm_vel_cmd_[0] = msg->linear.x;
    // arm_vel_cmd_[1] = msg->linear.y;
    // arm_vel_cmd_[2] = msg->linear.z;
    
    // Map the angular velocity components from the Twist message
    arm_vel_cmd_[3] = msg->angular.x; // X component of angular velocity
    arm_vel_cmd_[4] = msg->angular.y; // Y component of angular velocity
    arm_vel_cmd_[5] = msg->angular.z; // Z component of angular velocity

    // Check if the speed is below the maximum
    double norm_vel = arm_msg_new_.norm();
    if (norm_vel > max_speed_ee_) {arm_msg_new_ *= (max_speed_ee_/norm_vel);}
    // double norm_vel = arm_vel_cmd_.head<3>().norm();
    // if (norm_vel > max_speed_ee_) {arm_vel_cmd_ *= (max_speed_ee_/norm_vel);}
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

// Set the the motors' position and speed through the controllers
void ManipulatorPlannerNode::motorsController(const sensor_msgs::msg::JointState &js)
{
  std_msgs::msg::Float64 msg;
  msg.data = js.position[0];
  j0_pub_->publish(msg);
  msg.data = js.position[1];
  j1_pub_->publish(msg);
  msg.data = js.position[2];
  j2_pub_->publish(msg);
  msg.data = js.position[3];
  j3_pub_->publish(msg);
  msg.data = js.position[4];
  j4_pub_->publish(msg);
  msg.data = js.position[5];
  j5_pub_->publish(msg);
}

void setToZeroIfSmall(double &value)
{
    if (std::abs(value) < 1e-20) {value = 0.0;}
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
    // Compute the norm of the current linear vels components
    double norm_vel = arm_vel_cmd_.head<3>().norm();
    
    // Compute the norm of the linear vels components of the new msg
    double norm_msg = arm_msg_new_.norm();

    // Check if the acceleration is over the maximum, set a limitation
    if (abs(norm_msg-norm_vel)*ros_freq_ > max_accel_ee_)
    {
        // Split the acceleration over the three axes
        double acc_x = max_accel_ee_/3;
        double acc_y = max_accel_ee_/3;
        double acc_z = max_accel_ee_/3;
        if      (norm_msg > 0.001)  // norm_msg < -0.001 || 
        {
            acc_x = max_accel_ee_*abs(arm_msg_new_(0))/norm_msg;
            acc_y = max_accel_ee_*abs(arm_msg_new_(1))/norm_msg;
            acc_z = max_accel_ee_*abs(arm_msg_new_(2))/norm_msg;
            // Update linear velocity components
            arm_vel_cmd_(0) = arm_vel_cmd_(0) + sign(arm_msg_new_(0)-arm_vel_cmd_(0))*acc_x/ros_freq_;
            arm_vel_cmd_(1) = arm_vel_cmd_(1) + sign(arm_msg_new_(1)-arm_vel_cmd_(1))*acc_y/ros_freq_;
            arm_vel_cmd_(2) = arm_vel_cmd_(2) + sign(arm_msg_new_(2)-arm_vel_cmd_(2))*acc_z/ros_freq_;
        }
        else if (norm_vel > 0.001)  // norm_vel < -0.001 || 
        {
            acc_x = max_accel_ee_*(abs(arm_vel_cmd_[0]))/norm_vel;
            acc_y = max_accel_ee_*(abs(arm_vel_cmd_[1]))/norm_vel;
            acc_z = max_accel_ee_*(abs(arm_vel_cmd_[2]))/norm_vel;
            
            // Update linear velocity components
            arm_vel_cmd_(0) = arm_vel_cmd_(0) + sign(arm_msg_new_(0)-arm_vel_cmd_(0))*acc_x/ros_freq_;
            arm_vel_cmd_(1) = arm_vel_cmd_(1) + sign(arm_msg_new_(1)-arm_vel_cmd_(1))*acc_y/ros_freq_;
            arm_vel_cmd_(2) = arm_vel_cmd_(2) + sign(arm_msg_new_(2)-arm_vel_cmd_(2))*acc_z/ros_freq_;
        }
        else
        {
            arm_vel_cmd_.head<3>() = arm_msg_new_;
        }

    }
    else {arm_vel_cmd_.head<3>() = arm_msg_new_;}
    
    // Set a lower limit to velocities to avoid noises
    for (unsigned int k = 0; k < 6; k++) {setToZeroIfSmall(arm_vel_cmd_[k]);}
    
    // Compute the speed
    const unsigned int NUM_JOINTS = joint_names_.size();
    Eigen::VectorXd dq(NUM_JOINTS);
    dq = getPseudoInverseJacobian() * arm_vel_cmd_;
    
    // Set a lower limit to joint velocities to avoid noises
    for (unsigned int k = 0; k < 6; k++) {setToZeroIfSmall(dq[k]);}
    
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
    
    // Send the goal to the move it fake controller as trajectory point
    dynamic_planner_->moveRobot(js);
}

// Execute the jacobian based control
void ManipulatorPlannerNode::jointsRealTimeControl()
{
    const unsigned int NUM_JOINTS = joint_names_.size();

    // Check if the accelerations are acceptable and map the joints speed from the JointState message
    for (unsigned int k = 0; k < NUM_JOINTS; k++)
    {
        if (abs(js_msg_new_[k]-js_vel_cmd_[k])*ros_freq_ > max_acc_jnts_)
            {js_vel_cmd_[k] = js_vel_cmd_[k] + sign(js_msg_new_[k]-js_vel_cmd_[k])*max_acc_jnts_/ros_freq_;}
        else  {js_vel_cmd_[k] = js_msg_new_[k];}
    }

    // Convert joints state into Eigen::MatrixXd
    Eigen::VectorXd q(NUM_JOINTS);
    for (unsigned int k = 0; k < NUM_JOINTS; k++)
    {
        q[k] = dynamic_planner_->joints_values_group_[k];
    }
        
    // Set a lower limit to joint velocities to avoid noises
    for (unsigned int k = 0; k < 6; k++) {setToZeroIfSmall(js_vel_cmd_[k]);}

    // Update joint position setpoint
    Eigen::VectorXd qd(NUM_JOINTS);
    qd = q + js_vel_cmd_ / ros_freq_;

    // Build the msg for the joints setpoint
    sensor_msgs::msg::JointState js;
    js.name     = joint_names_;
    js.position.resize(qd.size());
    js.velocity.resize(js_vel_cmd_.size());

    // Insert positions and velocity setpoints
    for (unsigned int k = 0; k < NUM_JOINTS; k++)
    {
        js.position[k] = qd[k];
        js.velocity[k] = js_vel_cmd_[k];
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
    this->declare_parameter("max_spd_jnts", 1.0);
    this->declare_parameter("max_acc_jnts", 1.0);
    this->declare_parameter("gripper_links", std::vector<std::string>()); //This is used to disable collision with the fingers when attaching objects
    this->declare_parameter("prefix", std::string()); //Prefix for the joint and link names

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