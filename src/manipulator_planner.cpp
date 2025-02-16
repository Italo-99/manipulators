#include "manipulators/ManipulatorPlanner.h"

// ------------------------------------- PUBLIC METHODS -------------------------------------

ManipulatorPlannerNode::ManipulatorPlannerNode(const std::string node_name, const rclcpp::NodeOptions &options) 
: rclcpp::Node(node_name, options), node_name_(node_name) {

    declareParameters();

    //Initialize velocity variables
    const size_t NUM_JOINTS = this->get_parameter("joint_names").as_string_array().size();

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

    const std::string manipulator_name = this->get_parameter("manipulator_name").as_string();

    // Initialize service servers
    fkine_service_ = this->create_service<manipulator_interfaces::srv::FKine>(
        manipulator_name + "/get_fkine", 
        [this](const std::shared_ptr<manipulator_interfaces::srv::FKine::Request> request,
               std::shared_ptr<manipulator_interfaces::srv::FKine::Response> response) {
            this->getFKine_callback(request, response);
        }
    );

    invkine_service_ = this->create_service<manipulator_interfaces::srv::InvKine>(
        manipulator_name + "/get_invkine", 
        [this](const std::shared_ptr<manipulator_interfaces::srv::InvKine::Request> request,
               std::shared_ptr<manipulator_interfaces::srv::InvKine::Response> response) {
            this->getInvKine_callback(request, response);
        }
    );

    jacobian_service_ = this->create_service<manipulator_interfaces::srv::Jacobian>(
        manipulator_name + "/get_jacobian", 
        [this](const std::shared_ptr<manipulator_interfaces::srv::Jacobian::Request> request,
               std::shared_ptr<manipulator_interfaces::srv::Jacobian::Response> response) {
            this->getJacobian_callback(request, response);
        }
    );

    pseudoInverse_service_ = this->create_service<manipulator_interfaces::srv::PseudoInverse>(
        manipulator_name + "/get_pseudo_inverse", 
        [this](const std::shared_ptr<manipulator_interfaces::srv::PseudoInverse::Request> request,
               std::shared_ptr<manipulator_interfaces::srv::PseudoInverse::Response> response) {
            this->getPseudoInverseJacobian_callback(request, response);
        }
    );

    changePlannerParams_service_ = this->create_service<manipulator_interfaces::srv::ChangePlannerParameters>(
        manipulator_name + "/change_planner_params",
        [this](const std::shared_ptr<manipulator_interfaces::srv::ChangePlannerParameters::Request> request,
               std::shared_ptr<manipulator_interfaces::srv::ChangePlannerParameters::Response> response) {
            this->changePlannerParams_callback(request, response);
        }
    );

    instantKineSetter_service_ = this->create_service<std_srvs::srv::SetBool>(
        manipulator_name + "/instKine_setter",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            this->instantKineSetter_callback(request, response);
        }
    );

    jointsRealTimeSetter_service_ = this->create_service<std_srvs::srv::SetBool>(
        manipulator_name + "/joints_real_time_setter",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            this->jointsRealTimeSetter_callback(request, response);
        }
    );

    jacobianControlSetter_service_ = this->create_service<std_srvs::srv::SetBool>(
        manipulator_name + "/jacobian_control_setter",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
            this->jacobianControlSetter_callback(request, response);
        }
    );

    // Initialize clients

    instantKineSetter_client_ = this->create_client<std_srvs::srv::SetBool>(manipulator_name + "/instKine_setter");

    // Initialize subscribers
    tcpGoal_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        manipulator_name + "/tcp_goal", 1, 
        [this](const geometry_msgs::msg::Pose::SharedPtr msg) {
            this->tcpGoal_callback(msg);
        }
    );

    jointGoal_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        manipulator_name + "/joint_goal", 1, 
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->jointGoal_callback(msg);
        }
    );

    collisionObject_sub_ = this->create_subscription<moveit_msgs::msg::CollisionObject>(
        manipulator_name + "/collision_object", 1, 
        [this](const moveit_msgs::msg::CollisionObject::SharedPtr msg) {
            this->collisionObject_callback(msg);
        }
    );

    attachedcollisionObject_sub_ = this->create_subscription<moveit_msgs::msg::AttachedCollisionObject>(
        manipulator_name + "/attached_collision_object", 1, 
        [this](const moveit_msgs::msg::AttachedCollisionObject::SharedPtr msg) {
            this->attachedCollisionObject_callback(msg);
        }
    );

    cartesianPlan_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        manipulator_name + "/desired_cartesian_move", 1, 
        [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
            this->cartesianPlan_callback(msg);
        }
    );

    velJacSetpoint_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        manipulator_name + "/cmd_vel", 1, 
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            this->velJacSetpoint_callback(msg);
        }
    );

    realTimeSetpoint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        manipulator_name + "/js_cmd_vel", 1, 
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->realTimeSetpoint_callback(msg);
        }
    );

    // Initialize publishers
    std::vector<std::string> joint_names = this->get_parameter("joint_names").as_string_array();

    j0_pub_ = this->create_publisher<std_msgs::msg::Float64>(manipulator_name + "/" + joint_names[0] + "/motor_control", 1);
    j1_pub_ = this->create_publisher<std_msgs::msg::Float64>(manipulator_name + "/" + joint_names[1] + "/motor_control", 1);
    j2_pub_ = this->create_publisher<std_msgs::msg::Float64>(manipulator_name + "/" + joint_names[2] + "/motor_control", 1);
    j3_pub_ = this->create_publisher<std_msgs::msg::Float64>(manipulator_name + "/" + joint_names[3] + "/motor_control", 1);
    j4_pub_ = this->create_publisher<std_msgs::msg::Float64>(manipulator_name + "/" + joint_names[4] + "/motor_control", 1);
    j5_pub_ = this->create_publisher<std_msgs::msg::Float64>(manipulator_name + "/" + joint_names[5] + "/motor_control", 1);
}

ManipulatorPlannerNode::~ManipulatorPlannerNode() {
    // Stop the spinner thread
    executor_.cancel();
}

void ManipulatorPlannerNode::spinner() {
    unsigned long long int num_samples = 0; // Number of samples for the mean time calculation

    rclcpp::Rate rate(this->get_parameter("ros_freq").as_int());

    initializePlanner(); //Initialize dynamic_planner_

    //This creates a different thread for the executor 
    //It's necessary because the node which hosts the move group interface must be always spinning
    std::thread executor_thread = std::thread([this] {
        executor_.spin();
    });

    //This is the spinner for the main node of the manipulator_planner
    while (rclcpp::ok()) {
        auto start_time = std::chrono::high_resolution_clock::now();

        // Jacobian speed control
        if (jac_control_) {
            // Publish tcp pose
            tcpPose_pub_->publish(getFKine());
            // Publish tcp speed
            tcpVel_pub_->publish(getTcpVel());
            // Update vel cmd
            jacobianControl();    
        }
        else if (js_rt_control_) {
            // Publish tcp pose
            tcpPose_pub_->publish(getFKine());
            // Publish tcp speed
            tcpVel_pub_->publish(getTcpVel());
            // Update joints vel command
            jointsRealTimeControl();

        }

        rclcpp::spin_some(this->shared_from_this());

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;
        
        // Calculate the mean time for each iteration of the spinner
        spinner_mean_ = (spinner_mean_ * static_cast<double>(num_samples) + elapsed_time.count()) / static_cast<double>(num_samples + 1);
        num_samples++;

        rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(), "Spinner mean time: %f ms", spinner_mean_);
}

// ------------------------------------- PRIVATE METHODS -------------------------------------

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
        attached_link = this->get_parameter("ee_name").as_string();
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
    return dynamic_planner_->getFKine(this->get_parameter("ee_name").as_string()).pose;
}

// Get the tcp twist by multiplying joints vels with the jacobian
const geometry_msgs::msg::Twist ManipulatorPlannerNode::getTcpVel()
{
    // Initialize dq with the appropriate size and assign values
    const unsigned int NUM_JOINTS = get_parameter("joint_names").as_string_array().size();
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
    return getJacobian(this->get_parameter("ee_name").as_string());
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
    return getPseudoInverseJacobian(this->get_parameter("ee_name").as_string());
}

//CALLBACK FUNCTIONS
void ManipulatorPlannerNode::getFKine_callback(
    const std::shared_ptr<manipulator_interfaces::srv::FKine::Request> request, 
    std::shared_ptr<manipulator_interfaces::srv::FKine::Response> response
) {
    /*
    Callback function for the forward kinematics service
    Interface:
        request: None
        response: 
            geometry_msgs::msg::PoseStamped tcp_pose: End effector pose
    */
    request.get(); //Suppress unused var warning
    response->tcp_pose = dynamic_planner_->getFKine(this->get_parameter("ee_name").as_string());
}


void ManipulatorPlannerNode::getInvKine_callback(
    const std::shared_ptr<manipulator_interfaces::srv::InvKine::Request> request,
    std::shared_ptr<manipulator_interfaces::srv::InvKine::Response> response
) {
    /*
    Callback function for the inverse kinematics service
    Interface:
        request: 
            geometry_msgs::msg::Pose target_pose: End effector pose
        response: 
            float64[] joint_values: inverse kinematics result
    */
    response->joint_values = dynamic_planner_->invKine(
        request->target_pose, 
        this->get_parameter("ee_name").as_string()
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
            float64[] matrix_values: Flattened jacobian matrix
    */
    request.get(); //Suppress unused var warning

    Eigen::MatrixXd jacobian = dynamic_planner_->getJacobian(this->get_parameter("ee_name").as_string());
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
            float64[] matrix_values: Flattened pseudo-inverse jacobian matrix
    */
    request.get(); //Suppress unused var warning

    Eigen::MatrixXd pseudo_inv = getPseudoInverseJacobian();
    std::vector<double> pseudo_inv_values(pseudo_inv.data(), pseudo_inv.data() + pseudo_inv.size()); //Flattens the matrix into a vector

    response->matrix_values = pseudo_inv_values;
}

void ManipulatorPlannerNode::changePlannerParams_callback(
    const std::shared_ptr<manipulator_interfaces::srv::ChangePlannerParameters::Request> request,
    std::shared_ptr<manipulator_interfaces::srv::ChangePlannerParameters::Response> response
) {
    /*
    Callback function for the change planner parameters service
    Interface:
        request: 
            float64 vel_factor: Velocity factor
            float64 acc_factor: Acceleration factor
        response: 
            bool success: True if the parameters were changed successfully
    */

    DynamicPlannerParams params = dynamic_planner_->getParams();
    params.acc_factor = request->acc_factor;
    params.vel_factor = request->vel_factor;
    dynamic_planner_->setParams(params);
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Planner parameters changed successfully (vel_factor: %f, acc_factor: %f)", params.vel_factor, params.acc_factor);
}

void ManipulatorPlannerNode::tcpGoal_callback(const geometry_msgs::msg::Pose::SharedPtr msg) 
{
    /*
    Callback function for the TCP goal subscriber
    Args:
        msg: TCP goal pose
    */
    RCLCPP_INFO(this->get_logger(), "Received TCP goal");

    dynamic_planner_->stop(); //Stop the execution of the current trajectory (if any)

    dynamic_planner_->plan(*msg, this->get_parameter("ee_name").as_string());
    dynamic_planner_->moveRobot();
}

void ManipulatorPlannerNode::jointGoal_callback(const sensor_msgs::msg::JointState::SharedPtr msg) 
{
    /*
    Callback function for the joint goal subscriber
    Args:
        msg: Joint goal positions
    */

    RCLCPP_INFO(this->get_logger(), "Received joint goal");

    dynamic_planner_->stop(); //Stop the execution of the current trajectory (if any)
    
    dynamic_planner_->plan(msg->position);
    dynamic_planner_->moveRobot();
}

void ManipulatorPlannerNode::collisionObject_callback(const moveit_msgs::msg::CollisionObject::SharedPtr collision_object) 
{
    // Add the collision object to the planning scene
    RCLCPP_INFO(this->get_logger(), "Received collision object: %s, operation: %d", collision_object->id.c_str(), collision_object->operation);
    dynamic_planner_->getPlanningScene()->applyCollisionObjects({*collision_object.get()});
}

void ManipulatorPlannerNode::attachedCollisionObject_callback(const moveit_msgs::msg::AttachedCollisionObject::SharedPtr collision_object) 
{
    // Add the collision object to the planning scene and attach it to a link
    // If link_name is empty, the 'ee_name' parameter is used
    RCLCPP_INFO(this->get_logger(), "Received attached collision object: %s, operation: %d", collision_object->object.id.c_str(), collision_object->object.operation);
    std::string link_name = collision_object->link_name.empty() ? this->get_parameter("ee_name").as_string() : collision_object->link_name;

    dynamic_planner_->getPlanningScene()->applyCollisionObjects({collision_object->object});
    dynamic_planner_->getMoveGroup()->attachObject(
        collision_object->object.id, 
        link_name, 
        this->get_parameter("gripper_links").as_string_array()
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
}

// Set the instantaneous inverse Kinematics
bool ManipulatorPlannerNode::instantKineSetter_callback(const std_srvs::srv::SetBool::Request::SharedPtr &req, std_srvs::srv::SetBool::Response::SharedPtr &res)
{
    // Set instantaneous kine param
    this->set_parameter(rclcpp::Parameter("inst_kine", req->data));

    RCLCPP_INFO(get_logger(), "Instantaneous kinematic mode set as %s", req->data ? "True":"False");
    // Stop the robot to prevent bad behaviours during mode switch
    arm_vel_cmd_[0] = 0.;
    arm_vel_cmd_[1] = 0.;
    arm_vel_cmd_[2] = 0.;
    arm_vel_cmd_[3] = 0.;
    arm_vel_cmd_[4] = 0.;
    arm_vel_cmd_[5] = 0.;
    // Return success
    res->success = true;
    res->message = req->data ? "Instantaneous kinematics control mode enabled":"Instantaneous kinematics control mode disabled";

    return true;
}

// Set the jacobian speed based control
bool ManipulatorPlannerNode::jointsRealTimeSetter_callback(const std_srvs::srv::SetBool::Request::SharedPtr &req, std_srvs::srv::SetBool::Response::SharedPtr &res)
{
    // Set robot real time joints speed control
    js_rt_control_ = req->data;
    RCLCPP_INFO(get_logger(), "Joints real time control mode set as %s", js_rt_control_ ? "True":"False");

    // Stop the robot to prevent bad behaviours during mode switch
    for (unsigned int k = 0; k < this->get_parameter("joint_names").as_string_array().size(); k++)
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
    return true;
}

// Set the jacobian speed based control
bool ManipulatorPlannerNode::jacobianControlSetter_callback(const std_srvs::srv::SetBool::Request::SharedPtr &req, std_srvs::srv::SetBool::Response::SharedPtr &res)
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
    return true;
}

// Update speed setpoint of the arm for the real time joints speed based control
void ManipulatorPlannerNode::realTimeSetpoint_callback(const sensor_msgs::msg::JointState::SharedPtr& msg)
{
    double max_spd_jnts = this->get_parameter("max_spd_jnts").as_double();
    for (unsigned int k = 0; k < get_parameter("joint_names").as_string_array().size(); k++)
    {
        // Update setpoint of the k-th joint
        js_msg_new_[k] = msg->velocity[k];
        // Check if the vel cmds exceed the maximum acceptable speed
        if (abs(msg->velocity[k]) > max_spd_jnts)
        {js_msg_new_[k] = sign(msg->velocity[k])*max_spd_jnts;}
    }
}

// Update the velocity setpoint of the arm for the jacobian speed based control
void ManipulatorPlannerNode::velJacSetpoint_callback(const geometry_msgs::msg::Twist::SharedPtr& msg)
{
    double max_speed_ee = this->get_parameter("max_speed_ee").as_double();
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
    if (norm_vel > max_speed_ee) {arm_msg_new_ *= (max_speed_ee/norm_vel);}
    // double norm_vel = arm_vel_cmd_.head<3>().norm();
    // if (norm_vel > max_speed_ee) {arm_vel_cmd_ *= (max_speed_ee/norm_vel);}
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

    int ros_freq = this->get_parameter("ros_freq").as_int();
    double max_accel_ee = this->get_parameter("max_accel_ee").as_double();
    std::vector<std::string> joint_names = this->get_parameter("joint_names").as_string_array();

    // Check if the acceleration is over the maximum, set a limitation
    if (abs(norm_msg-norm_vel)*ros_freq > max_accel_ee)
    {
        // Split the acceleration over the three axes
        double acc_x = max_accel_ee/3;
        double acc_y = max_accel_ee/3;
        double acc_z = max_accel_ee/3;
        if      (norm_msg > 0.001)  // norm_msg < -0.001 || 
        {
            acc_x = max_accel_ee*abs(arm_msg_new_(0))/norm_msg;
            acc_y = max_accel_ee*abs(arm_msg_new_(1))/norm_msg;
            acc_z = max_accel_ee*abs(arm_msg_new_(2))/norm_msg;
            // Update linear velocity components
            arm_vel_cmd_(0) = arm_vel_cmd_(0) + sign(arm_msg_new_(0)-arm_vel_cmd_(0))*acc_x/ros_freq;
            arm_vel_cmd_(1) = arm_vel_cmd_(1) + sign(arm_msg_new_(1)-arm_vel_cmd_(1))*acc_y/ros_freq;
            arm_vel_cmd_(2) = arm_vel_cmd_(2) + sign(arm_msg_new_(2)-arm_vel_cmd_(2))*acc_z/ros_freq;
        }
        else if (norm_vel > 0.001)  // norm_vel < -0.001 || 
        {
            acc_x = max_accel_ee*(abs(arm_vel_cmd_[0]))/norm_vel;
            acc_y = max_accel_ee*(abs(arm_vel_cmd_[1]))/norm_vel;
            acc_z = max_accel_ee*(abs(arm_vel_cmd_[2]))/norm_vel;
            // Update linear velocity components
            arm_vel_cmd_(0) = arm_vel_cmd_(0) + sign(arm_msg_new_(0)-arm_vel_cmd_(0))*acc_x/ros_freq;
            arm_vel_cmd_(1) = arm_vel_cmd_(1) + sign(arm_msg_new_(1)-arm_vel_cmd_(1))*acc_y/ros_freq;
            arm_vel_cmd_(2) = arm_vel_cmd_(2) + sign(arm_msg_new_(2)-arm_vel_cmd_(2))*acc_z/ros_freq;
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
    const unsigned int NUM_JOINTS = joint_names.size();
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
    qd = q + dq / ros_freq;

    // Build the msg for the joints setpoint
    sensor_msgs::msg::JointState js;
    js.name     = joint_names;
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
    std::vector<std::string> joint_names = this->get_parameter("joint_names").as_string_array();
    int ros_freq = this->get_parameter("ros_freq").as_int();
    double max_acc_jnts = this->get_parameter("max_acc_jnts").as_double();
    const unsigned int NUM_JOINTS = joint_names.size();

    // Check if the accelerations are acceptable and map the joints speed from the JointState message
    for (unsigned int k = 0; k < NUM_JOINTS; k++)
    {
        if (abs(js_msg_new_[k]-js_vel_cmd_[k])*ros_freq > max_acc_jnts)
            {js_vel_cmd_[k] = js_vel_cmd_[k] + sign(js_msg_new_[k]-js_vel_cmd_[k])*max_acc_jnts/ros_freq;}
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
    qd = q + js_vel_cmd_ / ros_freq;

    // Build the msg for the joints setpoint
    sensor_msgs::msg::JointState js;
    js.name     = joint_names;
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

void ManipulatorPlannerNode::set_instKine(bool inst_kine)
{
    // Set instantaneous kine param
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = inst_kine;
    instantKineSetter_client_->wait_for_service();
    auto result = instantKineSetter_client_->async_send_request(request);
}

//HELPER FUNCTIONS
void ManipulatorPlannerNode::declareParameters() {
    this->declare_parameter("manipulator_name", std::string());
    this->declare_parameter("planning_group", std::string());
    this->declare_parameter("joint_names", std::vector<std::string>());
    this->declare_parameter("ee_name", "tool_0");
    this->declare_parameter("base_link", "base_link");
    this->declare_parameter("sample_time", 0.002);
    this->declare_parameter("world_frame", "base_link");
    this->declare_parameter("vel_factor", 1.0);
    this->declare_parameter("acc_factor", 1.0);
    this->declare_parameter("ros_freq", 500);
    this->declare_parameter("max_speed_ee", 1.0);
    this->declare_parameter("max_accel_ee", 1.0);
    this->declare_parameter("max_spd_jnts", 1.0);
    this->declare_parameter("max_acc_jnts", 1.0);
    this->declare_parameter("inst_kine", true);
    this->declare_parameter("gripper_links", std::vector<std::string>()); //This is used to disable collision with the fingers when attaching objects
}
 
void ManipulatorPlannerNode::initializePlanner() {
    //Initialize the dynamic planner
    auto move_group_interface_node = std::make_shared<rclcpp::Node>("dynamic_planner_node", rclcpp::NodeOptions());
    executor_.add_node(move_group_interface_node);
    dynamic_planner_ = std::make_shared<DynamicPlanner>(move_group_interface_node,
                                                        this->get_parameter("planning_group").as_string(),
                                                        this->get_parameter("vel_factor").as_double(),
                                                        this->get_parameter("acc_factor").as_double(),
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