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

    const std::string manipulator_name = this->get_parameter("manipulator_name").as_string();
    joints_vel_cmd_ = Eigen::VectorXd::Zero(NUM_JOINTS);
    ee_vel_cmd_ = Eigen::VectorXd::Zero(NUM_JOINTS);

    //Initialize service servers
    fkine_service_ = this->create_service<manipulator_interfaces::srv::FKine>(
        manipulator_name + "/get_fkine", 
        std::bind(&ManipulatorPlannerNode::getFKine_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    invkine_service_ = this->create_service<manipulator_interfaces::srv::InvKine>(
        manipulator_name + "/get_invkine", 
        std::bind(&ManipulatorPlannerNode::getInvKine_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    jacobian_service_ = this->create_service<manipulator_interfaces::srv::Jacobian>(
        manipulator_name + "/get_jacobian", 
        std::bind(&ManipulatorPlannerNode::getJacobian_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    pseudoInverse_service_ = this->create_service<manipulator_interfaces::srv::PseudoInverse>(
        manipulator_name + "/get_pseudo_inverse", 
        std::bind(&ManipulatorPlannerNode::getPseudoInverseJacobian_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    changePlannerParams_service_ = this->create_service<manipulator_interfaces::srv::ChangePlannerParameters>(
        manipulator_name + "/change_planner_params",
        std::bind(&ManipulatorPlannerNode::changePlannerParams_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    attachedCollisionObject_service_ = this->create_service<manipulator_interfaces::srv::AttachedCollisionObject>(
        manipulator_name + "/attached_collision_object",
        std::bind(&ManipulatorPlannerNode::attachedCollisionObject_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    //Initialize subscribers
    tcpGoal_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
        manipulator_name + "/tcp_goal", 5, 
        std::bind(&ManipulatorPlannerNode::tcpGoal_callback, this, std::placeholders::_1)
    );

    jointGoal_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        manipulator_name + "/joint_goal", 5, 
        std::bind(&ManipulatorPlannerNode::jointGoal_callback, this, std::placeholders::_1)
    );

    collisionObject_sub_ = this->create_subscription<manipulator_interfaces::msg::CollisionObject>(
        manipulator_name + "/collision_object", 5, 
        std::bind(&ManipulatorPlannerNode::collisionObject_callback, this, std::placeholders::_1)
    );
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
    RCLCPP_INFO(this->get_logger(), "Planner parameters changed successfully (vel_factor: %f, acc_factor: %f)", params.vel_factor, params.acc_factor);
}

void ManipulatorPlannerNode::attachedCollisionObject_callback(
    const std::shared_ptr<manipulator_interfaces::srv::AttachedCollisionObject::Request> request,
    std::shared_ptr<manipulator_interfaces::srv::AttachedCollisionObject::Response> response
) {
    /*
    Callback function for the attached collision object service
    Interface:
        request: 
            string object_name: Name of the object
            shape_msgs::msg::SolidPrimitive object_primitive: Object primitive (shape)
            geometry_msgs::msg::Pose object_pose: Position of the object
            string link_name: Name of the link to attach the object (if left empty, the 'ee_name' parameter is used)
        response: 
            bool success: True if the object was added successfully
    */

    std::string link_name = request->link_name.empty() ? this->get_parameter("ee_name").as_string() : request->link_name;
    addAttachedCollisionObject(
        request->object_name,
        request->primitive,
        request->pose,
        link_name,
        this->get_parameter("gripper_links").as_string_array()
    );
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

    dynamic_planner_->stop(); //Stop the execution of the current trajectory (if any)
    
    dynamic_planner_->plan(msg->position);
    dynamic_planner_->moveRobot();
}

void ManipulatorPlannerNode::collisionObject_callback(const manipulator_interfaces::msg::CollisionObject::SharedPtr collision_object) 
{
    /*
    Callback function for the collision object subscriber
    Interface:: 
        string object_name: Name of the object
        shape_msgs::msg::SolidPrimitive object_primitive: Object primitive (shape)
        geometry_msgs::msg::Pose object_pose: Position of the object
    */
    addCollisionObject(
        collision_object->object_name,
        this->get_parameter("world_frame").as_string(),
        collision_object->primitive,
        collision_object->pose
    );
}

//HELPER FUNCTIONS
void ManipulatorPlannerNode::declareParameters() {
    this->declare_parameter("manipulator_name", std::string());
    this->declare_parameter("planning_group", std::string());
    this->declare_parameter("joint_names", std::vector<std::string>());
    this->declare_parameter("ee_name", "tool_0");
    this->declare_parameter("base_link", "base_link");
    this->declare_parameter("sample_time", 0.002);
    this->declare_parameter("world_frame", "world");
    this->declare_parameter("vel_factor", 1.0);
    this->declare_parameter("acc_factor", 1.0);
    this->declare_parameter("ros_freq", 500);
    this->declare_parameter("max_speed_ee", 1.0);
    this->declare_parameter("max_accel_ee", 1.0);
    this->declare_parameter("max_spd_jnts", 1.0);
    this->declare_parameter("max_acc_jnts", 1.0);
    this->declare_parameter("gripper_links", std::vector<std::string>()); //This is used to disable collision with the fingers when attaching objects
}
 
void ManipulatorPlannerNode::initializePlanner() {
    //Initialize the dynamic planner
    auto move_group_interface_node = std::make_shared<rclcpp::Node>("dynamic_planner_node");
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