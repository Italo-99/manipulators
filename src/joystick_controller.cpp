#include "manipulators/JoystickController.h"


JoystickController* JoystickController::instance__ = nullptr;

JoystickController::JoystickController(const std::string& node_name)
    : rclcpp::Node(node_name), jacobian_control_(false), real_time_control_(false)
{
    declareParameters();

    ros_freq_ = this->get_parameter("ros_freq").as_int();
    joint_names_ = this->get_parameter("joint_names").as_string_array();
    manipulator_name_ = this->get_parameter("manipulator_name").as_string();
    vel_step_ = this->get_parameter("vel_step").as_double();
    rot_step_ = this->get_parameter("rot_step").as_double();
    js_step_ = this->get_parameter("js_step").as_double();
    gripper_group_ = this->get_parameter("gripper_group").as_string();

    if (joint_names_.size() == 0){
        RCLCPP_ERROR(this->get_logger(), "Joint names must be provided!");
        return;
    }

    js_cmd_vel_ = sensor_msgs::msg::JointState();
    js_cmd_vel_.name = joint_names_;
    js_cmd_vel_.velocity = std::vector<double>(joint_names_.size(), 0);

    auto joy_sub_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions joy_sub_options;
    joy_sub_options.callback_group = joy_sub_cb_group;

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 1, 
        [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {
            joyCallback(msg);
        },
        joy_sub_options
    );

    velJacSetpoint_pub_  = this->create_publisher<geometry_msgs::msg::Twist>(manipulator_name_ + "/cmd_vel", 1);
    velJsRtSetpoint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(manipulator_name_ + "/js_cmd_vel", 1);

    auto clients_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    setJacobianControl_client_ = this->create_client<std_srvs::srv::SetBool>(manipulator_name_ + "/jacobian_control_setter", rmw_qos_profile_services_default, clients_cb_group);
    setJsRealTimeControl_client_ = this->create_client<std_srvs::srv::SetBool>(manipulator_name_ + "/joints_real_time_setter", rmw_qos_profile_services_default, clients_cb_group);
    
    if (!gripper_group_.empty()){
        moveGripper_client_ = this->create_client<std_srvs::srv::SetBool>(manipulator_name_ + "/move_gripper", rmw_qos_profile_services_default, clients_cb_group);
    }
}

void JoystickController::joyCallback(const sensor_msgs::msg::Joy::SharedPtr &joy){
    js_cmd_vel_.velocity = std::vector<double>(joint_names_.size(), 0);
    arm_cmd_vel_ = geometry_msgs::msg::Twist();


    // Linear velocity
    double x_axis = joy->axes[AxesMap::LEFTX];
    double y_axis = joy->axes[AxesMap::LEFTY];
    double z_axis = joy->axes[AxesMap::RIGHTY];

    bool rotation_control = joy->buttons[ButtonsMap::RIGHTSHOULDER];

    if(joy->buttons[ButtonsMap::LEFTSTICK] && !real_time_control_){
        jacobian_control_ = false;
        real_time_control_ = true;
        setJacobianSpeedControl(false);
        setJsRealTimeControl(true);
    } else if(joy->buttons[ButtonsMap::RIGHTSTICK] && !jacobian_control_){
        jacobian_control_ = true;
        real_time_control_ = false;
        setJacobianSpeedControl(true);
        setJsRealTimeControl(false);
    }

    if(jacobian_control_){
        if(rotation_control){
            arm_cmd_vel_.angular.x = x_axis * rot_step_;
            arm_cmd_vel_.angular.y = y_axis * rot_step_;
            arm_cmd_vel_.angular.z = z_axis * rot_step_;
        } else {
            arm_cmd_vel_.linear.x = x_axis * vel_step_;
            arm_cmd_vel_.linear.y = y_axis * vel_step_;
            arm_cmd_vel_.linear.z = z_axis * vel_step_;
        }
    } else if(real_time_control_){
        if(joy->axes[AxesMap::TRIGGERLEFT] < -0.3){          //Unlock j1 and j2
            js_cmd_vel_.velocity[0] = -y_axis * js_step_;
            js_cmd_vel_.velocity[1] = -z_axis * js_step_;
        }
        if(joy->axes[AxesMap::TRIGGERRIGHT] < -0.3) {        //Unlock j3 and j4
            js_cmd_vel_.velocity[2] = -x_axis * js_step_;
            js_cmd_vel_.velocity[3] = -z_axis * js_step_;
        }
        if(joy->buttons[ButtonsMap::LEFTSHOULDER]){         //Unlock j5 and j6
            js_cmd_vel_.velocity[4] = -x_axis * js_step_;
            js_cmd_vel_.velocity[5] = -y_axis * js_step_;
        }
    }

    if(joy->buttons[ButtonsMap::DPAD_UP]){
        moveGripper(false);
    } else if(joy->buttons[ButtonsMap::DPAD_DOWN]){
        moveGripper(true);
    }
}

void JoystickController::declareParameters(){
    declare_parameter("ros_freq", 500);
    declare_parameter("joint_names", std::vector<std::string>());
    declare_parameter("manipulator_name", "manipulator");
    declare_parameter("vel_step", 0.4);
    declare_parameter("rot_step", 0.4);
    declare_parameter("js_step", 1.0);
    declare_parameter("gripper_group", std::string()); //Leave empty if no gripper
}

// SHUTDOWN HANDLER

void JoystickController::static_shutdown_handler(int sig)
{
    //Very unelegant way to call a non-static shutdown handler before the context is destroyed, but it works
    sig++; //Suppress unused var warning
    instance__->shutdown_handler();
}

// Shutdown handler
void JoystickController::shutdown_handler()
{
    // Show the result of the jacobian control mean duration
    RCLCPP_INFO(get_logger(), "Shutting down joystick controller node...");
    
    // Shutdown ROS
    rclcpp::shutdown();
}

// COMMANDS
void JoystickController::publishCmd()
{
    if      (jacobian_control_)  {velJacSetpoint_pub_->publish(arm_cmd_vel_);}
    else if (real_time_control_) {velJsRtSetpoint_pub_->publish(js_cmd_vel_);}
}

// Set Jacobian-based speed control
void JoystickController::setJacobianSpeedControl(const bool set)
{
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = set;

    // Wait for the service to be available
    while (!setJacobianControl_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()){
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "jacobian_control_setter service not available, waiting again...");
    }

    // Send the request asynchronously
    auto response_future = setJacobianControl_client_->async_send_request(request);

    // std::future_status status = response_future.wait_for(std::chrono::seconds(clients_wait_timeout_));
    // if(status != std::future_status::ready){
    //     RCLCPP_ERROR(this->get_logger(), "Service call failed. status: %d", status);
    // }
}

// Set Joints real time speed control
void JoystickController::setJsRealTimeControl(const bool set)
{
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = set;

    // Wait for the service to be available
    while (!setJsRealTimeControl_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()){
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "joints_real_time_setter service not available, waiting again...");
    }

    // Send the request asynchronously
    auto response_future = setJsRealTimeControl_client_->async_send_request(request);

    // std::future_status status = response_future.wait_for(std::chrono::seconds(clients_wait_timeout_));
    // if(status != std::future_status::ready){
    //     RCLCPP_ERROR(this->get_logger(), "Service call failed. status: %d", status);
    // }
}

void JoystickController::moveGripper(const bool close){

    if(gripper_group_.empty()){
        RCLCPP_ERROR(this->get_logger(), "Gripper is not available in this manipulator.");
        return;
    }

    std_srvs::srv::SetBool::Request::SharedPtr request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = close;

    // Wait for the service to be available
    while (!moveGripper_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(this->get_logger(), "gripperMove service not available, waiting again...");
    }

    // Send the request asynchronously and get the response
    auto response_future = moveGripper_client_->async_send_request(request);
}

void JoystickController::spinner(){
    rclcpp::Rate rate(ros_freq_);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(this->shared_from_this());

    signal(SIGINT, JoystickController::static_shutdown_handler);

    while(rclcpp::ok()){
        publishCmd();
        executor.spin_some();
        rate.sleep();
    }
}