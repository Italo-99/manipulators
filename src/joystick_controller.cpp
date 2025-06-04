#include "manipulators/JoystickController.h"

JoystickController::JoystickController(ManipulatorMenuParams params, rclcpp::Node::SharedPtr node, const bool sync_parameters)
    : ManipulatorMenu(params, node, sync_parameters), jacobian_control_(false), real_time_control_(false)
{
    declareParameters();

    params_ = params;

    vel_step_ = node_->get_parameter("vel_step").as_double();
    rot_step_ = node_->get_parameter("rot_step").as_double();
    js_step_ = node_->get_parameter("js_step").as_double();

    js_cmd_vel_ = sensor_msgs::msg::JointState();
    js_cmd_vel_.name = params_.joint_names;
    js_cmd_vel_.velocity = std::vector<double>(params_.joint_names.size(), 0);

    auto joy_sub_cb_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions joy_sub_options;
    joy_sub_options.callback_group = joy_sub_cb_group;

    joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 1, 
        [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {
            joyCallback(msg);
        },
        joy_sub_options
    );

    velJacSetpoint_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        params_.manipulator_name + "/cmd_vel", 1
    );
    velJsRtSetpoint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        params_.manipulator_name + "/js_cmd_vel", 1
    );

    rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback(
        std::bind(&JoystickController::shutdown_handler, this) // Register shutdown handler
    );
}


void JoystickController::joyCallback(const sensor_msgs::msg::Joy::SharedPtr &joy){
    js_cmd_vel_.velocity = std::vector<double>(params_.joint_names.size(), 0);
    arm_cmd_vel_ = geometry_msgs::msg::Twist();
    joy_msg_ = joy; // Store the last received joystick message

    if (joy->buttons[ButtonsMap::CROSS]) { // Cross button pressed
        RCLCPP_INFO(node_->get_logger(), "Going to home  position gripper facing down...");
        userGoHomeDown();
        return;
    } else if (joy->buttons[ButtonsMap::SQUARE]) { // Square button pressed
        RCLCPP_INFO(node_->get_logger(), "Going to home position gripper facing front...");
        userGoHomeFront();
        return;
    }


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

    // if(joy->buttons[ButtonsMap::DPAD_UP]){
    //     moveGripper(false);
    // } else if(joy->buttons[ButtonsMap::DPAD_DOWN]){
    //     moveGripper(true);
    // }
}

void JoystickController::declareParameters(){
    node_->declare_parameter("vel_step", 0.4);
    node_->declare_parameter("rot_step", 0.4);
    node_->declare_parameter("js_step", 1.0);
}

// Shutdown handler
void JoystickController::shutdown_handler()
{
    // Show the result of the jacobian control mean duration
    RCLCPP_INFO(node_->get_logger(), "Shutting down joystick controller node...");
}

// COMMANDS
void JoystickController::publishCmd()
{
    if      (jacobian_control_)  {velJacSetpoint_pub_->publish(arm_cmd_vel_);}
    else if (real_time_control_) {velJsRtSetpoint_pub_->publish(js_cmd_vel_);}
}

void JoystickController::spinnerJoystick(){
    cmd_pub_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(int(1000 / params_.ros_freq)),
        [this]() -> void {
            publishCmd();
        }
    );

    spinner();

    // Shutdown ROS
    rclcpp::shutdown();
}