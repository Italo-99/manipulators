#include "manipulators/JoystickController.h"

JoystickController::JoystickController(const std::string& node_name, ManipulatorMenuParams& params)
    : rclcpp::Node(node_name), params_(params)
{

    vel_step = 0.4;
    rot_step = 0.4;
    js_step = 1.0;

    js_cmd_vel_ = sensor_msgs::msg::JointState();
    js_cmd_vel_.name = params_.joint_names;
    js_cmd_vel_.velocity = std::vector<double>(params_.joint_names.size(), 0);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 1, 
        [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {
            joyCallback(msg);
        }
    );

    velJacSetpoint_pub_  = this->create_publisher<geometry_msgs::msg::Twist>(params_.manipulator_name + "/cmd_vel", 1);
    velJsRtSetpoint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(params_.manipulator_name + "/js_cmd_vel", 1);
}

void JoystickController::publishCmd(){
    velJacSetpoint_pub_->publish(arm_cmd_vel_);
    velJsRtSetpoint_pub_->publish(js_cmd_vel_);
}

void JoystickController::joyCallback(const sensor_msgs::msg::Joy::SharedPtr &joy){
    js_cmd_vel_.velocity = std::vector<double>(params_.joint_names.size(), 0);
    arm_cmd_vel_ = geometry_msgs::msg::Twist();


    // Linear velocity
    double x_axis = joy->axes[AxesMap::LEFTX];
    double y_axis = joy->axes[AxesMap::LEFTY];
    double z_axis = joy->axes[AxesMap::RIGHTY];

    bool rotation_control = joy->buttons[ButtonsMap::RIGHTSHOULDER];

    if(joy->buttons[ButtonsMap::LEFTSTICK] && !real_time_control_){
        jacobian_control_ = false;
        real_time_control_ = true;
        manipulator_menu_->setJacobianSpeedControl(false);
        manipulator_menu_->setJsRealTimeControl(true);
    } else if(joy->buttons[ButtonsMap::RIGHTSTICK] && !jacobian_control_){
        jacobian_control_ = true;
        real_time_control_ = false;
        manipulator_menu_->setJacobianSpeedControl(true);
        manipulator_menu_->setJsRealTimeControl(false);
    }

    if(jacobian_control_){
        if(rotation_control){
            arm_cmd_vel_.angular.x = x_axis * rot_step;
            arm_cmd_vel_.angular.y = y_axis * rot_step;
            arm_cmd_vel_.angular.z = z_axis * rot_step;
        } else {
            arm_cmd_vel_.linear.x = x_axis * vel_step;
            arm_cmd_vel_.linear.y = y_axis * vel_step;
            arm_cmd_vel_.linear.z = z_axis * vel_step;
        }
    } else if(real_time_control_){
        if(joy->axes[AxesMap::TRIGGERLEFT] < -0.3){          //Unlock j1 and j2
            js_cmd_vel_.velocity[0] = -y_axis * js_step;
            js_cmd_vel_.velocity[1] = -z_axis * js_step;
        }
        if(joy->axes[AxesMap::TRIGGERRIGHT] < -0.3) {        //Unlock j3 and j4
            js_cmd_vel_.velocity[2] = -x_axis * js_step;
            js_cmd_vel_.velocity[3] = -z_axis * js_step;
        }
        if(joy->buttons[ButtonsMap::LEFTSHOULDER]){         //Unlock j5 and j6
            js_cmd_vel_.velocity[4] = -x_axis * js_step;
            js_cmd_vel_.velocity[5] = -y_axis * js_step;
        }
    }

    if(joy->buttons[ButtonsMap::DPAD_UP]){
        manipulator_menu_->gripperMove(false);
    } else if(joy->buttons[ButtonsMap::DPAD_DOWN]){
        manipulator_menu_->gripperMove(true);
    }
}

void JoystickController::spinner(){
    rclcpp::Node::SharedPtr menu_node = std::make_shared<rclcpp::Node>(params_.node_name);
    manipulator_menu_ = std::make_shared<ManipulatorMenu>(params_, menu_node);

    rclcpp::Rate rate(params_.ros_freq);

    std::thread menu_thread = std::thread([this] {
        manipulator_menu_->spinner();
    });

    while(rclcpp::ok()){
        rclcpp::spin_some(this->shared_from_this());
        publishCmd();
        rate.sleep();
    }

    rclcpp::shutdown();
}