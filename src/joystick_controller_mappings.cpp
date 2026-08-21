#include "manipulators/JoystickController.h"

// FACTORY

std::shared_ptr<JoystickController> JoystickControllerFactory::fromProfile(
    const std::string& profile,
    ManipulatorMenuParams params, 
    rclcpp::Node::SharedPtr node, 
    const bool sync_parameters)
{
    // Convert profile string to lowercase for case-insensitive comparison
    std::string profileLower = profile;
    std::transform(profileLower.begin(), profileLower.end(), profileLower.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    
    // Define a type for controller factory functions
    using ControllerFactory = std::function<std::shared_ptr<JoystickController>(
        ManipulatorMenuParams, rclcpp::Node::SharedPtr, bool)>;
    
    // PROFILES MAP
    static const std::unordered_map<std::string, ControllerFactory> profileFactories = {
        {"ps3", [](ManipulatorMenuParams p, rclcpp::Node::SharedPtr n, bool s) {
            return std::make_shared<JoystickControllerPS3>(p, n, s);
        }}
        
        // Add more profiles here as they are implemented
        // {"xbox", [](ManipulatorMenuParams p, rclcpp::Node::SharedPtr n, bool s) {
        //     return std::make_shared<JoystickControllerXbox>(p, n, s);
        // }}
    };
    
    // Look up the factory function in the map
    if (profileLower == "default") {
        RCLCPP_INFO(node->get_logger(), "Creating default controller profile");
        return std::make_shared<JoystickController>(params, node, sync_parameters);
    }
    
    auto factoryIt = profileFactories.find(profileLower);

    if (factoryIt != profileFactories.end()) {
        RCLCPP_INFO(node->get_logger(), "Creating %s controller profile", profile.c_str());
        return factoryIt->second(params, node, sync_parameters);
    }
    
    // Default to base implementation if profile not found
    RCLCPP_WARN(node->get_logger(), "Profile '%s' not found. Using default controller profile", 
                profile.c_str());
    return std::make_shared<JoystickController>(params, node, sync_parameters);
}

// MAPPINGS

void JoystickControllerPS3::joyCallback(const sensor_msgs::msg::Joy::SharedPtr &joy)
{
    js_cmd_vel_.velocity = std::vector<double>(params_.joint_names.size(), 0);
    arm_cmd_vel_ = geometry_msgs::msg::Twist();
    joy_msg_ = joy; // Store the last received joystick message

    if (joy->buttons[0]) { // Cross button pressed
        RCLCPP_INFO(node_->get_logger(), "Going to home  position gripper facing down...");
        userGoHomeDown();
        return;
    } else if (joy->buttons[3]) { // Square button pressed
        RCLCPP_INFO(node_->get_logger(), "Going to home position gripper facing front...");
        userGoHomeFront();
        return;
    }

    // Linear velocity
    double x_axis = joy->axes[0]; // LEFTX
    double y_axis = joy->axes[1]; // LEFTY
    double z_axis = joy->axes[4]; // RIGHTY

    bool rotation_control = joy->buttons[5]; //R1

    if(joy->buttons[11] && control_mode_ != ControlMode::JOINTS){ //L3 
        joySetJointsControlMode();
    } else if(joy->buttons[12] && control_mode_ != ControlMode::JACOBIAN){ //R3
        joySetJacobainControlMode();
    }

    if(control_mode_ == ControlMode::JACOBIAN){
        if(rotation_control){
            arm_cmd_vel_.angular.x = x_axis * rot_step_;
            arm_cmd_vel_.angular.y = y_axis * rot_step_;
            arm_cmd_vel_.angular.z = z_axis * rot_step_;
        } else {
            arm_cmd_vel_.linear.x = x_axis * vel_step_;
            arm_cmd_vel_.linear.y = y_axis * vel_step_;
            arm_cmd_vel_.linear.z = z_axis * vel_step_;
        }
    } else if(control_mode_ == ControlMode::JOINTS){
        if(joy->axes[2] > -0.5){          //Unlock j1 and j2 (L2)
            js_cmd_vel_.velocity[0] = -y_axis * js_step_;
            js_cmd_vel_.velocity[1] = -z_axis * js_step_;
        }
        if(joy->axes[5] > -0.5) {        //Unlock j3 and j4  (R2)
            js_cmd_vel_.velocity[2] = -x_axis * js_step_;
            js_cmd_vel_.velocity[3] = -z_axis * js_step_;
        }
        if(joy->buttons[4]){             //Unlock j5 and j6  (L1)
            js_cmd_vel_.velocity[4] = -x_axis * js_step_;
            js_cmd_vel_.velocity[5] = -y_axis * js_step_;
        }
    }

    if(joy->buttons[13]){ //DPAD UP
        moveGripper(false);
    } else if(joy->buttons[14]){ //DPAD DOWN
        moveGripper(true);
    }
}
