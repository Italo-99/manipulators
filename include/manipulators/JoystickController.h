#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "manipulator_interfaces/msg/joint_goal.hpp"
#include <signal.h>
#include <functional>

#include "manipulators/ManipulatorMenu.h"

enum AxesMap {
    LEFTY = 0,
    LEFTX = 1,
    RIGHTX = 2,
    RIGHTY = 3,
    TRIGGERLEFT = 4,
    TRIGGERRIGHT = 5
};

enum ButtonsMap {
    CROSS = 0,
    CIRCLE = 1,
    SQUARE = 2,
    TRIANGLE = 3,
    SELECT = 4,
    GUIDE = 5,
    START = 6,
    LEFTSTICK = 7,
    RIGHTSTICK = 8,
    LEFTSHOULDER = 9,
    RIGHTSHOULDER = 10,
    DPAD_UP = 11,
    DPAD_DOWN = 12,
    DPAD_LEFT = 13,
    DPAD_RIGHT = 14
};

class JoystickController : public ManipulatorMenu
{
    public:
        JoystickController(ManipulatorMenuParams params, rclcpp::Node::SharedPtr node, const bool sync_parameters = false);

        void spinnerJoystick();

    protected:

        virtual void joyCallback(const sensor_msgs::msg::Joy::SharedPtr &joy); //Callback for joystick 
        void declareParameters(); 
    
        //Shutdown handler
        void shutdown_handler();

        //Commands
        virtual void publishCmd();               //Publish velocity commands to manipulator

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;                    //Receive commands from joystick
        sensor_msgs::msg::Joy::SharedPtr joy_msg_; //Last received joystick message

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velJacSetpoint_pub_;        //Publish end effector velocity commands to manipulator
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr velJsRtSetpoint_pub_;    //Publish joint velocity commands to manipulator
        
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velAdmSetpoint_pub_;        //Publish velocity commands to admittance controller

        sensor_msgs::msg::JointState js_cmd_vel_;
        geometry_msgs::msg::Twist arm_cmd_vel_;

        double vel_step_, rot_step_, js_step_;
        std::string joy_topic_;

        bool jacobian_control_, real_time_control_, admittance_control_;

        rclcpp::TimerBase::SharedPtr cmd_pub_timer_;
};

class JoystickControllerFactory{
    public:
        static std::shared_ptr<JoystickController> fromProfile(
            const std::string& profile,
            ManipulatorMenuParams params, 
            rclcpp::Node::SharedPtr node, 
            const bool sync_parameters = false
        );
};

/*
    ===================================================================
    ======================== JOYSTICK PROFILES ========================
    ===================================================================



*/

class JoystickControllerPS3 : public JoystickController
{
    public:
        JoystickControllerPS3(ManipulatorMenuParams params, rclcpp::Node::SharedPtr node, const bool sync_parameters = false): 
            JoystickController(params, node, sync_parameters) {}

    protected:
        void joyCallback(const sensor_msgs::msg::Joy::SharedPtr &joy) override;
};
