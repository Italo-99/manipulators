#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "manipulator_interfaces/msg/joint_goal.hpp"
#include <signal.h>

#include "manipulators/ManipulatorMenu.h"

enum AxesMap {
    LEFTX = 0,
    LEFTY = 1,
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

        // Loads the mapping for joystick commands from the YAML file specified in the 'mapping_file' parameter
        void loadMapping();

        //Evaluate the event (eg: move joint 4, move gripper, etc.) 
        /// Returns the value of the axis (if above 'threshold'), capped to 'max' and multiplied by 'scale'
        double evaluateAxis(std::string category, std::string event);
        /// Returns true if the button is pressed, false otherwise (if an axis is provided it will need to be above 'threshold')
        bool evaluateButton(std::string category, std::string event);

        virtual void joyCallback(const sensor_msgs::msg::Joy::SharedPtr &joy); //Callback for joystick commands
        void declareParameters(); 
    
        //Shutdown handler
        void shutdown_handler();

        //Commands
        void publishCmd();                        //Publish velocity commands to manipulator

        YAML::Node mapping_;                     //Mapping of joystick commands

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;                    //Receive commands from joystick
        sensor_msgs::msg::Joy::SharedPtr joy_msg_; //Last received joystick message

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velJacSetpoint_pub_;        //Publish end effector velocity commands to manipulator
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr velJsRtSetpoint_pub_;    //Publish joint velocity commands to manipulator

        sensor_msgs::msg::JointState js_cmd_vel_;
        geometry_msgs::msg::Twist arm_cmd_vel_;

        double vel_step_, rot_step_, js_step_;
        std::string gripper_group_;

        bool jacobian_control_, real_time_control_;

        int clients_wait_timeout_ = 5; 

        rclcpp::TimerBase::SharedPtr cmd_pub_timer_;
};