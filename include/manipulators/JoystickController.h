#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "manipulator_interfaces/msg/joint_goal.hpp"
#include <signal.h>
#include <functional>

#include "manipulators/ManipulatorMenu.h"

enum ControlMode {
    NONE = 0,
    JOINTS = 1,
    JACOBIAN = 2,
    ADMITTANCE = 3
};

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

static const rmw_qos_profile_t QOS_PROFILE_RELIABLE =
{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

static const rmw_qos_profile_t QOS_PROFILE_BEST_EFFORT =
{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

class JoystickController : public ManipulatorMenu
{
    public:
        JoystickController(ManipulatorMenuParams params, rclcpp::Node::SharedPtr node, const bool sync_parameters = false);

        void spinnerJoystick();

    protected:

        virtual void joyCallback(const sensor_msgs::msg::Joy::SharedPtr &joy); //Callback for joystick 
        void declareParameters(); 
        
        void joyDisableControl();              // Set control_mode_ to NONE, stop motion and disable all joystick controls
        void joySetJacobainControlMode();
        void joySetJointsControlMode();
        void joySetAdmittanceControlMode();
    
        //Shutdown handler
        void shutdown_handler();

        //Commands
        virtual void publishCmd();               //Publish velocity commands to manipulator

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;                    //Receive commands from joystick
        sensor_msgs::msg::Joy::SharedPtr joy_msg_; //Last received joystick message

        sensor_msgs::msg::JointState js_cmd_vel_;
        geometry_msgs::msg::Twist arm_cmd_vel_;

        double vel_step_, rot_step_, js_step_;
        std::string joy_topic_;
        
        ControlMode control_mode_;
        std::string control_frame_;

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
