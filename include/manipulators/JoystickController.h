#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "manipulator_interfaces/msg/joint_goal.hpp"
#include <signal.h>

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

class JoystickController : public rclcpp::Node
{
    public:
        JoystickController(const std::string& node_name);

        void spinner();

    protected:
        virtual void joyCallback(const sensor_msgs::msg::Joy::SharedPtr &joy); //Callback for joystick commands
        void declareParameters(); 
    
        //Shutdown handler
        void shutdown_handler();

        //Commands
        virtual void publishCmd();                        //Publish velocity commands to manipulator
        void setJacobianSpeedControl(const bool value);   //Set jacobian control
        void setJsRealTimeControl(const bool value);      //Set joints real time control
        void moveGripper(const bool closed);              //Move gripper
        void jointGoal(const std::vector<double>& goal);  //Move manipulator to a joint goal (angles in degrees)

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;                    //Receive commands from joystick

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velJacSetpoint_pub_;        //Publish end effector velocity commands to manipulator
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr velJsRtSetpoint_pub_;    //Publish joint velocity commands to manipulator

        sensor_msgs::msg::JointState js_cmd_vel_;
        geometry_msgs::msg::Twist arm_cmd_vel_;

        int ros_freq_;
        std::vector<std::string> joint_names_;
        std::string manipulator_name_;
        double vel_step_, rot_step_, js_step_;
        std::string gripper_group_;

        bool jacobian_control_, real_time_control_;

        int clients_wait_timeout_ = 5; 

        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr setJacobianControl_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr setJsRealTimeControl_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr moveGripper_client_;
        rclcpp::Publisher<manipulator_interfaces::msg::JointGoal>::SharedPtr jointGoal_pub_;

        rclcpp::executors::MultiThreadedExecutor executor_;
        rclcpp::TimerBase::SharedPtr mainloop_timer_;
};