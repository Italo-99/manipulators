#ifndef JOYSTICK_CONTROLLER_H
#define JOYSTICK_CONTROLLER_H

#include <iostream>
#include <map>
#include <signal.h>
#include <string>
#include <vector>
#include <manipulators/ManipulatorMenu.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int8.h>

class JoystickController
{
    public:
        
        // Constructor
            JoystickController(const std::string& node_name);

        // Spinner
            void spinner();
    private:
        
        // ---------------------- FUNCTIONS ------------------------- //

            // Shutdown handler
                static void shutdown_handler(int sig);

            // Joy handlers
                void mapController(const std::string& controller);
                void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
                void move_Joystick();

        // ---------------------- VARIABLES ------------------------- //

            // Arguments and parameters
                std::vector<std::string>  joint_name_;
                std::string               group_name_;
                std::string               base_name_;
                std::string               controller_;
                double                    rot_step_;
                double                    lin_step_;
                std::vector<double>       joint_home_;

            // Joy mapping
                std::map<std::string, int>  buttonMapping_;
                std::map<std::string, int>  axesMapping_;
                std::vector<double>         butt_des_;

            // Control variables
                bool                        mode_speed_;
                sensor_msgs::JointState     js_cmd_vel_;
                geometry_msgs::Twist        arm_cmd_vel_;

            // Manipulator Menu object
                ManipulatorMenu* manipulator_;

            // ROS handlers
                ros::NodeHandle nh_;
                ros::Subscriber joy_sub_;
                ros::Publisher  jts_pub_;
                ros::Publisher  vel_pub_;
};

#endif // JOYSTICK_CONTROLLER_H