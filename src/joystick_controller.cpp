/* Software License Agreement (Apache Licence 2.0)
 *
 *  Copyright (c) [2024], [Andrea Pupa] [italo Almirante]
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *   3. The name of the author may not be used to endorse or promote
 *      products derived from this software without specific prior
 *      written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: [Italo Almirante]
 *  Created on: [2024-11-03]
*/

#include "manipulators/JoystickController.h"

// Class constructor
JoystickController::JoystickController(const std::string& node_name)
{
    // Get node parameters values
    std::string default_base_name       = "base_link";
    std::string default_group_name      = "manipulator";
    std::string default_controller_name = "Logitech";
    nh_.param(node_name+"/base_name",     base_name_,   default_base_name         );
    nh_.param(node_name+"/group_name",    group_name_,  default_group_name        );
    nh_.param(node_name+"/controller",    controller_,  default_controller_name   );
    nh_.param(node_name+"/lin_step",      lin_step_,    0.01                      );
    nh_.param(node_name+"/rot_step",      rot_step_,    0.08                      );
    nh_.param(node_name+"/joint_home",    joint_home_, {0.,-90.,-90.,-90.,+90.,0.});
    nh_.param(node_name+"/joint_name",    joint_name_, {"shoulder_pan_joint",   "shoulder_lift_joint",  "elbow_joint",
                                                    "wrist_1_joint",        "wrist_2_joint",        "wrist_3_joint"}             );

    ROS_INFO("Linear  speed step set to: %f", lin_step_);
    ROS_INFO("Angular speed step set to: %f", rot_step_);

    // Declaration of manipulator menu
    ManipulatorMenuParams     params;
    params.manipulator_name = group_name_;
    params.joint_names      = joint_name_;
    params.base_link_name   = base_name_;
    manipulator_            = new ManipulatorMenu(params);

    // Initialize the subscriber to /joy topic
    joy_sub_ = nh_.subscribe("/joy_arm", 1, &JoystickController::joyCallback, this);

    // Publish twist cmd vel to arm
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>(   group_name_+"/cmd_vel",   1);
    jts_pub_ = nh_.advertise<sensor_msgs::JointState>(group_name_+"/js_cmd_vel",1);

    // Joystick controller mode
    mode_speed_ = false;        // False to control the EE, true to control the joints

    //Initialize the JointStates vector
    js_cmd_vel_.name     = joint_name_;
    js_cmd_vel_.velocity.resize(joint_name_.size());
    js_cmd_vel_.velocity = {0.,0.,0.,0.,0.,0.};

    //Initialize the Twist vector
    arm_cmd_vel_.linear.x   = 0.;
    arm_cmd_vel_.linear.y   = 0.;
    arm_cmd_vel_.linear.z   = 0.;
    arm_cmd_vel_.angular.x  = 0.;
    arm_cmd_vel_.angular.y  = 0.;
    arm_cmd_vel_.angular.z  = 0.;
    
    //Declaration of roboticController
    ROS_INFO("Controller: %s", controller_.c_str());

    // Map joystick commands
    mapController(controller_);

    //Define the buttons
    butt_des_ = {0.,0.,0.,0.,0.,0.};
}

// Initialize
void JoystickController::mapController(const std::string& controller)
{
    //Controller PS4 params
    if (controller == "PS4")
    {
        buttonMapping_ =
                        {
                            {"cross",           0},
                            {"circle",          1},
                            {"triangle",        2},
                            {"square",          3},
                            {"button_L1",       4},
                            {"button_L2",       6},
                            {"button_R1",       5},
                            {"button_R2",       7},
                            {"analog_button_sx",11}
                        };

        axesMapping_ = 
                        {
                            {"axis_x",  1},
                            {"axis_y",  0}, 
                            {"axis_z",  4}, 
                            {"arrow_up",7}
                        };
    }

    //Controller Logitech params
    if (controller == "Logitech")
    {
        buttonMapping_ =
                        {
                            {"square",          0},
                            {"cross",           1},
                            {"circle",          2},
                            {"triangle",        3},
                            {"button_L1",       4},
                            {"button_R1",       5},
                            {"button_L2",       6},
                            {"button_R2",       7},
                            {"analog_button_sx",10}
                        };

        axesMapping_ =
                        {
                            {"axis_x",  1},
                            {"axis_y",  0}, 
                            {"axis_z",  3}, 
                            {"arrow_up",5}
                        };
    }

    if (controller == "Android")
    {
        buttonMapping_ =
                        {
                            {"square",          3},
                            {"cross",           0},
                            {"circle",          1},
                            {"triangle",        4},
                            {"button_L1",       6},
                            {"button_R1",       7},
                            {"button_L2",       8},
                            {"button_R2",       9},
                            {"analog_button_sx",13}
                        };

        axesMapping_ =
                        {
                            {"axis_x",  1},
                            {"axis_y",  0}, 
                            {"axis_z",  3}, 
                            {"arrow_up",7}
                        };
    }
}

// Shutdown handler
void JoystickController::shutdown_handler(int sig)
{
    // Shutdown ROS
    ros::shutdown();
}

// Joy Callback
void JoystickController::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    // Get buttons pressed
    butt_des_[0] = joy->buttons[buttonMapping_["cross"]];
    butt_des_[1] = joy->buttons[buttonMapping_["button_L2"]];
    butt_des_[2] = joy->buttons[buttonMapping_["button_R2"]];
    butt_des_[3] = joy->axes[axesMapping_["arrow_up"]];
    butt_des_[4] = joy->buttons[buttonMapping_["button_L1"]];
    butt_des_[5] = joy->buttons[buttonMapping_["button_R1"]];
    butt_des_[6] = joy->buttons[buttonMapping_["analog_button_sx"]];

    // PREDEFINED JOINT POSES
    if (butt_des_[0] == 1)  manipulator_->publishJointGoal(joint_home_);    // Go to home pose

    // ENABLE/DISABLE SPEED MODE
    if (butt_des_[6] == 1)                              // Set the manual teleop of the EE                         
    {
        mode_speed_ = !mode_speed_;
        if (!mode_speed_)
        {
            manipulator_->setJsRealTimeControl(false);  // Disable js cmd vel to joints
            manipulator_->setJacobianSpeedControl(true);// Enable     cmd vel to end-effector
        }
        else
        {
            manipulator_->setJsRealTimeControl(true);    // Enable js cmd vel to joints
            manipulator_->setJacobianSpeedControl(false);// Disable   cmd vel to end-effector
        }
        
    }

    // EE SPEED CONTROL
    if (butt_des_[5] == 1)  // Angular twist of the EE enabled 
    {
        arm_cmd_vel_.linear.x  = 0.;
        arm_cmd_vel_.linear.y  = 0.;
        arm_cmd_vel_.linear.z  = 0.;
        arm_cmd_vel_.angular.x = +joy->axes[axesMapping_["axis_y"]] * rot_step_;
        arm_cmd_vel_.angular.y = +joy->axes[axesMapping_["axis_x"]] * rot_step_;
        arm_cmd_vel_.angular.z = +joy->axes[axesMapping_["axis_z"]] * rot_step_;
    }
    else                    // Linear  twist of the EE enabled
    {
        arm_cmd_vel_.linear.x  = +joy->axes[axesMapping_["axis_x"]] * lin_step_;
        arm_cmd_vel_.linear.y  = +joy->axes[axesMapping_["axis_y"]] * lin_step_;
        arm_cmd_vel_.linear.z  = +joy->axes[axesMapping_["axis_z"]] * lin_step_;
        arm_cmd_vel_.angular.x = 0.;
        arm_cmd_vel_.angular.y = 0.;
        arm_cmd_vel_.angular.z = 0.;
    }
    
    // JOINTS SPEED CONTROL

    // Rotation of the joints 1 and 2
    if (butt_des_[1] == 1)
    {
        js_cmd_vel_.velocity[0] = -joy->axes[axesMapping_["axis_x"]] * rot_step_;
        js_cmd_vel_.velocity[1] = -joy->axes[axesMapping_["axis_z"]] * rot_step_;
    }

    // Rotation of the joints 3 and 4
    if (butt_des_[2] == 1)
    {
        js_cmd_vel_.velocity[2] = -joy->axes[axesMapping_["axis_x"]] * rot_step_;
        js_cmd_vel_.velocity[3] = -joy->axes[axesMapping_["axis_z"]] * rot_step_;
    }

    //  Rotation of the joints 5 and 6
    if (butt_des_[4] == 1)
    {
        js_cmd_vel_.velocity[4] = -joy->axes[axesMapping_["axis_x"]] * rot_step_;
        js_cmd_vel_.velocity[5] = -joy->axes[axesMapping_["axis_z"]] * rot_step_;
    }
}

// Joystick move
void JoystickController::move_Joystick()
{
    if (!mode_speed_)   vel_pub_.publish(arm_cmd_vel_); // EE     move
    else                jts_pub_.publish(js_cmd_vel_);  // Joints move
}

// ROS spinner
void JoystickController::spinner()
{
  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, shutdown_handler);

  // Setup a rate for ROS loop execution
  ros::Rate r(10.);

  // ROS loop
  while (ros::ok())
  {
    ros::spinOnce();
    move_Joystick();
    r.sleep();
  }

}

// MAIN FUNCTION: this is a node
int main(int argc, char** argv)
{
  // Init the node name
  std::string node_name = "joy_arm_controller";

  // Initialize node
  ros::init(argc, argv, node_name);

  // Istantiate an object of the class ManipulatorPlanner
  JoystickController j(node_name);

  j.spinner();

  // File end
  return 0;
}