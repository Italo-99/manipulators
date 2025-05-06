#ifndef MANIPULATOR_ROUTINES_H
#define MANIPULATOR_ROUTINES_H
#include "manipulators/ManipulatorMenu.h"

class ManipulatorRoutines : public ManipulatorMenu
{
public:
    ManipulatorRoutines(const rclcpp::Node::SharedPtr& node, ManipulatorMenuParams params);

    void routinesSpinnerMenu();

    void pickProbe_callback(); // Receive probe pose through tf2 listener, then execute pickUpRoutine

    /*
        PICKUP ROUTINE

            1. Plan end effector goal to obj_pose offset by pick_z_offset_ in z axis (positive up),
            if viable execute it, otherwise return false
            2. If use_gripper open the gripper
            3. Plan cartesian goal to reach object along z axis, if viable execute it, otherwise return false
            4. If use_gripper close the gripper
            5. Plan joint goal to destination
            6. If plan is viable execute it, otherwise return false
            7. If use_gripper open the gripper
    */
    bool pickUpRoutine(geometry_msgs::msg::Pose obj_pose, std::vector<double> destination, const bool use_gripper = false);

private:

    void initializeMenuRoutines(); // Initialize the menu
    MenuUserInterface<ManipulatorRoutines> *menu_;


    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr routineCmd_sub_; //Subscribe to routines execution commands

    double pick_z_offset_ = 0.2; //Pick routine will place the ee at this height above the probe
    bool is_executing_ = false;  //If a routine is being executed

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::TimerBase::SharedPtr routines_timer_; // Timer for the routines
};

template class MenuUserInterface<ManipulatorRoutines>;

#endif