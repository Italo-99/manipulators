#include "manipulators/ManipulatorRoutines.h"

ManipulatorRoutines::ManipulatorRoutines(const rclcpp::Node::SharedPtr& node, ManipulatorMenuParams params)
    : ManipulatorMenu(params, node, false)
{
    routineCmd_sub_ = node_->create_subscription<std_msgs::msg::Int8>(
        params_.manipulator_name + "/routine_cmd", 1,
        [this](const std_msgs::msg::Int8::SharedPtr msg) {
            // Fix the callback parameter type to match subscription type
        }
    );

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void ManipulatorRoutines::routinesSpinnerMenu()
{
    // Setup a rate for ROS loop execution
    rclcpp::Rate r(10);
    initializeMenuRoutines();

    std::thread spinner_thread = std::thread([this] {
        spinner();
    });

    // ROS loop
    while (rclcpp::ok())
    {
        // Display the user menu and process user choices
        menu_->printMenu();
        int choice = menu_->getUserChoice();
        RCLCPP_INFO(node_->get_logger(), "User choice: %d", choice);
        menu_->processChoice(choice);

        // Wait for next loop time
        r.sleep();
    }

    // Shutdown ROS if Ctrl+C or Ctrl+D are pressed
    rclcpp::shutdown();
}

void ManipulatorRoutines::pickProbe_callback()
{
    if(is_executing_)
    {
        RCLCPP_WARN(node_->get_logger(), "Pick probe command received while another routine is executing");
        return;
    }

    geometry_msgs::msg::TransformStamped transform;

    try {
        transform = tf_buffer_->lookupTransform(params_.base_link_name, "probe", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
          node_->get_logger(), "Could not transform 'probe' to %s: %s",
          params_.base_link_name.c_str(), ex.what());
        return;
    }

    geometry_msgs::msg::Pose probe_pose;
    probe_pose.position.x = transform.transform.translation.x;
    probe_pose.position.y = transform.transform.translation.y;
    probe_pose.position.z = transform.transform.translation.z;
    probe_pose.orientation = transform.transform.rotation;

    is_executing_ = true;
    bool success = pickUpRoutine(probe_pose, getKnownPose("home_gripper_down"), !(params_.gripper == "no_gripper"));
    is_executing_ = false;

    // if (executed_ && current_pose_.position.x != 0.0)
    // {
    //     return;
    // }
    // executed_ = true;

    // geometry_msgs::msg::Pose pick_pose_offset;

    // pick_pose_offset.position.x = current_pose_.position.x;
    // pick_pose_offset.position.y = current_pose_.position.y;
    // pick_pose_offset.position.z = current_pose_.position.z;

    // RCLCPP_INFO(node_->get_logger(), "Current pose: x: %f, y: %f, z: %f", pick_pose_offset.position.x, pick_pose_offset.position.y, pick_pose_offset.position.z);

    // pick_pose_offset.position.x += probe_pose->pose.position.x;
    // pick_pose_offset.position.y -= probe_pose->pose.position.y;
    // pick_pose_offset.position.z -= probe_pose->pose.position.z;

    // pick_pose_offset.position.z += pick_z_offset_;

    // pick_pose_offset.position.y += 0.0775; // Offset to the front
    // pick_pose_offset.position.z -= 0.024; // Offset to the top

    // pick_pose_offset.orientation = quaternion_from_euler(180, 0, 0);

    // std_msgs::msg::Bool pickProbeResult;

    // RCLCPP_INFO(node_->get_logger(), "Pick probe offset pose: x: %f, y: %f, z: %f", pick_pose_offset.position.x, pick_pose_offset.position.y, pick_pose_offset.position.z);

    // publishTcpGoal(pick_pose_offset);

    // manipulator_interfaces::msg::TrajectoryResult traj_result = planAndWait(pick_pose_offset, std::vector<double>(), "", 4U);
    // if (traj_result.success)
    // {
    //     RCLCPP_INFO(node_->get_logger(), "Pick probe offset trajectory planned successfully");
    //     bool execution_success = executeAndWait(traj_result.trajectory);
    //     if (execution_success)
    //     {
    //         pickProbeResult.data = true;
    //         RCLCPP_INFO(node_->get_logger(), "End effectot reached pick probe offset pose");
    //     }
    //     else
    //     {
    //         pickProbeResult.data = false;
    //         RCLCPP_ERROR(node_->get_logger(), "Pick probe offset trajectory execution failed");
    //     }
    // }
    // else
    // {
    //     RCLCPP_ERROR(node_->get_logger(), "Pick probe offset trajectory planning failed");
    //     pickProbeResult.data = false;
    // }
    // pickProbeResult_pub_->publish(pickProbeResult);
}

bool ManipulatorRoutines::pickUpRoutine(geometry_msgs::msg::Pose obj_pose, std::vector<double> destination, const bool use_gripper)
{
    // 1. Plan end effector goal to obj_pose offset by pick_z_offset_ in z axis (positive up),
    //    if viable execute it, otherwise return false

    geometry_msgs::msg::Pose pick_pose_offset = obj_pose;
    pick_pose_offset.position.z += pick_z_offset_;
    pick_pose_offset.orientation = quaternion_from_euler(180, 0, 90);

    manipulator_interfaces::msg::TrajectoryResult traj_result = planAndWait(pick_pose_offset, std::vector<double>(), "", 4U);

    if (traj_result.success)
    {
        RCLCPP_INFO(node_->get_logger(), "Pick probe offset trajectory planned successfully");
        bool execution_success = executeAndWait(traj_result.trajectory);
        if (execution_success)
        {
            RCLCPP_INFO(node_->get_logger(), "End effectot reached pick probe offset pose");
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Pick probe offset trajectory execution failed");
            return false;
        }
        moveGripper(false);
        //stocazzo
        move_along_z(-pick_z_offset_ + 0.1, true);
        rclcpp::sleep_for(std::chrono::milliseconds(800));
        moveGripper(true);
        rclcpp::sleep_for(std::chrono::milliseconds(300));
        publishJointGoal(getKnownPose("home_gripper_down"), std::vector<double>(), true);
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Pick probe offset trajectory planning failed");
        return false;
    }

    return true;
}

void ManipulatorRoutines::initializeMenuRoutines()
{
    menu_ = new MenuUserInterface<ManipulatorRoutines>(this);

    int section_start = 0; //Temporary variable to hold the last section start point
    menu_->addSection("Routines", section_start, section_start + 1);
    menu_->addChoice("Pick probe", &ManipulatorRoutines::pickProbe_callback);
}