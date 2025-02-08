#ifndef MANIPULATOR_MENU_H
#define MANIPULATOR_MENU_H

// IMPORT LIBRARIES
#include <iostream>
#include <cmath>
#include <unordered_map>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/display_robot_state.hpp"
#include "moveit/robot_state/conversions.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "std_srvs/srv/set_bool.hpp"

#include "motors_trajectory/srv/roboti_q_gripper_control.hpp"
#include "manipulator_interfaces/srv/coppelia_menu.hpp"
#include "manipulator_interfaces/srv/attached_collision_object.hpp"
#include "manipulator_interfaces/srv/inv_kine.hpp"
#include "manipulator_interfaces/srv/pseudo_inverse.hpp"
#include "manipulator_interfaces/srv/f_kine.hpp"
#include "manipulator_interfaces/srv/jacobian.hpp"
#include "manipulator_interfaces/srv/change_planner_parameters.hpp"

class ManipulatorMenu
{
 public:
  // ---------------------  PUBLIC CONSTRUCTOR ---------------------
    ManipulatorMenu(const rclcpp::Node::SharedPtr& node);

    sensor_msgs::msg::JointState current_joint_pose_;

  // ---------------------  PUBLIC FUNCTIONS ---------------------

    // Spinner
      void spinnerMenu(void);         // Asynchronous spinner for ROS routines with user menu
      void spinner(void);             // Update current robot joints state

    // Coppelia
      void startCoppeliaSim(void);        // Start simulation on CoppeliaSim
      void stopCoppeliaSim(void);         // Stop  simulation on CoppeliaSim
      void saveCoppeliaScene(void);       // Save  scene      on CoppeliaSim

    // Joint and TCP moves
      sensor_msgs::msg::JointState publishJointGoal(const std::vector<double> joints);  // publish a joint goal to the manipulator planner
      sensor_msgs::msg::JointState publishJointGoal(const sensor_msgs::msg::JointState jointStateMsg);
      geometry_msgs::msg::Pose     publishTcpGoal(const std::vector<double> position);  // publish a tcp   goal to the manipulator planner
      geometry_msgs::msg::Pose     publishTcpGoal(const geometry_msgs::msg::Pose tcpPoseMsg);
      geometry_msgs::msg::Pose     publishTcpIKGoal(const std::vector<double> position);// publish a tcpIK goal to the manipulator planner
      geometry_msgs::msg::Pose     publishTcpIKGoal(const geometry_msgs::msg::Pose tcpPoseMsg);
      geometry_msgs::msg::Pose     publishCartesianMove(const uint   axis1,  // publish a carthesian move command
                                              const uint   axis2,
                                              const double pos1,
                                              const double pos2,
                                              const uint   steps);
      sensor_msgs::msg::JointState oneJointMove(const int num, const double joint_rot); // to define a rotation around a single joint
      sensor_msgs::msg::JointState goHome(const bool);               // to setup home position

    // Get the position and orientation of the end effector (they contain a ros spin once)
      geometry_msgs::msg::Pose getEEpose();
      std::vector<double> getEEpos_rpy();

    // Get the transform between two frames
      geometry_msgs::msg::PoseStamped getTf(const std::string& source_frame, const std::string& target_frame);

    // Move along axes
      geometry_msgs::msg::Pose move_along_x(const double x_step,bool cartesian = false);
      geometry_msgs::msg::Pose move_along_y(const double y_step,bool cartesian = false);
      geometry_msgs::msg::Pose move_along_z(const double z_step,bool cartesian = false);

    // Tcp orientation handling
      geometry_msgs::msg::Pose make_tcp_rot(const std::vector<double> rot_vec);
      geometry_msgs::msg::Pose rotate_around_x(const double x_rot_step);
      geometry_msgs::msg::Pose rotate_around_y(const double y_rot_step);
      geometry_msgs::msg::Pose rotate_around_z(const double z_rot_step);
      geometry_msgs::msg::Pose change_tcp_orient(const std::vector<double> rot_vec);

    // Add collision objects
      void publishCollisionObject(const moveit_msgs::msg::CollisionObject collisionObjectMsg);
      void addObj(const std::string& name,
                const int            obj_type, 
                std::vector<double>  obj_dims, 
                double               obj_pos[], 
                double               rot_pos[],
                uint                 operation);

    // Add attached collision objects
      void publishAttachedCollisionObject(const moveit_msgs::msg::AttachedCollisionObject collisionAttachedObjectMsg);
      void addAttachedObj(const std::string&  name,
                          const int           obj_type, 
                          std::vector<double> obj_dims, 
                          double              obj_pos[], 
                          double              rot_pos[],
                          uint                operation);

    // Gripper control
      void openGripper(void);
      void closeGripper(void);
      void moveGripper(const double);
      void grabObjGripper(void);
      void detachObjGripper(void);
      void openRealGripper(void);
      void closeRealGripper(void);
      void moveRealGripper(const float);
      
    // Quaternions handling
      geometry_msgs::msg::Quaternion quaternion_from_euler(double roll, double pitch, double yaw);
      std::vector<double> euler_from_quaternion(const geometry_msgs::msg::Quaternion quat);

    // Degrees and radians conversions
      std::vector<double> deg_from_rad(const std::vector<double>);
      std::vector<double> rad_from_deg(const std::vector<double>);
    // Kinematics params getters
      geometry_msgs::msg::Pose getCurrentFKineClient(void);
      Eigen::MatrixXd     pseudoInverseClient(void);
      std::vector<double> invKineClient(const geometry_msgs::msg::Pose pose);
      Eigen::MatrixXd     getJacobianClient(void);
    // Kinematics params setters
      void setJacobianSpeedControl(bool);
      void setInstantKineMode(bool);
      void setNewPlannerParams(float,float);
      void setJsRealTimeControl(bool);

 private:

  // --------------------- PRIVATE FUNCTIONS ---------------------
    // ---------------  PRIVATE COPPELIA METHODS ---------------------
      void wait_for_response(void);     // Send the request and show the response

    // --------------------- PRIVATE PUBS/SUBS ---------------------

      void jointStateVisualizer();      // listen to joint state publisher

    // --------------------- MOVE FUNCTIONS ---------------------

      void testJointGoal(void);             // to test a joint goal
      void userJointGoal(void);             // to perform a joint goal set by the user 
      void oneJointMove_user();             // to move only a single joint

      void testTcpGoal(void);               // to test a tcp goal
      void userTcpGoal(void);               // to perform a tcp goal set by the user 
      void userTcpIKGoal(void);             // to perform a tcpIK goal set by the user 

      void userCartesianMove(void);         // to perform a cartesian move set by the user

      // Joint state callback function
      void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr& msg);

      void userGripperMove(void);          // to perform a gripper move set by the user
      void callGripperSrv(const bool);     // to call open/close gripper srv
      void callGrabbingSrv(const bool);    // to call grab/detach gripper srv
      void callRealGripperSrv(const float);// to call real gripper open close
    
    // --------------------- UTILS FUNCTIONS ---------------------
      // Enviornment updates functions
        void addCollObj(void);          // Add a collision object by the user
        void deleteCollObj(void);       // Delete a given collision object from the user menu
        void addUserAttachedObj(void);  // Add an attached collision object by the user

      // Menu handling
        void  printMenu();
        int   getUserChoice();
        void  processChoice(int choice);

        void declareParameters();       // Declare the parameters for the node

  // --------------------- PRIVATE VARIABLES ---------------------

    const rclcpp::Node::SharedPtr node_;

    // ---------------------  ROS HANDLING ---------------------
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointGoalPublisher_;   
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr tcpPosePublisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr tcpPoseIKPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr  carthesianMovePublisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  display_goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  eepose_pub_;
    rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr collisionObjectPublisher_;
    rclcpp::Publisher<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr  collisionAttObjectPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  moveGripperPublisher_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointStateSubscriber_;

    rclcpp::Client<manipulator_interfaces::srv::ChangePlannerParameters>::SharedPtr plannerParamsClient_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr setInstKineClient_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr setJacobianControlClient_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr setRealTimeControlClient_;

    // --------------------- ROBOT STATE ---------------------------
    geometry_msgs::msg::PoseStamped current_tcp_pose_;
    std::unordered_map<std::string, double> joints_map_group_;
    std::vector<double> joints_values_group_;

    // --------------------- Kinematics client init ---------------------
    rclcpp::Client<manipulator_interfaces::srv::InvKine>::SharedPtr invKineClient_;
    rclcpp::Client<manipulator_interfaces::srv::PseudoInverse>::SharedPtr pseudoInvClient_;
    rclcpp::Client<manipulator_interfaces::srv::FKine>::SharedPtr fKineClient_;
    rclcpp::Client<manipulator_interfaces::srv::Jacobian>::SharedPtr jacobianClient_;

    //   manipulators::InvKine       invKine_srv_;
    //   manipulators::PseudoInverse pseudoInv_srv_;
    //   manipulators::FKine         fKine_srv_;
    //   manipulators::Jacobian      jacobian_srv_;

    // ---------------------  COPPELIA HANDLING ---------------------
    rclcpp::Client<manipulator_interfaces::srv::CoppeliaMenu>::SharedPtr coppeliaClient_;
    manipulator_interfaces::srv::CoppeliaMenu::Request::SharedPtr coppelia_req_;

    // // ---------------------  GRIPPER HANDLING ---------------------
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr gripper_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr grab_client_;
    rclcpp::Client<motors_trajectory::srv::RobotiQGripperControl>::SharedPtr real_gripper_client_;

};

#endif /* MANIPULATOR_MENU_H */
