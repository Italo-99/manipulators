#ifndef MANIPULATOR_PLANNER_H
#define MANIPULATOR_PLANNER_H

#include "manipulators/DynamicPlanner.h"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <Eigen/Geometry>
#include <vector> 
#include <fcl/fcl.h>

//Interfaces
#include "shape_msgs/msg/solid_primitive.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "manipulator_interfaces/srv/f_kine.hpp"
#include "manipulator_interfaces/srv/inv_kine.hpp"
#include "manipulator_interfaces/srv/jacobian.hpp"
#include "manipulator_interfaces/srv/change_planner_tolerances.hpp"
#include "manipulator_interfaces/srv/change_planner_scaling_factors.hpp"
#include "manipulator_interfaces/srv/pseudo_inverse.hpp"
#include "manipulator_interfaces/srv/enable_real_time_constraints.hpp"
#include "manipulator_interfaces/msg/joint_goal.hpp"
#include "manipulator_interfaces/msg/tcp_goal.hpp"
#include "manipulator_interfaces/msg/cartesian_goal.hpp"

class ManipulatorPlannerNode : public rclcpp::Node {
    public:
        ManipulatorPlannerNode(
            const std::string node_name, 
            const rclcpp::NodeOptions &options
        );

        ~ManipulatorPlannerNode();

        void spinner(); //Spins the main node and executor_ in a different thread

    private:

        void shutdown_handler(); 

        // --------------- KINEMATICS FUNCTIONS ---------------

        /*!
            @brief Computes the current forward kinematics of the manipulator.
            @return The end effector pose as a geometry_msgs::msg::Pose
        */
        const geometry_msgs::msg::Pose getFKine();

        /*!
            @brief Computes the current end effector velocity.
            @return The end effector velocity as a geometry_msgs::msg::Twist
        */
        const geometry_msgs::msg::Twist getTcpVel();

        /*!
            @brief Computes the jacobian matrix for the passed joint state.
            @param joint_positions: Joint positions to compute the jacobian
            @param end_effector_link: Name of the end effector link to which the jacobian is referred
            @return The jacobian matrix as an Eigen::MatrixXd
        */
        const Eigen::MatrixXd getJacobian(const std::vector<double> &joint_positions, const std::string &end_effector_link);
        
        /*!
            @brief Computes the jacobian matrix for the current position.
            @param end_effector_link: Name of the end effector link to which the jacobian is referred
            @return The jacobian matrix as an Eigen::MatrixXd
        */
        const Eigen::MatrixXd getJacobian(const std::string &end_effector_link);

        /*!
            @brief Computes the jacobian matrix for the current position and the end effector set in the "ee_name" parameter.
            @return The jacobian matrix as an Eigen::MatrixXd
        */
        const Eigen::MatrixXd getJacobian();

        /*!
            @brief Computes the pseudo-inverse of the jacobian matrix for the passed joint state.
            @param joint_positions: Joint positions to compute the pseudo-inverse jacobian
            @param end_effector_link: Name of the end effector link to which the jacobian is referred
            @return The pseudo-inverse jacobian matrix as an Eigen::MatrixXd
        */
        const Eigen::MatrixXd getPseudoInverseJacobian(const std::vector<double> &joint_positions, const std::string &end_effector_link);

        /*!
            @brief Computes the pseudo-inverse of the jacobian matrix for the current position.
            @param end_effector_link: Name of the end effector link to which the jacobian is referred
            @return The pseudo-inverse jacobian matrix as an Eigen::MatrixXd
        */
        const Eigen::MatrixXd getPseudoInverseJacobian(const std::string &end_effector_link);

        /*!
            @brief Computes the pseudo-inverse of the jacobian matrix for the current position and the end effector set in the "ee_name" parameter.
            @return The pseudo-inverse jacobian matrix as an Eigen::MatrixXd
        */
        const Eigen::MatrixXd getPseudoInverseJacobian();

        // --------------- CALLBACK FUNCTIONS ---------------
        
        //SERVICES
        void getFKine_callback(
            const manipulator_interfaces::srv::FKine::Request::SharedPtr request, 
            manipulator_interfaces::srv::FKine::Response::SharedPtr response
        );

        void getInvKine_callback(
            const manipulator_interfaces::srv::InvKine::Request::SharedPtr request,
            manipulator_interfaces::srv::InvKine::Response::SharedPtr response
        );

        void getJacobian_callback(
            const manipulator_interfaces::srv::Jacobian::Request::SharedPtr request,
            manipulator_interfaces::srv::Jacobian::Response::SharedPtr response
        );

        void getPseudoInverseJacobian_callback(
            const manipulator_interfaces::srv::PseudoInverse::Request::SharedPtr request,
            manipulator_interfaces::srv::PseudoInverse::Response::SharedPtr response
        );

        void changePlannerScalingFactors_callback(
            const manipulator_interfaces::srv::ChangePlannerScalingFactors::Request::SharedPtr request,
            manipulator_interfaces::srv::ChangePlannerScalingFactors::Response::SharedPtr response
        );

        void changePlannerTolerances_callback(
            const manipulator_interfaces::srv::ChangePlannerTolerances::Request::SharedPtr request,
            manipulator_interfaces::srv::ChangePlannerTolerances::Response::SharedPtr response
        );

        void realTimeConstraintsSetter_callback(
            const manipulator_interfaces::srv::EnableRealTimeConstraints::Request::SharedPtr request,
            manipulator_interfaces::srv::EnableRealTimeConstraints::Response::SharedPtr response
        );

        void jointsRealTimeSetter_callback( // Set the real time joints speed based control
            const std_srvs::srv::SetBool::Request::SharedPtr req, 
            std_srvs::srv::SetBool::Response::SharedPtr res
        );    
        
        void jacobianControlSetter_callback( // Set the jacobian speed based control
            const std_srvs::srv::SetBool::Request::SharedPtr req, 
            std_srvs::srv::SetBool::Response::SharedPtr res
        );  

        //SUBSCRIBERS
        void tcpGoal_callback(const manipulator_interfaces::msg::TcpGoal::SharedPtr goal_pose);
        void jointGoal_callback(const manipulator_interfaces::msg::JointGoal::SharedPtr goal_joints);
        void cartesianPlan_callback(const manipulator_interfaces::msg::CartesianGoal::SharedPtr waypoints);
        void executionControl_callback(const std_msgs::msg::Bool::SharedPtr msg);
        void collisionObject_callback(const moveit_msgs::msg::CollisionObject::SharedPtr collision_object);
        void attachedCollisionObject_callback(const moveit_msgs::msg::AttachedCollisionObject::SharedPtr attached_collision_object);
        void velJacSetpoint_callback(const geometry_msgs::msg::Twist::SharedPtr &msg); // Update the velocity setpoint of the arm for the jacobian speed based control
        void realTimeSetpoint_callback(const sensor_msgs::msg::JointState::SharedPtr &msg); // Update speed setpoint of the arm for the real time joints speed based control
        void jointConstraint_callback(const moveit_msgs::msg::JointConstraint::SharedPtr joint_constraint); // Update joint constraints
        void positionConstraint_callback(const moveit_msgs::msg::PositionConstraint::SharedPtr position_constraint); // Update position constraints
        void orientationConstraint_callback(const moveit_msgs::msg::OrientationConstraint::SharedPtr orientation_constraint); // Update orientation constraints

        // --------------- CONTROL FUNCTIONS ---------------

        // Execute the jacobian based control
        void jacobianControl();
        void updateJacobianSpeedCmd(); //Fit ee_vel_cmd_ to the acceleration limits for the end effector and assign the value to current_ee_vel_
        void updateJacobianConstraintPrimitives(); //Update the list of constraint primitives for the jacobian control

        // Execute the real time joints speed based control
        void jointsRealTimeControl();

        // --------------- HELPER FUNCTIONS ---------------

        void addPrefix(const std::string& prefix, std::vector<std::string>& vector) const; //Add a prefix to all elements of a vector
        
        void initializePlanner(); //Creates a node for the DynamicPlanner, adds it to the executor and initializes the planner_ object
        void checkParams(); //Declare the parameters for the node
        double sign(double val); //Returns the sign of a number or 0 (+1.0 or -1.0)

        bool isPoseInsidePrimitive(
            const geometry_msgs::msg::Pose &pose, 
            const shape_msgs::msg::SolidPrimitive &primitive, 
            const geometry_msgs::msg::Pose &primitive_pose
        );

        // --------------- VARIABLES INITIALIZATION ---------------

        std::string node_name_;
        size_t NUM_JOINTS = 6;

        //Parameters
        std::string manipulator_name_;
        std::string planning_group_;
        std::vector<std::string> joint_names_;
        std::string ee_name_;
        std::string base_link_;
        std::string world_frame_;
        double ros_freq_;
        double max_speed_ee_;
        double max_accel_ee_;
        double max_rot_speed_ee_;
        double max_rot_accel_ee_;
        double max_spd_jnts_;
        double max_acc_jnts_;
        std::vector<std::string> gripper_links_;
        double min_jacobian_determinant_;
        Eigen::MatrixXd jacobian_var_;
        bool limit_joints_control_; // If true, the joints speed control is limited to the max_spd_jnts_ parameter
        bool limit_jacobian_control_; // If true, the jacobian speed control is limited to the max_speed_ee_ parameter
        double jac_sigma_threshold_;
        double jac_max_damping_factor_;

        //Services
        rclcpp::Service<manipulator_interfaces::srv::FKine>::SharedPtr fkine_service_;
        rclcpp::Service<manipulator_interfaces::srv::InvKine>::SharedPtr invkine_service_;
        rclcpp::Service<manipulator_interfaces::srv::Jacobian>::SharedPtr jacobian_service_;
        rclcpp::Service<manipulator_interfaces::srv::PseudoInverse>::SharedPtr pseudoInverse_service_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr jointsRealTimeSetter_service_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr jacobianControlSetter_service_;
        rclcpp::Service<manipulator_interfaces::srv::EnableRealTimeConstraints>::SharedPtr realTimeConstraintsSetter_service_;
        rclcpp::Service<manipulator_interfaces::srv::ChangePlannerTolerances>::SharedPtr changePlannerTolerances_service_;
        rclcpp::Service<manipulator_interfaces::srv::ChangePlannerScalingFactors>::SharedPtr changePlannerScalingFactors_service_;

        //Subscribers
        rclcpp::Subscription<manipulator_interfaces::msg::TcpGoal>::SharedPtr tcpGoal_sub_;                         //Subscriber for TCP goal (cartesian space goals) requests
        rclcpp::Subscription<manipulator_interfaces::msg::JointGoal>::SharedPtr jointGoal_sub_;                     //Subscriber for joint space goal requests
        rclcpp::Subscription<manipulator_interfaces::msg::CartesianGoal>::SharedPtr cartesianPlan_sub_;             //Subscriber for cartesian space waypoints
        rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr collisionObject_sub_;                    //Subscriber for addition and update of collision objects
        rclcpp::Subscription<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr attachedcollisionObject_sub_;    //Subscriber for addition and update of collision objects
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velJacSetpoint_sub_;                             //Subscriber for the velocity setpoint for the jacobian speed based control
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr realTimeSetpoint_sub_;                        //Subscriber for the speed setpoint for the real time joints speed based control
        rclcpp::Subscription<moveit_msgs::msg::PositionConstraint>::SharedPtr positionConstraint_sub_;              //Subscriber for position constraints
        rclcpp::Subscription<moveit_msgs::msg::OrientationConstraint>::SharedPtr orientationConstraint_sub_;        //Subscriber for orientation constraints
        rclcpp::Subscription<moveit_msgs::msg::JointConstraint>::SharedPtr jointConstraint_sub_;                    //Subscriber for joint constraints
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clearConstraints_sub_;                                //Subscriber for clearing constraints

        //Publishers
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr tcpPose_pub_;    // Publisher to end effector pose
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr tcpVel_pub_;    // Publisher to end effector velocity

        //Execution
        std::shared_ptr<DynamicPlanner> dynamic_planner_;       //Dynamic planner object
        
        rclcpp::executors::MultiThreadedExecutor executor_;     //Executor for accessory nodes (e.g. dynamic planner)
        double spinner_mean_ = 0.0;                             //Mean value for the time taken for each iteration of the spinner
        rclcpp::TimerBase::SharedPtr mainloop_timer_;           //Timer for the main loop
        rclcpp::TimerBase::SharedPtr tcpPose_timer_;            //Timer for publishing tcp pose
        rclcpp::TimerBase::SharedPtr tcpVel_timer_;             //Timer for publishing tcp velocity
        
        std::shared_ptr<rclcpp::CallbackGroup> services_cb_group_;       // Callback group for services
        std::shared_ptr<rclcpp::CallbackGroup> subscribers_cb_group_;    // Callback group for subscribers
        std::shared_ptr<rclcpp::CallbackGroup> main_cb_group_;           // Callback group for mainloop timer 

        //Real time control variables
        bool   jac_control_ = false;            // True if the speed control through inverse Jacobian has been enabled
        bool js_rt_control_ = false;            // True if the speed control through direct real time joints cmd has been enabled
        Eigen::VectorXd current_ee_vel_;        // Command of cartesian speed to ee
        Eigen::VectorXd current_js_vel_;        // Command of speed to the joints
        Eigen::VectorXd ee_vel_cmd_;            // New command of speed to the ee
        Eigen::VectorXd js_vel_cmd_;            // New command of joints speed
        std::vector<shape_msgs::msg::SolidPrimitive> constraints_primitives_; // List of contraints for jacobian control
        std::vector<geometry_msgs::msg::Pose> constraints_poses_; // List of poses for jacobian control
};

#endif
