#ifndef MANIPULATOR_PLANNER_H
#define MANIPULATOR_PLANNER_H

#include "manipulators/DynamicPlanner.h"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <Eigen/Geometry>
#include <vector>

//Interfaces
#include "shape_msgs/msg/solid_primitive.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float64.hpp"

#include "manipulator_interfaces/srv/f_kine.hpp"
#include "manipulator_interfaces/srv/inv_kine.hpp"
#include "manipulator_interfaces/srv/jacobian.hpp"
#include "manipulator_interfaces/srv/change_planner_parameters.hpp"
#include "manipulator_interfaces/srv/pseudo_inverse.hpp"

class ManipulatorPlannerNode : public rclcpp::Node {
    public:
        ManipulatorPlannerNode(
            const std::string node_name, 
            const rclcpp::NodeOptions &options
        );

        ~ManipulatorPlannerNode();

        void spinner(); //Spins the main node and executor_ in a different thread

    private:
        // --------------- COLLISION OBJECTS ---------------

        enum ShapeType {        //Shape type for shape_msgs::msg::SolidPrimitive
            BOX = 1,            //Sizes: x, y, z
            SPHERE = 2,         //Sizes: radius
            CYLINDER = 3,       //Sizes: height, radius
            CONE = 4            //Sizes: height, radius
        };

        void addCollisionObject( //UNUSED
            const std::string &object_name,
            const std::string &object_frame,
            const ShapeType object_type,
            const std::vector<double> &object_dims,
            const geometry_msgs::msg::Pose &object_pose
        );

        void addCollisionObject( //UNUSED
            const std::string &object_name,
            const std::string &object_frame,
            const shape_msgs::msg::SolidPrimitive &object_primitive,
            const geometry_msgs::msg::Pose &object_pose
        );

        void addAttachedCollisionObject( //UNUSED
            const std::string &object_name,
            const shape_msgs::msg::SolidPrimitive &object_primitive,
            const geometry_msgs::msg::Pose &object_pose,
            const std::string &link_name="",
            const std::vector<std::string> &disabled_collisions={}
        );

        void addAttachedCollisionObject( //UNUSED
            const std::string &object_name,
            const ShapeType object_type,
            const std::vector<double> &object_dims,
            const geometry_msgs::msg::Pose &object_pose,
            const std::string &link_name="",
            const std::vector<std::string> &disabled_collisions={}
        );

        // --------------- KINEMATICS FUNCTIONS ---------------

        const geometry_msgs::msg::Pose getFKine();
        const geometry_msgs::msg::Twist getTcpVel();

        const Eigen::MatrixXd getJacobian(const std::string &end_effector_link);
        const Eigen::MatrixXd getJacobian();
        
        const Eigen::MatrixXd getPseudoInverseJacobian(const std::string &end_effector_link);
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

        void changePlannerParams_callback(
            const manipulator_interfaces::srv::ChangePlannerParameters::Request::SharedPtr request,
            manipulator_interfaces::srv::ChangePlannerParameters::Response::SharedPtr response
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
        void tcpGoal_callback(const geometry_msgs::msg::Pose::SharedPtr goal_pose);
        void jointGoal_callback(const sensor_msgs::msg::JointState::SharedPtr goal_joints);
        void collisionObject_callback(const moveit_msgs::msg::CollisionObject::SharedPtr collision_object);
        void attachedCollisionObject_callback(const moveit_msgs::msg::AttachedCollisionObject::SharedPtr attached_collision_object);
        void cartesianPlan_callback(const geometry_msgs::msg::PoseArray::SharedPtr waypoints);
        void velJacSetpoint_callback(const geometry_msgs::msg::Twist::SharedPtr &msg); // Update the velocity setpoint of the arm for the jacobian speed based control
        void realTimeSetpoint_callback(const sensor_msgs::msg::JointState::SharedPtr &msg); // Update speed setpoint of the arm for the real time joints speed based control

        // --------------- CONTROL FUNCTIONS ---------------

        // Motors controller when no planner
        void motorsController(const sensor_msgs::msg::JointState &js);

        // Execute the jacobian based control
        void jacobianControl();

        // Execute the real time joints speed based control
        void jointsRealTimeControl();

        // --------------- HELPER FUNCTIONS ---------------

        void initializePlanner(); //Creates a node for the DynamicPlanner, adds it to the executor and initializes the planner_ object
        void declareParameters(); //Declare the parameters for the node
        void setPrimitiveDimensions(const ShapeType object_type, const std::vector<double> &object_dims, shape_msgs::msg::SolidPrimitive &primitive); //Set the dimensions of the primitive object
        double sign(double val); //Returns the sign of a number or 0 (+1.0 or -1.0)

        // --------------- VARIABLES INITIALIZATION ---------------

        std::string node_name_;

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
        double max_spd_jnts_;
        double max_acc_jnts_;
        std::vector<std::string> gripper_links_;

        //Services
        rclcpp::Service<manipulator_interfaces::srv::FKine>::SharedPtr fkine_service_;
        rclcpp::Service<manipulator_interfaces::srv::InvKine>::SharedPtr invkine_service_;
        rclcpp::Service<manipulator_interfaces::srv::Jacobian>::SharedPtr jacobian_service_;
        rclcpp::Service<manipulator_interfaces::srv::PseudoInverse>::SharedPtr pseudoInverse_service_;
        rclcpp::Service<manipulator_interfaces::srv::ChangePlannerParameters>::SharedPtr changePlannerParams_service_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr jointsRealTimeSetter_service_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr jacobianControlSetter_service_;

        //Subscribers
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr tcpGoal_sub_; //Subscriber for TCP goal (cartesian space goals) requests
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointGoal_sub_; //Subscriber for joint space goal requests
        rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr collisionObject_sub_; //Subscriber for addition and update of collision objects
        rclcpp::Subscription<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr attachedcollisionObject_sub_; //Subscriber for addition and update of collision objects
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cartesianPlan_sub_; //Subscriber for cartesian space waypoints
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velJacSetpoint_sub_; //Subscriber for the velocity setpoint for the jacobian speed based control
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr realTimeSetpoint_sub_; //Subscriber for the speed setpoint for the real time joints speed based control

        //Publishers
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j0_pub_;           // Publisher to j0 motor controller
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j1_pub_;           // Publisher to j1 motor controller
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j2_pub_;           // Publisher to j2 motor controller
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j3_pub_;           // Publisher to j3 motor controller
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j4_pub_;           // Publisher to j4 motor controller
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr j5_pub_;           // Publisher to j5 motor controller
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr tcpPose_pub_;    // Publisher to end effector pose
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr tcpVel_pub_;    // Publisher to end effector velocity

        std::shared_ptr<DynamicPlanner> dynamic_planner_; //Dynamic planner object
        rclcpp::TimerBase::SharedPtr dynamic_planner_timer_; //Timer for dynamic planner initialization
        rclcpp::executors::SingleThreadedExecutor executor_; //Executor for accessory nodes (e.g. dynamic planner)

        //Real time control variables
        bool   jac_control_ = false;            // True if the speed control through inverse Jacobian has been enabled
        bool js_rt_control_ = false;            // True if the speed control through direct real time joints cmd has been enabled
        Eigen::VectorXd arm_vel_cmd_; // Command of speed to the end_effector
        Eigen::VectorXd  js_vel_cmd_; // Command of speed to the joints
        Eigen::VectorXd arm_msg_new_; // New command of speed to the ee
        Eigen::VectorXd js_msg_new_;  // New command of joints speed

        double spinner_mean_ = 0.0; //Mean value for the time taken for each iteration of the spinner
};

#endif