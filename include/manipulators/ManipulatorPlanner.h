#include "manipulators/DynamicPlanner.h"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <Eigen/Geometry>
#include <vector>

//Interfaces
#include "shape_msgs/msg/solid_primitive.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "manipulator_interfaces/srv/f_kine.hpp"
#include "manipulator_interfaces/srv/inv_kine.hpp"
#include "manipulator_interfaces/srv/jacobian.hpp"
#include "manipulator_interfaces/srv/change_planner_parameters.hpp"
#include "manipulator_interfaces/srv/pseudo_inverse.hpp"
#include "manipulator_interfaces/srv/attached_collision_object.hpp"
#include "manipulator_interfaces/msg/collision_object.hpp"

class ManipulatorPlannerNode : public rclcpp::Node {
    public:
        ManipulatorPlannerNode(const std::string node_name, const rclcpp::NodeOptions &options);

        void spinner(); //Spins the main node and executor_ in a different thread

    private:
        // --------------- COLLISION OBJECTS ---------------

        enum ShapeType {        //Shape type for shape_msgs::msg::SolidPrimitive
            BOX = 1,            //Sizes: x, y, z
            SPHERE = 2,         //Sizes: radius
            CYLINDER = 3,       //Sizes: height, radius
            CONE = 4            //Sizes: height, radius
        };

        void addCollisionObject(
            const std::string &object_name,
            const std::string &object_frame,
            const ShapeType object_type,
            const std::vector<double> &object_dims,
            const geometry_msgs::msg::Pose &object_pose
        );

        void addCollisionObject(
            const std::string &object_name,
            const std::string &object_frame,
            const shape_msgs::msg::SolidPrimitive &object_primitive,
            const geometry_msgs::msg::Pose &object_pose
        );

        void addAttachedCollisionObject(
            const std::string &object_name,
            const shape_msgs::msg::SolidPrimitive &object_primitive,
            const geometry_msgs::msg::Pose &object_pose,
            const std::string &link_name="",
            const std::vector<std::string> &disabled_collisions={}
        );

        void addAttachedCollisionObject(
            const std::string &object_name,
            const ShapeType object_type,
            const std::vector<double> &object_dims,
            const geometry_msgs::msg::Pose &object_pose,
            const std::string &link_name="",
            const std::vector<std::string> &disabled_collisions={}
        );

        // --------------- KINEMATICS FUNCTIONS ---------------

        const Eigen::MatrixXd getJacobian(const std::string &end_effector_link);
        const Eigen::MatrixXd getJacobian();
        
        const Eigen::MatrixXd getPseudoInverseJacobian(const std::string &end_effector_link);
        const Eigen::MatrixXd getPseudoInverseJacobian();

        // --------------- CALLBACK FUNCTIONS ---------------
        
        //SERVICES
        void getFKine_callback(
            const std::shared_ptr<manipulator_interfaces::srv::FKine::Request> request, 
            std::shared_ptr<manipulator_interfaces::srv::FKine::Response> response
        );

        void getInvKine_callback(
            const std::shared_ptr<manipulator_interfaces::srv::InvKine::Request> request,
            std::shared_ptr<manipulator_interfaces::srv::InvKine::Response> response
        );

        void getJacobian_callback(
            const std::shared_ptr<manipulator_interfaces::srv::Jacobian::Request> request,
            std::shared_ptr<manipulator_interfaces::srv::Jacobian::Response> response
        );

        void getPseudoInverseJacobian_callback(
            const std::shared_ptr<manipulator_interfaces::srv::PseudoInverse::Request> request,
            std::shared_ptr<manipulator_interfaces::srv::PseudoInverse::Response> response
        );

        void changePlannerParams_callback(
            const std::shared_ptr<manipulator_interfaces::srv::ChangePlannerParameters::Request> request,
            std::shared_ptr<manipulator_interfaces::srv::ChangePlannerParameters::Response> response
        );

        void attachedCollisionObject_callback(
            const std::shared_ptr<manipulator_interfaces::srv::AttachedCollisionObject::Request> request,
            std::shared_ptr<manipulator_interfaces::srv::AttachedCollisionObject::Response> response
        );

        //SUBSCRIBERS
        void tcpGoal_callback(const geometry_msgs::msg::Pose::SharedPtr goal_pose);
        void jointGoal_callback(const sensor_msgs::msg::JointState::SharedPtr goal_joints);
        void collisionObject_callback(const manipulator_interfaces::msg::CollisionObject::SharedPtr collision_object);


        // --------------- HELPER FUNCTIONS ---------------

        void initializePlanner(); //Creates a node for the DynamicPlanner, adds it to the executor and initializes the planner_ object
        void declareParameters(); //Declare the parameters for the node
        void setPrimitiveDimensions(const ShapeType object_type, const std::vector<double> &object_dims, shape_msgs::msg::SolidPrimitive &primitive); //Set the dimensions of the primitive object


        // --------------- VARIABLES INITIALIZATION ---------------

        std::string node_name_;

        //Services
        rclcpp::Service<manipulator_interfaces::srv::FKine>::SharedPtr fkine_service_;
        rclcpp::Service<manipulator_interfaces::srv::InvKine>::SharedPtr invkine_service_;
        rclcpp::Service<manipulator_interfaces::srv::Jacobian>::SharedPtr jacobian_service_;
        rclcpp::Service<manipulator_interfaces::srv::PseudoInverse>::SharedPtr pseudoInverse_service_;
        rclcpp::Service<manipulator_interfaces::srv::ChangePlannerParameters>::SharedPtr changePlannerParams_service_;
        rclcpp::Service<manipulator_interfaces::srv::AttachedCollisionObject>::SharedPtr attachedCollisionObject_service_;

        //Subscribers
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr tcpGoal_sub_; //Subscriber for TCP goal (cartesian space goals) requests
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointGoal_sub_; //Subscriber for joint space goal requests
        rclcpp::Subscription<manipulator_interfaces::msg::CollisionObject>::SharedPtr collisionObject_sub_; //Subscriber for addition and update of collision objects


        std::shared_ptr<DynamicPlanner> dynamic_planner_; //Dynamic planner object
        rclcpp::TimerBase::SharedPtr dynamic_planner_timer_; //Timer for dynamic planner initialization
        rclcpp::executors::SingleThreadedExecutor executor_; //Executor for accessory nodes (e.g. dynamic planner)

        //Real time control variables
        Eigen::VectorXd joints_vel_cmd_; //Speed command for the joints
        Eigen::VectorXd ee_vel_cmd_; //Speed command for the end effector

        double spinner_mean_ = 0.0; //Mean value for the time taken for each iteration of the spinner
};