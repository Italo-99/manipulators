#ifndef MANIPULATOR_MENU_H
#define MANIPULATOR_MENU_H

// IMPORT LIBRARIES
#include <iostream>
#include <cmath>
#include <Eigen/Geometry>
#include <unordered_map>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/display_robot_state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "std_srvs/srv/set_bool.hpp"

#include "motors_trajectory/srv/roboti_q_gripper_control.hpp"
#include "manipulator_interfaces/srv/coppelia_menu.hpp"
#include "manipulator_interfaces/srv/inv_kine.hpp"
#include "manipulator_interfaces/srv/pseudo_inverse.hpp"
#include "manipulator_interfaces/srv/f_kine.hpp"
#include "manipulator_interfaces/srv/jacobian.hpp"
#include "manipulator_interfaces/srv/change_planner_parameters.hpp"
#include "manipulator_interfaces/msg/joint_goal.hpp"
#include "manipulator_interfaces/msg/tcp_goal.hpp"
#include "manipulator_interfaces/msg/trajectory_result.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "manipulators/MenuUserInterface.h"

struct ManipulatorMenuParams
{
    std::string node_name         = "manipulator_menu_node";
    double ros_freq               = 500;
    std::string manipulator_name  = "manipulator";
    std::string planning_group    = "ur_manipulator";

    bool gripper                  = false;
    std::string gripper_group     = "robotiq_85_gripper";

    std::vector<std::string> joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
    std::string base_link_name    = "base_link";
};

class ManipulatorMenu
{
    //NOTE: In general when using the sensor_msgs/JointState type the angles will be expressed in radians, 
    //      instead when using a vector joint angles will be expressed in degrees.
    //      In a similar way when using vectors to represent poses, the first 3 elements will be the x, y 
    //      and z coordinates in meters while the last 3 elements will be the roll, pitch and yaw in degrees.
    public:
        // ---------------------  PUBLIC CONSTRUCTOR ---------------------
        ManipulatorMenu(
            ManipulatorMenuParams &params,
            const rclcpp::Node::SharedPtr& node
        );

        ~ManipulatorMenu();

        sensor_msgs::msg::JointState current_joint_pose_;

        // ---------------------  PUBLIC FUNCTIONS ---------------------

        // Spinner
        void spinnerMenu(void);         // Asynchronous spinner for ROS routines with user menu
        void spinner(void);             // Update current robot joints state

        // Joint and TCP moves
        //The following functions will plan and execute a trajectory, then return immediatly
        // publish a joint goal to the manipulator planner
        sensor_msgs::msg::JointState publishJointGoal(
            const std::vector<double> joint_goal, 
            const std::vector<double> start_state = std::vector<double>(), 
            const bool execute=true);

        sensor_msgs::msg::JointState publishJointGoal(
            const sensor_msgs::msg::JointState joint_goal, 
            const std::vector<double> start_state = std::vector<double>(), 
            const bool execute=true);

        // publish a tcp goal to the manipulator planner
        geometry_msgs::msg::Pose publishTcpGoal(
            const std::vector<double> position, 
            const std::vector<double> start_state = std::vector<double>(), 
            const bool execute=true);  

        geometry_msgs::msg::Pose publishTcpGoal(
            const geometry_msgs::msg::Pose tcpPoseMsg, 
            const std::vector<double> start_state = std::vector<double>(), 
            const bool execute=true);
            
        sensor_msgs::msg::JointState oneJointMove(const int num, const double joint_rot); // to execute rotation of a single joint
        sensor_msgs::msg::JointState goHome(const bool);                                  // to setup home position

        // Planning 

        //The following functions will plan a trajectory and return it (if timeout or error return an empty trajectory)
        //timeout arg is in seconds
        moveit_msgs::msg::RobotTrajectory planAndWait(const sensor_msgs::msg::JointState joint_goal, const std::vector<double> start_state = std::vector<double>(), uint timeout=2);
        moveit_msgs::msg::RobotTrajectory planAndWait(const geometry_msgs::msg::Pose tcp_goal, const std::vector<double> start_state = std::vector<double>(), uint timeout=2);

        //The following functions will execute a trajectory and return once it's finished
        bool executeAndWait(const moveit_msgs::msg::RobotTrajectory joint_trajectory, uint timeout=20);

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
        

        // Matrix utils
        void printMatrix(const Eigen::MatrixXd& matrix);
        void listToMatrix(const std::vector<double> &list, Eigen::MatrixXd &matrix);

        // Quaternions utils
        geometry_msgs::msg::Quaternion quaternion_from_euler(double roll, double pitch, double yaw);
        std::vector<double> euler_from_quaternion(const geometry_msgs::msg::Quaternion quat);

        // Degrees and radians conversions
        std::vector<double> deg_from_rad(const std::vector<double>);
        std::vector<double> rad_from_deg(const std::vector<double>);

        // Joint states conversions
        sensor_msgs::msg::JointState joint_state_from_vector(const std::vector<double> positions);
        std::vector<double> vector_from_joint_state(const sensor_msgs::msg::JointState joint_state);

        // Pose conversions
        geometry_msgs::msg::Pose pose_from_vector(const std::vector<double> vector_pos);
        std::vector<double> vector_from_pose(const geometry_msgs::msg::Pose pose);
        
        // Distance utils
        double euclidean_distance(const std::vector<double>& a, const std::vector<double>& b);
        double euclidean_distance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);
        double angular_distance(const geometry_msgs::msg::Quaternion& q1, const geometry_msgs::msg::Quaternion& q2);

        // Kinematics params getters
        geometry_msgs::msg::Pose  getFKineClient(const sensor_msgs::msg::JointState joint_state = sensor_msgs::msg::JointState()); // Get the forward kinematics of the pose, if empty uses the current joint state
        Eigen::MatrixXd           pseudoInverseClient(void);
        std::vector<double>       invKineClient(const geometry_msgs::msg::Pose pose);
        Eigen::MatrixXd           getJacobianClient(void);
        bool                      gripperMoveClient(const bool close);

        template <typename T>
        T getManipulatorParameter(const std::string& param_name);

        // Kinematics params setters
        void setJacobianSpeedControl(bool);
        void setNewPlannerParams(float,float);
        void setJsRealTimeControl(bool);

    private:

        // --------------------- PRIVATE PUBS/SUBS ---------------------

        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr& msg);
        void trajectoryCallback(const manipulator_interfaces::msg::TrajectoryResult::SharedPtr& msg);

        // --------------------- USER ACTIONS ---------------------
        
        void userJointGoal(void);            // to perform a joint goal set by the user 
        void userOneJointMove_user();        // to move only a single joint
        
        void userTcpGoal(void);              // to perform a tcp goal set by the user 

        void userMoveAlongX(void);           // to move the end effector along the x axis
        void userMoveAlongY(void);           // to move the end effector along the y axis
        void userMoveAlongZ(void);           // to move the end effector along the z axis

        void userMakeTcpRot(void);           // to rotate the end effector around the 3 carthesian axis
        void userRotateAroundX(void);        // to rotate the end effector around the x axis
        void userRotateAroundY(void);        // to rotate the end effector around the y axis
        void userRotateAroundZ(void);        // to rotate the end effector around the z axis
        
        //Known positions
        void userGoHomeDown(void);           // to go to the home position gripper facing down
        void userGoHomeFront(void);          // to go to the home position gripper facing front

        //Visualization
        void userJointStateVisualizer();
        void userEEPoseVisualizer();

        // Environment updates functions
        void userAddCollObj(void);          // Add a collision object by the user
        void userDeleteCollObj(void);       // Delete a given collision object from the user menu
        void userAddAttachedObj(void);  // Add an attached collision object by the user

        // Kinematics queries
        void userGetInvKine(void);           // Get the inverse kinematics of a given pose
        void userGetPseudoInv(void);         // Get the pseudo inverse of the manipulator
        void userGetJacobian(void);          // Get the jacobian of the manipulator

        // Planner params
        void userSetPlannerParams(void);            // Set the planner parameters
        void userSetJacobianSpeedControl(void);     // Set the jacobian speed control
        void userSetRealTimeControl(void);          // Set the real time control of the joints

        // Gripper
        void userGripperMove(void);                 // Move the gripper

        void userRunTest(void);                     // Run a test function

        //Initialize the menu instance and add the menu options and sections
        void initializeMenu();

        // --------------------- PRIVATE VARIABLES ---------------------

        ManipulatorMenuParams params_;

        rclcpp::Node::SharedPtr node_;

        MenuUserInterface<ManipulatorMenu> *menu_;

        uint clients_wait_timeout_ {10}; //Seconds

        // Ros
        rclcpp::Publisher<manipulator_interfaces::msg::JointGoal>::SharedPtr jointGoal_pub_;   
        rclcpp::Publisher<manipulator_interfaces::msg::TcpGoal>::SharedPtr tcpGoal_pub_;

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr displayGoal_pub_;
        rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr collisionObject_pub_;
        rclcpp::Publisher<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr attachedCollisionObject_pub_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointState_sub_;

        rclcpp::Client<manipulator_interfaces::srv::ChangePlannerParameters>::SharedPtr changePlannerParams_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr setJacobianControl_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr setRealTimeControl_client_;
        
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr gripperGrab_client_; //Not implemented for now
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr gripperMove_client_;

        rclcpp::SyncParametersClient::SharedPtr getManipulatorParams_client_; //gets the parameter from the manipulator planner node

        // Kinematics clients
        rclcpp::Client<manipulator_interfaces::srv::InvKine>::SharedPtr invKine_client_;
        rclcpp::Client<manipulator_interfaces::srv::PseudoInverse>::SharedPtr pseudoInverse_client_;
        rclcpp::Client<manipulator_interfaces::srv::FKine>::SharedPtr fKine_client_;
        rclcpp::Client<manipulator_interfaces::srv::Jacobian>::SharedPtr jacobian_client_;

    public:
        // --------------------- PUBLIC VARIABLES ---------------------

        // Planning 
        rclcpp::Subscription<manipulator_interfaces::msg::TrajectoryResult>::SharedPtr plannedTrajectory_sub_;      // Subscription to the planned trajectory
        rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr trajectory_pub_;                            // Publishes the trajectory to be executed
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr executionControl_pub_;                                    // Moves or stops the robot

        moveit_msgs::msg::RobotTrajectory planned_trajectory_;
        bool traj_received_;
        bool traj_error_;

        // Robot state
        geometry_msgs::msg::PoseStamped current_tcp_pose_;
        std::unordered_map<std::string, double> joints_map_group_;
        std::vector<double> joints_values_group_;
};

template class MenuUserInterface<ManipulatorMenu>;

#endif /* MANIPULATOR_MENU_H */
