/** @file */
#ifndef MANIPULATOR_MENU_H
#define MANIPULATOR_MENU_H

// IMPORT LIBRARIES
#include <iostream>
#include <cmath>
#include <Eigen/Geometry>
#include <unordered_map>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/position_constraint.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/orientation_constraint.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int8.hpp"

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
#include "manipulator_interfaces/srv/change_planner_tolerances.hpp"
#include "manipulator_interfaces/srv/change_planner_scaling_factors.hpp"
#include "manipulator_interfaces/msg/joint_goal.hpp"
#include "manipulator_interfaces/msg/tcp_goal.hpp"
#include "manipulator_interfaces/msg/cartesian_goal.hpp"
#include "manipulator_interfaces/msg/trajectory_result.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "manipulators/MenuUserInterface.h"

/*! @struct ManipulatorMenuParams
        @brief Struct to hold the parameters for the ManipulatorMenu class.

        @param node_name Name of the node.
        @param ros_freq Frequency of the node.
        @param manipulator_name Name of the manipulator, must match the one in the manipulator planner.
        @param planning_group Name of the planning group specified in the srdf, must match the one in the manipulator planner.
        @param joint_names Names of the joints, must be members of the planning group.
        @param base_link_name Name of the base link, used as a reference frame for coordinates.
        @param tcp_position_tolerance Tolerance for the position of the end effector (m).
        @param tcp_orientation_tolerance Tolerance for the orientation of the end effector (rad).
        @param joint_tolerance Tolerance for the position of the joints (rad).
        @param known_poses_path Full path to the yaml file with the known poses (leave empty if not used).
                               Known poses are defined as their name and a vector of joint angles in degrees eg: home: [0, 0, 0, 0, 0, 0].
        @param gripper Type of gripper to be used, must be in the list of available grippers.
        @param gripper_group (specific for gripper type robotiq_85) Name of the gripper group, must match the one in the srdf.
        @param gripper_IO_cmds (specific for gripper type toolIO) Tool IO commands for the gripper, must be in the list of available grippers.

        @details Available grippers:
  
        @li no_gripper: No gripper, only the manipulator is used.
        @li robotiq_85: Robotiq 85 gripper, actuated through a service.
        @li toolIO: Generic gripper actuated through the tool IO of the robot.
*/
struct ManipulatorMenuParams
{

    /*! Constructor with default values */
    ManipulatorMenuParams() = default;

    /*! Constructor which retrieves values from node parameters */
    ManipulatorMenuParams(const rclcpp::Node::SharedPtr& node);

    std::string node_name                = "manipulator_menu_node"; 
    double ros_freq                      = 10;
    std::string manipulator_name         = "manipulator";
    std::string planning_group           = "ur_manipulator";

    std::vector<std::string> joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    std::string base_link_name           = "base_link";
    
    double tcp_position_tolerance        = 0.01; //Tolerance for the position of the end effector
    double tcp_orientation_tolerance     = 0.01; //Tolerance for the orientation of the end effector
    double joint_tolerance               = 0.01; //Tolerance for the position of the joints

    std::string known_poses_path          = "";  

    std::string gripper                  = "no_gripper"; //Must be in list of available grippers

    //Robotiq 85 gripper parameters
    std::string gripper_group            = "robotiq_85_gripper";
    //toolIO gripper parameters
    std::vector<int8_t> gripper_IO_cmds  = {0, 0};
};

/*! @class ManipulatorMenu
*       @brief Main class for the manipulator menu.
*       @details This class is used to create a high level, command line interface for the manipulator planner.
*                It allows to plan and execute trajectories as well as perform various operations on the manipulator.
*       @note In general when using the sensor_msgs/JointState type the angles will be expressed in radians, 
*             instead when using a vector joint angles will be expressed in degrees.
*             In a similar way when using vectors to represent poses, the first 3 elements will be the x, y 
*             and z coordinates in meters while the last 3 elements will be the roll, pitch and yaw in degrees.
*  
*/
class ManipulatorMenu
{
    //NOTE: In general when using the sensor_msgs/JointState type the angles will be expressed in radians, 
    //      instead when using a vector joint angles will be expressed in degrees.
    //      In a similar way when using vectors to represent poses, the first 3 elements will be the x, y 
    //      and z coordinates in meters while the last 3 elements will be the roll, pitch and yaw in degrees.
    public:
        // ---------------------  PUBLIC CONSTRUCTOR ---------------------
        /*!
            @brief Constructor for the ManipulatorMenu class.
            @param params: Parameters for the manipulator menu (must match the ones passed to the manipulator planner)
            @param node: Pointer to the node that will host the menu
            @param sync_parameters: If true, the parameters will be synchronized with the manipulator planner automatically 
                             (may cause some delay or issues, if the menu blocks when started disable this option and 
                              manually set the parameters in src/manipulator_menu_user.cpp)
        */
        ManipulatorMenu(
            ManipulatorMenuParams params,
            const rclcpp::Node::SharedPtr& node,
            const bool sync_parameters = true
        );

        ~ManipulatorMenu();

        // ---------------------  PUBLIC FUNCTIONS ---------------------

        /*!
            @brief Start the menu and wait for user input.
            @details This function will start a thread that will run the function spinner() and then it will start the
            command line menu.
            @note This will block the execution until the user exits the menu.
        */
        void spinnerMenu(void);

        /*!
            @brief Start the actual spinner for the node.
            @details This function should run asynchronously to the menu interface and will handle the node specific tasks 
            such as publishing and subscribing to topics, waiting for services and handling the node lifecycle.
        */
        void spinner(void);

        // Goal publishers

        /*!
            @brief Publish a joint goal to the manipulator planner.
            @param joint_goal: Joint goal to be published (in degrees).
            @param start_state: Start state of the manipulator (in degrees).
            @param execute: If true, the trajectory will be executed.
            @return The passed joint goal.
        */
        sensor_msgs::msg::JointState publishJointGoal(
            const std::vector<double> joint_goal, 
            const std::vector<double> start_state = std::vector<double>(), 
            const bool execute=true);

        /*!
            @brief Publish a joint goal to the manipulator planner.
            @param joint_goal: Joint goal to be published (in radians).
            @param start_state: Start state of the manipulator (in degrees).
            @param execute: If true, the trajectory will be executed.
            @return The passed joint goal.
        */
        sensor_msgs::msg::JointState publishJointGoal(
            const sensor_msgs::msg::JointState joint_goal, 
            const std::vector<double> start_state = std::vector<double>(), 
            const bool execute=true);

        /*!
            @brief Publish a TCP goal to the manipulator planner.
            @param position: Position of the end effector (in meters).
            @param start_state: Start state of the manipulator (in degrees).
            @param frame: Frame in which the position is expressed (leave empty for default frame).
            @param execute: If true, the trajectory will be executed.
            @return The passed TCP goal.
        */
        geometry_msgs::msg::Pose publishTcpGoal(
            const std::vector<double> position, 
            const std::vector<double> start_state = std::vector<double>(),
            const std::string& frame = "", //Leave empty for default frame
            const bool execute=true);

        /*!
            @brief Publish a TCP goal to the manipulator planner.
            @param tcpPoseMsg: TCP goal to be published (in meters).
            @param start_state: Start state of the manipulator (in degrees).
            @param frame: Frame in which the position is expressed (leave empty for default frame).
            @param execute: If true, the trajectory will be executed.
            @return The passed TCP goal.
        */
        geometry_msgs::msg::Pose publishTcpGoal(
            const geometry_msgs::msg::Pose tcpPoseMsg, 
            const std::vector<double> start_state = std::vector<double>(),
            const std::string& frame = "", //Leave empty for default frame
            const bool execute=true);

        /*!
            @brief Publish a TCP goal to the manipulator planner.
            @param tcp_goal: TCP goal to be published (in meters).
            @param start_state: Start state of the manipulator (in degrees).
            @param frame: Frame in which the position is expressed (leave empty for default frame).
            @param execute: If true, the trajectory will be executed
        */
        void publishCartesianGoal(
            const std::vector<geometry_msgs::msg::Pose> waypoints,
            const std::vector<double> start_state = std::vector<double>(),
            const std::string& frame = "", //Leave empty for default frame
            const bool execute=true
        );
        
        /*!
            @brief Execute movement of a single joint.
            @param num: Number of the joint to be moved.
            @param joint_rot: Degrees to rotate the joint.
            @return The joint state of the manipulator after the movement.
        */
        sensor_msgs::msg::JointState oneJointMove(const int num, const double joint_rot); // to execute rotation of a single joint

        /*!
            @brief Get the joint state of a known pose specified in the yaml file.
            @param pose_name: Name of the pose to be retrieved.
            @return The known pose as a vector of joint angles in degrees.
        */
        std::vector<double> getKnownPose(const std::string& pose_name); //Get a known pose from the yaml file

        // Planning 

        /*!
            @brief Send a joint goal to the manipulator planner and wait for the result.
            @param joint_goal: Joint goal to be published (in radians).
            @param start_state: Start state of the manipulator (in degrees).
            @param timeout: Timeout for the operation (in seconds).
            @return The planned trajectory result (see manipulator_interfaces::msg::TrajectoryResult).
        */
        manipulator_interfaces::msg::TrajectoryResult planAndWait(
            const sensor_msgs::msg::JointState joint_goal, 
            const std::vector<double> start_state = std::vector<double>(),
            uint timeout=2);
        
        /*!
            @brief Send a TCP goal to the manipulator planner and wait for the result.
            @param tcp_goal: TCP goal to be published.
            @param start_state: Start state of the manipulator (in degrees).
            @param timeout: Timeout for the operation (in seconds).
            @return The planned trajectory result (see manipulator_interfaces::msg::TrajectoryResult).
        */
        manipulator_interfaces::msg::TrajectoryResult planAndWait(
            const geometry_msgs::msg::Pose tcp_goal, 
            const std::vector<double> start_state = std::vector<double>(), 
            const std::string& frame = "", //Leave empty for default frame
            uint timeout=2);

        /*!
            @brief Send a Cartesian goal to the manipulator planner and wait for the result.
            @param waypoints: Waypoints to be published.
            @param start_state: Start state of the manipulator (in degrees).
            @param timeout: Timeout for the operation (in seconds).
            @return The planned trajectory result (see manipulator_interfaces::msg::TrajectoryResult).
        */
        manipulator_interfaces::msg::TrajectoryResult cartesianPlanAndWait(
            const std::vector<geometry_msgs::msg::Pose> waypoints, 
            const std::vector<double> start_state = std::vector<double>(), 
            const std::string& frame = "", //Leave empty for default frame
            uint timeout=2);

        /*!
            @brief Execute a trajectory, blocks until the trajectory is executed or timeout is reached.
            @param joint_trajectory: Joint trajectory to be executed.
            @param timeout: Timeout for the operation (in seconds).
            @return True if the trajectory was executed successfully, false otherwise.
        */
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

        void moveGripper(const bool close);

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
        

        // Path constraints

        // Publish joint constraint, the specified joint will be constrained to [position - tolerance_below, position + tolerance_above]
        void publishJointConstraint(const uint &joint_num, 
                                    const double &position, 
                                    const double &tolerance_above, 
                                    const double &tolerance_below, 
                                    const double &weight  = 1.0);


        //Publish a primitive as a position constraint, the specified link will stay inside that primitive
        void publishPositionConstraint(const std::string& link_name, 
                                       const geometry_msgs::msg::Pose& shape_pose, 
                                       const uint &shape_type, 
                                       const std::vector<double>& shape_dims, 
                                       const double &weight = 1.0);

        //Publish an orientation constraint, the specified link will maintain that orientation +- the specified tolerances
        void publishOrientationConstraint(const std::string& link_name, 
                                          const geometry_msgs::msg::Quaternion& orientation, 
                                          const std::vector<double> &tolerances = {0.01, 0.01, 0.01}, //Along x,y,z axis 
                                          const double &weight = 1.0);

        void publishClearConstraints(void); //Clear all constraints

        void publishToolIOCmd(const size_t id, const bool value); //Publish a command to the tool digital IO

        // Matrix utils
        void printMatrix(const Eigen::MatrixXd& matrix);
        void listToMatrix(const std::vector<double> &list, Eigen::MatrixXd &matrix);

        // Quaternions utils (rpy in degrees)
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

        // Kinematics clients
        // These clients will block execution until a response is received or timeout is reached
        geometry_msgs::msg::Pose  getFKineClient(const sensor_msgs::msg::JointState joint_state = sensor_msgs::msg::JointState()); // Get the forward kinematics of the pose, if empty uses the current joint state
        Eigen::MatrixXd           pseudoInverseClient(void);
        std::vector<double>       invKineClient(const geometry_msgs::msg::Pose pose);
        Eigen::MatrixXd           getJacobianClient(void);
        bool                      gripperMoveClient(const bool close);

        //Setter clients
        //These clients don't expect actual results from the server, the response will only evaluate the success of the query and will be logged
        void setJacobianSpeedControl(bool);
        void setJsRealTimeControl(bool);
        void setPlannerScalingFactors(float,float);
        void setPlannerTolerances(float,float,float);

        //Get parameter from manipulator_planner node
        template <typename T>
        T getManipulatorParameter(const std::string& param_name);

    protected:

        void loadKnownPoses();

        void waitManipulatorParameters();

        void shutdown_handler(); // Shutdown handler for the node

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

        // Constraints
        void userAddJointConstraint(void);       // Add a joint constraint
        void userAddPositionConstraint(void);   // Add a position constraint
        void userAddOrientationConstraint(void); // Add an orientation constraint
        void userClearConstraints(void);         // Clear all constraints

        // Kinematics queries
        void userGetInvKine(void);           // Get the inverse kinematics of a given pose
        void userGetPseudoInv(void);         // Get the pseudo inverse of the manipulator
        void userGetJacobian(void);          // Get the jacobian of the manipulator

        // Planner params
        void userSetPlannerScalingFactors(void);    // Set the planner velocity and acceleration factors
        void userSetPlannerTolerances(void);        // Set the planner tolerances
        void userSetJacobianSpeedControl(void);     // Set the jacobian speed control
        void userSetRealTimeControl(void);          // Set the real time control of the joints

        // Gripper
        void userGripperMove(void);                 // Move the gripper

        void userRunTest(void);                     // Run a test function

        //Initialize the menu instance and add the menu options and sections
        void initializeMenu();

        // --------------------- PRIVATE VARIABLES ---------------------

        ManipulatorMenuParams params_;
        std::map<std::string, std::vector<double>> known_poses_; //Map of known poses

        rclcpp::executors::SingleThreadedExecutor executor_;
        rclcpp::Node::SharedPtr node_;

        MenuUserInterface<ManipulatorMenu> *menu_;

        uint clients_wait_timeout_ {10}; //Seconds

        //Planner goals publishers
        rclcpp::Publisher<manipulator_interfaces::msg::JointGoal>::SharedPtr jointGoal_pub_;   
        rclcpp::Publisher<manipulator_interfaces::msg::TcpGoal>::SharedPtr tcpGoal_pub_;
        rclcpp::Publisher<manipulator_interfaces::msg::CartesianGoal>::SharedPtr cartesianPlan_pub_;

        //Constraints publishers
        rclcpp::Publisher<moveit_msgs::msg::JointConstraint>::SharedPtr jointConstraints_pub_;
        rclcpp::Publisher<moveit_msgs::msg::PositionConstraint>::SharedPtr positionConstraints_pub_;
        rclcpp::Publisher<moveit_msgs::msg::OrientationConstraint>::SharedPtr orientationConstraints_pub_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr clearConstraints_pub_;

        //Collision objects publishers
        rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr collisionObject_pub_;
        rclcpp::Publisher<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr attachedCollisionObject_pub_;

        //Publisher to /ur_rtde/tool_digitalIO/command to send dital IO commands to end effector
        //This is used by gripper type toolIO and needs the correct middleware to communicate with the robot IO (eg: ars control lab ur_rtde_controller lib)
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr toolDigitalIO_pub_;

        //Subscriber to /joint_states topic
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointState_sub_;

        //Setter clients
        rclcpp::Client<manipulator_interfaces::srv::ChangePlannerScalingFactors>::SharedPtr changePlannerScalingFactors_client_;
        rclcpp::Client<manipulator_interfaces::srv::ChangePlannerTolerances>::SharedPtr changePlannerTolerances_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr setJacobianControl_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr setRealTimeControl_client_;
        
        //Gripper clients for robotiq85 gripper
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr gripperGrab_client_; //Not implemented for now
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr gripperMove_client_;

        //Client to retrieve parameters from the manipulator_planner node automatically
        rclcpp::SyncParametersClient::SharedPtr getManipulatorParams_client_; //gets the parameter from the manipulator planner node

        // Kinematics clients
        rclcpp::Client<manipulator_interfaces::srv::InvKine>::SharedPtr invKine_client_;
        rclcpp::Client<manipulator_interfaces::srv::PseudoInverse>::SharedPtr pseudoInverse_client_;
        rclcpp::Client<manipulator_interfaces::srv::FKine>::SharedPtr fKine_client_;
        rclcpp::Client<manipulator_interfaces::srv::Jacobian>::SharedPtr jacobian_client_;

    public:
        // --------------------- PUBLIC VARIABLES ---------------------

        sensor_msgs::msg::JointState current_joint_pose_;

        // Planning 
        rclcpp::Subscription<manipulator_interfaces::msg::TrajectoryResult>::SharedPtr plannedTrajectory_sub_;      // Subscription to the planned trajectory
        rclcpp::Publisher<moveit_msgs::msg::RobotTrajectory>::SharedPtr trajectory_pub_;                            // Publishes the trajectory to be executed
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr executionControl_pub_;                                    // Moves or stops the robot

        manipulator_interfaces::msg::TrajectoryResult traj_result_; // Planned trajectory
        bool traj_received_ = false; 

        // Robot state
        geometry_msgs::msg::PoseStamped current_tcp_pose_;
        std::unordered_map<std::string, double> joints_map_group_;
        std::vector<double> joints_values_group_;
};

template class MenuUserInterface<ManipulatorMenu>;

#endif /* MANIPULATOR_MENU_H */
