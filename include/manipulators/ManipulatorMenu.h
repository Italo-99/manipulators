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
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/position_constraint.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/orientation_constraint.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
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
#include "manipulator_interfaces/srv/inv_kine.hpp"
#include "manipulator_interfaces/srv/pseudo_inverse.hpp"
#include "manipulator_interfaces/srv/f_kine.hpp"
#include "manipulator_interfaces/srv/jacobian.hpp"
#include "manipulator_interfaces/srv/change_planner_tolerances.hpp"
#include "manipulator_interfaces/srv/change_planner_scaling_factors.hpp"
#include "manipulator_interfaces/srv/enable_real_time_constraints.hpp"
#include "manipulator_interfaces/srv/set_frame.hpp"

#include "manipulator_interfaces/msg/joint_goal.hpp"
#include "manipulator_interfaces/msg/tcp_goal.hpp"
#include "manipulator_interfaces/msg/cartesian_goal.hpp"
#include "manipulator_interfaces/msg/trajectory_result.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "manipulators/MenuUserInterface.hpp"

#define DEFAULT_PLANNING_TIMEOUT 2
#define DEFAULT_EXECUTION_TIMEOUT 20 

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
    double ros_freq                      = 10.0;
    std::string manipulator_name         = "manipulator";
    std::string planning_group           = "ur_manipulator";

    std::vector<std::string> joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    std::string base_link_name           = "base_link";
    std::string ee_link_name             = "tcp_gripper";
    
    double tcp_position_tolerance        = 0.01; //Tolerance for the position of the end effector
    double tcp_orientation_tolerance     = 0.01; //Tolerance for the orientation of the end effector
    double joint_tolerance               = 0.01; //Tolerance for the position of the joints
    
    bool has_admittance                  = false; //Whether the manipulator control has admittance capability (admittance_controller node must be running)

    std::string known_poses_path          = "";  

    std::string gripper                  = "no_gripper"; //Must be in list of available grippers

    //Robotiq 85 gripper parameters
    std::string gripper_group            = "robotiq_85_gripper";
    //toolIO gripper parameters
    std::vector<int64_t> gripper_IO_cmds  = {0, 0};
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
            @note This function will block until the trajectory is planned or the timeout is reached.
        */
        manipulator_interfaces::msg::TrajectoryResult planAndWait(
            const sensor_msgs::msg::JointState joint_goal, 
            const std::vector<double> start_state = std::vector<double>(),
            uint timeout=DEFAULT_PLANNING_TIMEOUT);
        
        /*!
            @brief Send a TCP goal to the manipulator planner and wait for the result.
            @param tcp_goal: TCP goal to be published.
            @param start_state: Start state of the manipulator (in degrees).
            @param timeout: Timeout for the operation (in seconds).
            @return The planned trajectory result (see manipulator_interfaces::msg::TrajectoryResult).
            @note This function will block until the trajectory is planned or the timeout is reached.
        */
        manipulator_interfaces::msg::TrajectoryResult planAndWait(
            const geometry_msgs::msg::Pose tcp_goal, 
            const std::vector<double> start_state = std::vector<double>(), 
            const std::string& frame = "", //Leave empty for default frame
            uint timeout=DEFAULT_PLANNING_TIMEOUT);

        /*!
            @brief Send a Cartesian goal to the manipulator planner and wait for the result.
            @param waypoints: Waypoints to be published.
            @param start_state: Start state of the manipulator (in degrees).
            @param timeout: Timeout for the operation (in seconds).
            @return The planned trajectory result (see manipulator_interfaces::msg::TrajectoryResult).
            @note This function will block until the trajectory is planned or the timeout is reached.
        */
        manipulator_interfaces::msg::TrajectoryResult cartesianPlanAndWait(
            const std::vector<geometry_msgs::msg::Pose> waypoints, 
            const std::vector<double> start_state = std::vector<double>(),
            const std::string& frame = "", //Leave empty for default frame
            uint timeout=DEFAULT_PLANNING_TIMEOUT);

        /*!
            @brief Execute a trajectory, blocks until the trajectory is executed or timeout is reached.
            @param joint_trajectory: Joint trajectory to be executed.
            @param timeout: Timeout for the operation (in seconds).
            @return True if the trajectory was executed successfully, false otherwise.
            @note This function will block until the trajectory is executed or the timeout is reached.
        */
        bool executeAndWait(const moveit_msgs::msg::RobotTrajectory joint_trajectory, uint timeout=DEFAULT_EXECUTION_TIMEOUT);

        /*!
            @brief Plan a joint goal, execute it and wait for the result.
            @details This function combines the two functions planAndWait() and executeAndWait().
            @param joint_goal: Joint goal to be published (in radians).
            @param timeout_planning: Timeout for the planning operation (in seconds).
            @param timeout_execution: Timeout for the execution operation (in seconds).
            @return True if the trajectory was executed successfully, false otherwise.
            @note This function will block until the trajectory is executed or the timeout is reached.
        */
       bool planExecuteAndWait(
            const sensor_msgs::msg::JointState joint_goal, 
            uint timeout_planning=DEFAULT_PLANNING_TIMEOUT,
            uint timeout_execution=DEFAULT_EXECUTION_TIMEOUT
        );
        
        /*!
            @brief Plan a joint goal, execute it and wait for the result.
            @details This function combines the two functions planAndWait() and executeAndWait().
            @param joint_goal: Joint goal to be published (in degrees).
            @param timeout_planning: Timeout for the planning operation (in seconds).
            @param timeout_execution: Timeout for the execution operation (in seconds).
            @return True if the trajectory was executed successfully, false otherwise.
            @note This function will block until the trajectory is executed or the timeout is reached.
        */
        bool planExecuteAndWait(
            const std::vector<double> joint_goal, 
            uint timeout_planning=DEFAULT_PLANNING_TIMEOUT,
            uint timeout_execution=DEFAULT_EXECUTION_TIMEOUT
        );

         /*!
            @brief Plan a joint goal, execute it and wait for the result.
            @details This function combines the two functions planAndWait() and executeAndWait().
            @param known_pose: Name of the known pose to plan to.
            @param timeout_planning: Timeout for the planning operation (in seconds).
            @param timeout_execution: Timeout for the execution operation (in seconds).
            @return True if the trajectory was executed successfully, false otherwise.
            @note This function will block until the trajectory is executed or the timeout is reached.
        */
        bool planExecuteAndWait(
            const std::string& known_pose,
            uint timeout_planning=DEFAULT_PLANNING_TIMEOUT,
            uint timeout_execution=DEFAULT_EXECUTION_TIMEOUT
        );

        /*!
            @brief Plan a TCP goal, execute it and wait for the result.
            @details This function combines planning and execution for a TCP goal position.
            @param tcp_goal: TCP goal position to be published.
            @param frame: Frame in which the position is expressed (leave empty for default frame).
            @param timeout_planning: Timeout for the planning operation (in seconds).
            @param timeout_execution: Timeout for the execution operation (in seconds).
            @return True if the trajectory was executed successfully, false otherwise.
            @note This function will block until the trajectory is executed or the timeout is reached.
        */
        bool planExecuteAndWait(
            const geometry_msgs::msg::Pose tcp_goal, 
            const std::string& frame = "", //Leave empty for default frame
            uint timeout_planning=DEFAULT_PLANNING_TIMEOUT,
            uint timeout_execution=DEFAULT_EXECUTION_TIMEOUT
        );

        /*!
            @brief Plan a Cartesian path through waypoints, execute it and wait for the result.
            @details This function combines planning and execution for a Cartesian trajectory through multiple waypoints.
            @param waypoints: Vector of waypoint poses defining the Cartesian path.
            @param frame: Frame in which the positions are expressed (leave empty for default frame).
            @param timeout_planning: Timeout for the planning operation (in seconds).
            @param timeout_execution: Timeout for the execution operation (in seconds).
            @return True if the trajectory was executed successfully, false otherwise.
            @note This function will block until the trajectory is executed or the timeout is reached.
        */
        bool cartesianPlanExecuteAndWait(
            const std::vector<geometry_msgs::msg::Pose> waypoints, 
            const std::string& frame = "", //Leave empty for default frame
            uint timeout_planning=DEFAULT_PLANNING_TIMEOUT,
            uint timeout_execution=DEFAULT_EXECUTION_TIMEOUT
        );

        /*!
            @brief Stop trajectory execution.
        */
        void stopTrajectory();
        
        /*!
            @brief Send stop command to all real time controls.
        */
        void stopRealTime();
        
        /*!
            @brief Stops whatever movement the manipulator is doing.
        */
        void emergencyStop();

        /*!
            @brief Get the current end effector pose.
            @note Might block due to service call, so use with caution in real-time applications.
        */
        geometry_msgs::msg::Pose getEEpose();

        /*!
            @brief Get the current position of the end effector in the form of a vector [x (m), y(m), z(m), roll(°), pitch(°), yaw(°)].
            @note Might block due to service call, so use with caution in real-time applications.
        */
        std::vector<double> getEEpos_rpy();

        /*!
            @brief Get the current joint state of the manipulator in radians.
        */
        sensor_msgs::msg::JointState getJointStateRadians();

        /*!
            @brief Get the current joint state of the manipulator in degrees.
        */
        std::vector<double> getJointStateDegrees();


        /*!
            @brief Get the transformation from src_frame to dest_frame.
        */
        geometry_msgs::msg::TransformStamped getTf(const std::string& src_frame, const std::string& dest_frame, uint num_tries=10);

        /*!
            @brief Get the pose of target_frame relative to reference_frame.
        */
        geometry_msgs::msg::PoseStamped getTfPose(const std::string& target_frame, const std::string& reference_frame, uint num_tries=10);

        /*!
            @brief Transform a pose from src_frame to dest_frame.
            @param src_frame: The frame in which the pose is currently expressed.
            @param dest_frame: The frame in which the pose should be expressed.
            @param pose: The pose to transform (relative to src_frame).
            @return The transformed pose expressed in dest_frame.
        */
        geometry_msgs::msg::PoseStamped transformPose(const std::string& src_frame, 
                                                      const std::string& dest_frame,
                                                      const geometry_msgs::msg::Pose &pose);
        
        /*!
            @brief Transform a twist from src_frame to dest_frame.
            @param src_frame: The frame in which the twist is currently expressed.
            @param dest_frame: The frame in which the twist should be expressed.
            @param twist: The twist to transform (relative to src_frame).
            @return The transformed twist expressed in dest_frame.
        */
        geometry_msgs::msg::TwistStamped transformTwist(const std::string& src_frame, 
                                                      const std::string& dest_frame,
                                                      const geometry_msgs::msg::Twist &twist);

        /*!
            @brief Get a pose offset from the passed one (axis should be treated same way as a tf frame).
        */
       geometry_msgs::msg::Pose getOffsetPose(const geometry_msgs::msg::Pose &pose, 
                                              const geometry_msgs::msg::Pose &offset);

        /*!
            @brief Move the end effector x_step meters along the x axis.
            @param x_step: Distance to move along the x axis (in meters).
            @param cartesian: If true, the movement will be planned as a Cartesian path, otherwise it will be a simple TCP goal.
            @return The new end effector pose after the movement.
        */
        geometry_msgs::msg::Pose move_along_x(const double x_step,bool cartesian = false);

        /*!
            @brief Move the end effector y_step meters along the y axis.
            @param y_step: Distance to move along the y axis (in meters).
            @param cartesian: If true, the movement will be planned as a Cartesian path, otherwise it will be a simple TCP goal.
            @return The new end effector pose after the movement.
        */
        geometry_msgs::msg::Pose move_along_y(const double y_step,bool cartesian = false);

        /*!
            @brief Move the end effector z_step meters along the z axis.
            @param z_step: Distance to move along the z axis (in meters).
            @param cartesian: If true, the movement will be planned as a Cartesian path, otherwise it will be a simple TCP goal.
            @return The new end effector pose after the movement.
        */
        geometry_msgs::msg::Pose move_along_z(const double z_step,bool cartesian = false);

        /*!
            @brief Rotate the end effector (relative to current orientation).
            @details rotate_around_x, rotate_around_y and rotate_around_z are used to rotate the end effector around the x, y and z axes respectively.
            @param rot_vec: Vector containing the rotation angles in degrees [x_rot, y_rot, z_rot].
            @return The new end effector pose after the rotation.
            @note The rotation is relative to the current orientation of the end effector.
        */
        geometry_msgs::msg::Pose make_tcp_rot(const std::vector<double> rot_vec);
        geometry_msgs::msg::Pose rotate_around_x(const double x_rot_step);
        geometry_msgs::msg::Pose rotate_around_y(const double y_rot_step);
        geometry_msgs::msg::Pose rotate_around_z(const double z_rot_step);
        geometry_msgs::msg::Pose change_tcp_orient(const std::vector<double> rot_vec);

        /*!
            @brief Move the gripper to the specified position.
            @param close: If true, the gripper will be closed, otherwise it will be opened.
            @note The gripper type must be set in the parameters.
        */
        void moveGripper(const bool close);

        /*!
            @brief Add a collision object to the planning scene.
            @param collisionObjectMsg: Collision object message to be published.
        */
        void publishCollisionObject(const moveit_msgs::msg::CollisionObject collisionObjectMsg);

        /*!
            @brief Add a collision object to the planning scene in form of a primitive shape.
            @param name: Name of the object.
            @param obj_type: Type of the object (1: BOX, DEFAULT_PLANNING_TIMEOUT: SPHERE, 3: CYLINDER, 4: CONE).
            @param obj_dims: Dimensions of the object (depends on the type, see below).
            @param obj_pos: Position of the object in the form [x (m), y (m), z (m)].
            @param rot_pos: Orientation of the object in the form [x (rad), y (rad), z (rad)].
            @param operation: Operation to be performed on the object (0: ADD, 1: REMOVE, DEFAULT_PLANNING_TIMEOUT: MOVE).
            @details
            - For a BOX, obj_dims should contain [length (m), width (m), height (m)].
            - For a SPHERE, obj_dims should contain [radius (m)].
            - For a CYLINDER or CONE, obj_dims should contain [height (m), radius (m)].
        */
        void addObj(const std::string&   name,
                    const int            obj_type, 
                    std::vector<double>  obj_dims, 
                    double               obj_pos[], 
                    double               rot_pos[],
                    uint                 operation);

        void addObj(const std::string&       name,
                    const int                obj_type, 
                    std::vector<double>      obj_dims, 
                    geometry_msgs::msg::Pose obj_pos,
                    uint                     operation);

        /*!
            @brief Remove a collision object from the planning scene.
            @param name: Name of the object to be removed.
        */

        void removeObj(const std::string& name);

        /*!
            @brief Publish a collision object attached to the end effector to the planning scene.
            @param collisionAttachedObjectMsg: Attached collision object message to be published.
            @details This function is used to attach an object to a link in the planning scene.
        */
        void publishAttachedCollisionObject(const moveit_msgs::msg::AttachedCollisionObject collisionAttachedObjectMsg);

        /*!
            @brief Add an attached collision object to the planning scene.
            @param name: Name of the object.
            @param obj_type: Type of the object (1: BOX, DEFAULT_PLANNING_TIMEOUT: SPHERE, 3: CYLINDER, 4: CONE).
            @param obj_dims: Dimensions of the object (depends on the type, see below).
            @param obj_pos: Position of the object in the form [x (m), y (m), z (m)].
            @param rot_pos: Orientation of the object in the form [x (rad), y (rad), z (rad), w (rad)].
            @param operation: Operation to be performed on the object (0: ADD, 1: REMOVE, DEFAULT_PLANNING_TIMEOUT: APPEND, 3: MOVE).
            @details
            - For a BOX, obj_dims should contain [length (m), width (m), height (m)].
            - For a SPHERE, obj_dims should contain [radius (m)].
            - For a CYLINDER or CONE, obj_dims should contain [height (m), radius (m)].
        */
        void addAttachedObj(const std::string&  name,
                            const int           obj_type, 
                            std::vector<double> obj_dims, 
                            double              obj_pos[], 
                            double              rot_pos[],
                            uint                operation);

        void addAttachedObj(const std::string&       name,
                            const int                obj_type, 
                            std::vector<double>      obj_dims, 
                            geometry_msgs::msg::Pose obj_pos,
                            uint                     operation);

        /*!
            @brief Remove an attached collision object from the planning scene.
            @param name: Name of the object to be removed.
        */
        void removeAttachedObj(const std::string& name); 
        

        // Path constraints

        /*!
            @brief Publish a joint constraint to the manipulator planner.
            @details The specified joint will be constrained to [position - tolerance_below, position + tolerance_above]
            @param joint_num: Index of the joint to be constrained (0-based).
            @param min_position: Minimum position of the joint (in degrees).
            @param max_position: Maximum position of the joint (in degrees).
            @param weight: Weight of the constraint (default is 1.0).
        */
        void publishJointConstraint(const uint &joint_num, 
                                    const double &min_position, 
                                    const double &max_position, 
                                    const double &weight  = 1.0);

        //Publish a primitive as a position constraint, the specified link will stay inside that primitive
        /*!
            @brief Publish a position constraint to the manipulator planner in form of a primitive.
            @details The specified link will stay inside that primitive.
            @param link_name: Name of the link to be constrained.
            @param shape_pose: Pose of the primitive in the form of a geometry_msgs::msg::Pose (relative to world).
            @param shape_type: Type of the primitive (1: BOX, DEFAULT_PLANNING_TIMEOUT: SPHERE, 3: CYLINDER, 4: CONE).
            @param shape_dims: Dimensions of the primitive (depends on the type, see below).
            @param weight: Weight of the constraint (default is 1.0).
            @details
            - For a BOX, shape_dims should contain [length (m), width (m), height (m)].
            - For a SPHERE, shape_dims should contain [radius (m)].
            - For a CYLINDER or CONE, shape_dims should contain [height (m), radius (m)].
        */
        void publishPositionConstraint(const std::string& link_name, 
                                       const geometry_msgs::msg::Pose& shape_pose, 
                                       const uint &shape_type, 
                                       const std::vector<double>& shape_dims, 
                                       const double &weight = 1.0);

        /*!
            @brief Publish an orientation constraint to the manipulator planner.
            @details The specified link will maintain the given orientation within the specified tolerances.
            @param link_name: Name of the link to be constrained.
            @param orientation: Orientation of the link in the form of a geometry_msgs::msg::Quaternion.
            @param tolerances: Tolerances for the orientation along the x, y, and z axes (default is {0.01, 0.01, 0.01}).
            @param weight: Weight of the constraint (default is 1.0).
        */
        void publishOrientationConstraint(const std::string& link_name, 
                                          const geometry_msgs::msg::Quaternion& orientation, 
                                          const std::vector<double> &tolerances = {0.01, 0.01, 0.01}, //Along x,y,z axis 
                                          const double &weight = 1.0);

        /*!
            @brief Publish a command to clear all path constraints in the manipulator planner.
            @details This will remove all previously set joint, position, and orientation constraints.
        */
        void publishClearConstraints(void); //Clear all constraints

        void publishToolIOCmd(const size_t id, const bool value); //Publish a command to the tool digital IO

        // Matrix utils
        void printMatrix(const Eigen::MatrixXd& matrix);
        void listToMatrix(const std::vector<double> &list, Eigen::MatrixXd &matrix);

        // Quaternions utils (rpy in degrees)
        geometry_msgs::msg::Quaternion quaternion_from_euler(double roll, double pitch, double yaw);
        std::vector<double> euler_from_quaternion(const geometry_msgs::msg::Quaternion quat); 
        geometry_msgs::msg::Quaternion quaternion_multiply(const geometry_msgs::msg::Quaternion& q1, const geometry_msgs::msg::Quaternion& q2);

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
        geometry_msgs::msg::Pose  getFKineClient(const sensor_msgs::msg::JointState joint_state = sensor_msgs::msg::JointState()); // Get the forward kinematics of the pose, if empty uses the current joint state
        Eigen::MatrixXd           pseudoInverseClient(void);
        std::vector<double>       invKineClient(const geometry_msgs::msg::Pose pose);
        Eigen::MatrixXd           getJacobianClient(void);
        bool                      gripperMoveClient(const bool close);

        //Setter clients
        /*!
            @brief Set the jacobian speed control parameter in the manipulator planner.
            @param enable: If true, the jacobian speed control will be enabled.
            @note This client doesn't expect actual results from the server, the response will only evaluate the success of the query and will be logged
        */
        void setJacobianSpeedControl(bool enable);
        /*!
            @brief Set the real time control parameter in the manipulator planner.
            @param enable: If true, the real time control will be enabled.
            @param stop_cmd: If true, a stop command will be sent to the real time control when enabling it to avoid unexpected movements.
            @note This client doesn't expect actual results from the server, the response will only evaluate the success of the query and will be logged
        */
        void setJsRealTimeControl(bool enable);

        /*!
            @brief Set the admittance control parameter in the manipulator planner.
            @param enable: If true, the admittance control will be enabled.
            @note This client doesn't expect actual results from the server, the response will only evaluate the success of the query and will be logged
        */
        void setAdmittanceControl(bool enable);

        /*!
            @brief Set the admittance control parameter in the manipulator planner.
            @param enable: If true, the admittance control will be enabled.
            @note This client doesn't expect actual results from the server, the response will only evaluate the success of the query and will be logged
        */
        void setAdmittanceVelMode(bool vel_mode);
        
        /*!
            @brief Set the planner velocity and acceleration factors.
            @param limit_joints: During real time control (both joints and jacobian) joints position will be limited according to limits and constraints.
            @param limit_jacobian: During jacobian control ee pose will be limited to the area specified by position constraints.
            @note This client doesn't expect actual results from the server, the response will only evaluate the success of the query and will be logged
        */
        void setRealTimeConstraints(bool limit_joints, bool limit_jacobian);

        /*!
            @brief Set the planner scaling factors.
            @param velocity_factor: Joints speed of planned trajectories will be scaled according to this value.
            @param acceleration_factor: Joints acceleration of planned trajectories will be scaled according to this value.
            @note This client doesn't expect actual results from the server, the response will only evaluate the success of the query and will be logged
        */
        void setPlannerScalingFactors(float vel_factor, float acc_factor);

        /*!
            @brief Set the tolerances in the manipulator planner.
            @param position_tolerance: Position tolerance for the end effector (in meters).
            @param orientation_tolerance: Orientation tolerance for the end effector (in radians).
            @param joint_tolerance: Joint tolerance for the manipulator (in radians).
            @note This client doesn't expect actual results from the server, the response will only evaluate the success of the query and will be logged
        */
        void setPlannerTolerances(float position_tolerance, float orientation_tolerance, float joint_tolerance);

        /*!
            @brief Publish a real time joint command to the manipulator planner.
            @details This will work only if real time control is enabled in the planner via the setJsRealTimeControl method.
            @param joint_speeds: Vector of joint velocities in degrees/s to be sent as a command.
        */
        void publishJointsCommand(const std::vector<double> joint_speeds);

        /*!
            @brief Publish a real time joint command to the manipulator planner.
            @details This will work only if real time control is enabled in the planner via the setJsRealTimeControl method.
            @param joint_state: JointState message containing the joint velocities in radians/s to be set.
        */
        void publishJointsCommand(const sensor_msgs::msg::JointState joint_state);

        /*!
            @brief Publish a real time Cartesian command to the manipulator planner.
            @details This will work only if real time control is enabled in the planner via the setJsRealTimeControl method.
            @param tcp_speed: Twist of the end effector containing the linear velocities in m/s and angular velocities in rad/s to be sent as a command.
            @param frame: Frame in which the twist is expressed (leave empty for default frame).
        */
        void publishJacobianCommand(const geometry_msgs::msg::Twist tcp_speed, const std::string& frame = "");

        /*!
            @brief Publish a real time Cartesian command to the manipulator planner.
            @details This will work only if real time control is enabled in the planner via the setJsRealTimeControl method.
            @param tcp_speed: Vector of 6 elements (vx, vy, vz, wx, wy, wz) containing the linear velocities in m/s and angular velocities in degrees/s to be sent as a command.
            @param frame: Frame in which the twist is expressed (leave empty for default frame).
        */
        void publishJacobianCommand(const std::vector<double> tcp_speed, const std::string& frame = "");

        /*!
            @brief Publish cartesian position command to the admittance controller.
            @details This will work only if admittance control is enabled in the planner via the setAdmittanceControl method and admittance control is set to position mode by setAdmittanceVelMode(false).
            @params tcp_pose: Desired tcp pose to be maintained by the admittance controlller.
            @params frame: Frame in which the pose is expressed (leave empty for default frame).
        */
        void publishAdmittancePositionCommand(const geometry_msgs::msg::Pose tcp_pose, const std::string& frame = "");
        
        /*!
            @brief Publish cartesian velocity command to the admittance controller.
            @details This will work only if admittance control is enabled in the planner via the setAdmittanceControl method and admittance control is set to velocity mode by setAdmittanceVelMode(true).
            @params tcp_speed: Twist of the end effector containing the linear velocities in m/s and angular velocities in radians/s to be sent as a command to the admittance controller.
            @params frame: Frame in which the twist is expressed (leave empty for default frame).
        */
        void publishAdmittanceVelocityCommand(const geometry_msgs::msg::Twist tcp_speed, const std::string& frame = "");

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
        void userOneJointMove();             // to move only a single joint
        
        void userTcpGoal(void);              // to perform a tcp goal set by the user 
        
        void userCartesianGoal(void);        // to perform a cartesian goal of multiple waypoints set by the user

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
        void userGoToKnownPose(void);        // to go to a known pose specified in the yaml file by providing its id

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
        void userSetRealTimeConstraints(void);      // Enable/disable real time constraints
        void userSetJacobianSpeedControl(void);     // Enable/disable the jacobian speed control
        void userSetRealTimeControl(void);          // Enable/disable the real time control of the joints
        
        // Admittance control
        void userSetAdmittanceControl(void);            // Enable/disable admittance control
        void userSetAdmittanceControlMode(void);        // Switch between admittance control modes

        // Gripper
        void userGripperMove(void);                 // Move the gripper

        void userRunTest(void);                     // Run a test function

        //Initialize the menu instance and add the menu options and sections
        void initializeMenu();

        // --------------------- PRIVATE VARIABLES ---------------------

        ManipulatorMenuParams params_;
        std::map<std::string, std::vector<double>> known_poses_; //Map of known poses

        rclcpp::executors::MultiThreadedExecutor executor_;
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
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr eePose_sub_; //Subscription to the end effector pose
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr eeVel_sub_; //Subscription to the end effector twist

        //Setter clients
        rclcpp::Client<manipulator_interfaces::srv::ChangePlannerScalingFactors>::SharedPtr changePlannerScalingFactors_client_;
        rclcpp::Client<manipulator_interfaces::srv::ChangePlannerTolerances>::SharedPtr changePlannerTolerances_client_;
        rclcpp::Client<manipulator_interfaces::srv::EnableRealTimeConstraints>::SharedPtr enableRealTimeConstraints_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr setJacobianControl_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr setRealTimeControl_client_;

        //Admittance control
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr setAdmittanceControl_client_;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr setAdmittanceVelMode_client_;
        
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
 
        // TF
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        // Real time control publishers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velJacSetpoint_pub_;        //Publish end effector velocity commands to manipulator
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr velJsRtSetpoint_pub_;    //Publish joint velocity commands to manipulator
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velAdmSetpoint_pub_;        //Publish velocity commands to admittance controller
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr poseAdmSetpoint_pub_; //Publish pose commands to admittance controller
    
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
        geometry_msgs::msg::Pose current_tcp_pose_;
        geometry_msgs::msg::Twist current_tcp_vel_;
        std::unordered_map<std::string, double> joints_map_group_;
        std::vector<double> joints_values_group_;
};

template class MenuUserInterface<ManipulatorMenu>;

#endif /* MANIPULATOR_MENU_H */
