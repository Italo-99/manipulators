#ifndef DYNAMIC_PLANNER_H
#define DYNAMIC_PLANNER_H

//C++ Imports
#include <string>
#include <vector>
#include <Eigen/Geometry>

//ROS Imports
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <tf2/convert.h>
#include "rviz_visual_tools/rviz_visual_tools.hpp"
//MoveIt2 Imports
#include <moveit/move_group/capability_names.h>
#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <manipulator_interfaces/msg/trajectory_result.hpp> 
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <moveit_msgs/srv/get_motion_sequence.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit/kinematic_constraints/utils.h>

// Struct definition of the parameters of the Dynamic Planner

struct DynamicPlannerParams
{
    std::string planning_pipeline   = "ompl";                    // planning pipeline (check moveit_config package for available pipelines)
    std::string planner_id          = "geometric::RRTConnect";   // name of the planner method (look up from list in <planning_pipeline>.yaml)
    int num_attempts                = 2;                         // max number of attempts to find a trajectory
    double planning_time            = 2;                         // maximum planning time in seconds (0: no limit)
    double vel_factor               = 1.;                        // velocity factor
    double acc_factor               = 1.;                        // acceleration factor
    double sample_time              = 0.002;                     // sample time for cartesian planner (seconds)
    double max_velocity             = 0.5;                       // maximum ee velocity for cartesian planner
    double position_tolerance       = 0.01;                      // tolerance for tcp position (m)
    double orientation_tolerance    = 0.01;                      // tolerance for tcp orientation (rad)
    double joint_tolerance          = 0.01;                      // tolerance for joint positions (rad)
    std::string world_frame         = "base_link";               // world frame
    std::string end_effector_link   = "tool0";                   // end effector link
    double min_cartesian_fraction   = 0.1;                       // Minimum fraction for the cartesian plan to be considered successful
    double caresian_blend_radius    = 0.0;                       // Blend radius for the cartesian plan

    DynamicPlannerParams() {}

    static DynamicPlannerParams fromNode(const rclcpp::Node::SharedPtr& node);
};

class DynamicPlanner
{
    public:
        DynamicPlanner(const rclcpp::Node::SharedPtr& node,
                       const std::string& planning_group,
                       bool dynamic_behavior = true);

        ~DynamicPlanner();

        void initialize(); //Initialize the dynamic planner (vars, subscribers, publishers, ...)

        // --------------- CONTROL METHODS ---------------
        
        enum PlanningSpace : bool
        {
            JOINTS_SPACE = 0,
            OPERATIVE_SPACE = 1
        };

        /*plan: joint goal
            NOTE: This is a blocking function, it will stop execution until the planning is done
            Args:
                joint_positions: Array of target joint position
        */
        void plan(const std::vector<double> joint_positions, const moveit::core::RobotStatePtr start_state);
        void plan(const std::vector<double> joint_positions);

        /*plan: pose goal
            NOTE: This is a blocking function, it will stop execution until the planning is done
            Args:
                goal_pose: Target position
                ee_link: End effector link
                frame: Reference frame
            
            Overloads use default values specified in the class definition and params
        */

        void plan(const geometry_msgs::msg::Pose& goal_pose, const std::string& ee_link, const std::string& frame);
        void plan(const geometry_msgs::msg::Pose& goal_pose, const std::string& ee_link);
        void plan(const geometry_msgs::msg::Pose& goal_pose);

        /*plan: cartesian goal
            Args:
                waypoints: Array of target positions to follow
                ee_link: End effector link
                frame: Reference frame
            Returns:
                fraction: Fraction of the trajectory that was planned
        */
        double cartesianPlan(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                             const std::string& ee_link,
                             const std::string& frame);

        double cartesianPlan(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                             const std::string& ee_link);

        double cartesianPlan(const std::vector<geometry_msgs::msg::Pose>& waypoints);

        void moveRobot(const sensor_msgs::msg::JointState &joint_state); //Single trajectory point
        void moveRobot(const trajectory_msgs::msg::JointTrajectoryPoint &traj_pt); //Single trajectory point
        
        void executeTrajectory(); //Asynchronous execution of the last planned trajectory
        void executeTrajectory(moveit_msgs::msg::RobotTrajectory& robot_trajectory); //Asynchronous execution of the passed trajectory (waypoints)

        bool isMoving(); //Check if the robot is moving
        bool isReady() const; //Check if the planner has received group definition, so the dynamic planner can start working

        void stop(); //Stop the execution of the planned trajectory

        // --------------- GETTERS AND SETTERS ----------------

        // Dynamic planner parameters getter and setter
        DynamicPlannerParams getParams() const;

        void setParams(const DynamicPlannerParams& params);

        void setDynamicBehavior(bool dynamic_behavior); //Set the dynamic_behavior_ variable
        bool isDynamic() const; //Check if dynamic_behaviour_ is true

        void setPlanningSpace(PlanningSpace space); //Set the planning space (joint or operative)
        PlanningSpace getPlanningSpace() const; //Get the current planning space
                
        std::shared_ptr<planning_scene::PlanningScene> getPlanningScene() const; //Get the PlanningSceneInterface
        std::shared_ptr<moveit_visual_tools::MoveItVisualTools> getVisualTools() const; //Get the MoveItVisualTools

        std::vector<double> getNamedTarget(const std::string& target_name);

        void setRobotState(moveit::core::RobotStatePtr& robot_state); //Set the state of the robot (subsequent planning will start from this state)
        moveit::core::RobotStatePtr getRobotState() const; //Get the current state of the robot
        
        const moveit::core::JointModelGroup* getJointModelGroup() const; //Get the joint model group of the planning group

        // --------------- PATH CONSTRAINTS ----------------
        void setPathConstraints(const moveit_msgs::msg::Constraints& position_constraint); //Set constraints for the path
        void clearPathConstraints(); //Clear all constraints
        moveit_msgs::msg::Constraints getPathConstraints() const; //Get the current path constraints

        bool checkJointConstraints(const std::vector<double>& joint_positions); //Check if the joint positions respect the constraints
        bool checkPoseConstraints(const std::vector<double> &joint_positions); //Check if the pose respects the constraints

        // --------------- FORWARD KINEMATICS ----------------
        /* \: computes forward kinematics
            Args:
                joint_positions: Array of joint positions
                end_effector_link: End effector link
            Returns:
                PoseStamped message
        */
        geometry_msgs::msg::PoseStamped getFKine(const std::vector<double>& joint_positions, const std::string &end_effector_link);
        geometry_msgs::msg::PoseStamped getFKine(const std::string &end_effector_link); //Get the current fkine
        geometry_msgs::msg::PoseStamped getFKine(); //Get the current fkine; end_effector_link_ is used as default

        // --------------- INVERSE KINEMATICS ---------------
        /* invKine: computes inverse kinematics
            Args:
                target_pose: Target position
                end_effector_link: End effector link
            Returns:
                Vector of joint positions
        */
        std::vector<double> invKine(const geometry_msgs::msg::Pose &target_pose, const std::string &end_effector_link);
        std::vector<double> invKine(const geometry_msgs::msg::Pose &target_pose); //end_effector_link_ is used as default

        // --------------- JACOBIAN ---------------

        //Get the Jacobian matrix of the manipulator
        const Eigen::MatrixXd getJacobian(const std::vector<double> &joint_positions, const std::string &end_effector_link);
        const Eigen::MatrixXd getJacobian(const std::string &end_effector_link);
        const Eigen::MatrixXd getJacobian(); //end_effector_link_ is used as default

        //Get the pseudo-inverse of the Jacobian matrix
        const Eigen::MatrixXd getPseudoInverseJacobian(const std::vector<double> &joint_positions, const std::string &end_effector_link);
        const Eigen::MatrixXd getPseudoInverseJacobian(const std::string &end_effector_link);
        const Eigen::MatrixXd getPseudoInverseJacobian(); //end_effector_link_ is used as default
        
        // --------------- PUBLIC VARIABLES ----------------
        std::vector<std::string> joints_names_group_;               // Joints group names
        std::vector<double> joints_values_group_;                   // Current joint values of the group
        std::vector<double> joints_speed_group_;                    // Current joint speeds of the group
        bool joints_group_received_ = false;                        // Check if joints group was received

    private:
        // --------------- CALLBACK METHODS ----------------

        void jointsState_callback(const sensor_msgs::msg::JointState::SharedPtr &joints_state);     //Update the joint states
        void trajectoryExecution_callback(); //Update the current trajectory point index
        void executionControl_callback(const std_msgs::msg::Bool::SharedPtr &msg); //Control the execution of the trajectory

        // --------------- TRAJECTORY METHODS ----------------

        void setTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory, const std::string &end_effector_link=""); //Set the planned trajectory

        //Computes the passed trajectory to make waypoints spaced equally in time
        //Returns true if successfull
        bool processTrajectory(moveit_msgs::msg::RobotTrajectory &trajectory); 

        bool checkTrajectory();                             //Check if trajectory_ is still clear of obstacles
        void recalculateTrajectory(size_t start_index);     //Recalculates the trajectory from the point at start_index onwards

        void mergeTrajectory(moveit_msgs::msg::RobotTrajectory &new_traj, size_t start_index); //Merge the old trajectory with the new one from start_index onwards

        bool checkTrajectoryConstraints(const moveit_msgs::msg::RobotTrajectory &trajectory); //Check if the trajectory respects the path constraints
    
        // --------------- PLANNING METHODS ----------------
     
        //Create a motion plan request with the current parameters
        moveit_msgs::msg::MotionPlanRequest createMotionPlanRequest();
        moveit_msgs::msg::MotionPlanRequest createMotionPlanRequest(const std::string &planning_pipeline, const std::string &planner_id); 

        moveit_msgs::msg::Constraints createJointGoalConstraints(const std::vector<double> &joint_positions); //Create goal constraints for joint positions
        moveit_msgs::msg::Constraints createTcpGoalConstraints(const geometry_msgs::msg::Pose &pose, const std::string &ee_link); //Create goal constraints for a pose
    
        moveit_msgs::action::MoveGroup::Result computeMotionPlan(const moveit_msgs::msg::MotionPlanRequest &motion_plan_request); //Call the motion planning service and return the response
        moveit_msgs::msg::MotionSequenceResponse computeMotionSequence(const moveit_msgs::msg::MotionSequenceRequest &motion_plan_request); //Call the cartesian motion sequence service and return the response

        // --------------- HELPER METHODS ----------------
        
        void checkParams(); // Get all the ros parameters and assign them to params_

        bool checkJointDiff(const std::vector<double> &joint_values);                             //Check if the difference between joint_values and current pose is negligible
        bool checkJointDiff(const std::vector<double> &val_a, const std::vector<double> &val_b);  //Check if the difference between val_a and val_b is negligible

        bool checkPoseDiff(const geometry_msgs::msg::Pose &pose, const std::string& ee_link);               //Check if the difference between pose and current pose of ee_link is negligible
        bool checkPoseDiff(const geometry_msgs::msg::Pose &pose_a, const geometry_msgs::msg::Pose &pose_b); //Check if the difference between pose_a and pose_b is negligible
    
        void updatePlannerParams();                                     //Update the planner parameters with values stored in params_
        geometry_msgs::msg::PoseStamped toPoseStamped(const Eigen::Isometry3d& pose, const std::string &frame_id=""); //Converts an Eigen pose to a PoseStamped message

        // --------------- VISUALIZATION ----------------
        //Visualize a primitive
        void visualizePrimitive(const shape_msgs::msg::SolidPrimitive &primitive, 
                                const geometry_msgs::msg::Pose &pose, 
                                const std::vector<double> rgba_color = {0.0, 0.0, 0.0, 0.1},
                                const std::string &ns = "rviz"); 

        //ROS Node
        //NOTE: It's critical for this node to be always spinning!
        rclcpp::Node::SharedPtr node_;

        //Parameters for planning
        DynamicPlannerParams params_;                   //Dynamic planner parameters
        bool dynamic_behavior_;                         //Whether or not the planner should recalculate its paths based on updates of collision objects
        PlanningSpace planning_space_ = JOINTS_SPACE;   //When recalculating trajectories, the space in which the planning is done

        //MoveIt2 interfaces
        std::shared_ptr<planning_scene::PlanningScene> planning_scene_;              //this holds all the information about the world
        std::shared_ptr<moveit_visual_tools::MoveItVisualTools> moveit_visual_tools_;
        rviz_visual_tools::RvizVisualToolsPtr rviz_visual_tools_;

        //Dynamic planner variables
        std::string planning_group_;
        moveit::core::RobotModelConstPtr kinematic_model_;      //this holds all the kinematic information about the robot (eg: links, joints, limits, etc.)
        moveit::core::RobotStatePtr kinematic_state_;           //this holds the current position of the robot at any given time
        moveit_msgs::msg::Constraints path_constraints_;

        //Trajectory variables
        moveit_msgs::msg::RobotTrajectory robot_trajectory_;    //Planned trajectory
        std::vector<double> final_joint_positions_;             //End joint positions of the planned trajectory
        geometry_msgs::msg::Pose final_pose_;                   //End effector pose of the planned trajectory
        std::string traj_end_effector_link_;                    //End effector link of the planned trajectory
        unsigned long trajpoint_;                               //Index of the current trajectory point
        bool is_moving_ = false;                                 //Whether the robot is moving or not, can be set to false to stop execution of trajectory
        bool force_stop_ = false;                                //Force stop the execution of the trajectory

        //Time optimal trajectory generation 
        trajectory_processing::TimeOptimalTrajectoryGenerationPtr time_optimal_traj_gen;
        const double totg_tolerance = 0.1;
        const double totg_min_angle_change = 0.001;

        //Publishers
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_cmd_pub_;                    //Joint state publisher for the fake controller
        rclcpp::Publisher<manipulator_interfaces::msg::TrajectoryResult>::SharedPtr trajectory_pub_;    //Trajectory result publisher see manipulator_interfaces/TrajectoryResult
        
        //Subscribers
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_state_sub_;        //Subscriber for updating the joint states
        rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr trajpoint_sub_;                  //Current trajpoint subscriber
        rclcpp::Subscription<moveit_msgs::msg::RobotTrajectory>::SharedPtr trajectory_sub_;     //Subscriber for trajectories
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr execution_ctrl_sub_;               //Subscriber for moving and stopping the robot
        
        //Service Clients
        rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr plan_action_client_; //Client for motion planning
        rclcpp::Client<moveit_msgs::srv::GetMotionSequence>::SharedPtr cartesian_motion_sequence_client_; //Client for the cartesian motion sequence service

        //Timers
        rclcpp::TimerBase::SharedPtr traj_timer_; //Timer to execute trajectory points

        //Default values for the 'plan' function
        std::string end_effector_link_ = "tool0";
        std::string world_frame_ = "base_link";
};

#endif //DYNAMIC_PLANNER_H
