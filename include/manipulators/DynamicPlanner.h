#ifndef DYNAMIC_PLANNER_H
#define DYNAMIC_PLANNER_H

//C++ Imports
#include <string>
#include <vector>
#include <Eigen/Geometry>

//ROS Imports
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <tf2/convert.h>

//MoveIt2 Imports
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

// Struct definition of the parameters of the Dynamic Planner
struct DynamicPlannerParams
{
    std::string planner_id = "RRTConnect"; // name of the planner method (look up from list in ompl_planning.yaml)
    int num_attempts       = 2;            // max number of attempts to find a trajectory
    double planning_time   = 2;            // maximum planning time in seconds (default 0: no limit)
    double vel_factor      = 1.;           // velocity factor
    double acc_factor      = 1.;           // acceleration factor
    double sample_time     = 0.002;        // sample time for cartesian planner
    double max_velocity    = 0.5;          // maximum ee velocity for cartesian planner
    double tolerance       = 0.01;         // tolerance for goal position

    // V1: empty struct
    DynamicPlannerParams() {}

    // V2: passing args as initializers
    DynamicPlannerParams(const std::string& planner_id, const int attempts,
                         const double time, const double v_factor, const double a_factor,
                         const double time_step, const double max_vel)
        : planner_id(planner_id), 
          num_attempts(attempts), 
          planning_time(time), 
          vel_factor(v_factor),
          acc_factor(a_factor),
          sample_time(time_step),
          max_velocity(max_vel)
    {}
};

class DynamicPlanner
{
    public:
        DynamicPlanner(const rclcpp::Node::SharedPtr& node,
                       const std::string& planning_group,
                       const double vel_factor = 0.2,
                       const double acc_factor = 0.2,
                       bool dynamic_behavior = true);

        void initialize(); //Initialize the dynamic planner (vars, subscribers, publishers, ...)
        void spinner(); 

        // --------------- CONTROL METHODS ---------------
        
        enum PlanningSpace : bool
        {
            JOINTS_SPACE = 0,
            OPERATIVE_SPACE = 1
        };

        /*plan: joint goal
            Args:
                joint_positions: Array of target joint positions
        */
        void plan(const std::vector<double> joint_positions);

        /*plan: pose goal
            Args:
                goal_pose: Target position
                ee_link: End effector link
                frame: Reference frame
            
            Overloads use default values specified in the class definition
        */

        void plan(const geometry_msgs::msg::Pose& goal_pose, const std::string& ee_link, const std::string& frame);
        void plan(const geometry_msgs::msg::Pose& goal_pose, const std::string& ee_link);
        void plan(const geometry_msgs::msg::Pose& goal_pose);

        void moveRobot(const trajectory_msgs::msg::JointTrajectoryPoint &traj_pt); //Single trajectory point
        void moveRobot(); //Execute the last planned trajectory
        void moveRobot(moveit_msgs::msg::RobotTrajectory& robot_trajectory); //Execute the passed trajectory (waypoints)

        bool isMoving(); //Check if the robot is moving

        void stop(); //Stop the execution of the planned trajectory

        // --------------- GETTERS AND SETTERS ----------------

        // Dynamic planner parameters getter and setter
        DynamicPlannerParams getParams() const;

        void setParams(const std::string& planner_id, const int attempts, const double time,
                       const double v_factor, const double a_factor);
                       
        void setParams(const DynamicPlannerParams& params);

        void setDynamicBehavior(bool dynamic_behavior); //Set the dynamic_behavior_ variable
        bool isDynamic() const; //Check if dynamic_behaviour_ is true

        void setPlanningSpace(PlanningSpace space); //Set the planning space (joint or operative)
        PlanningSpace getPlanningSpace() const; //Get the current planning space
                
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> getMoveGroup() const; //Get the MoveGroupInterface
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> getPlanningScene() const; //Get the PlanningSceneInterface
        std::shared_ptr<moveit_visual_tools::MoveItVisualTools> getVisualTools() const; //Get the MoveItVisualTools

        std::vector<double> getNamedTarget(const std::string& target_name);

        void setRobotState(moveit::core::RobotStatePtr& robot_state); //Set the state of the robot (subsequent planning will start from this state)
        moveit::core::RobotStatePtr getRobotState(); //Get the current state of the robot

        // --------------- PATH CONSTRAINTS ----------------
        void setPathConstraints(const moveit_msgs::msg::Constraints& position_constraint); //Set constraints for the path
        void clearPathConstraints(); //Clear all constraints
        moveit_msgs::msg::Constraints getPathConstraints() const; //Get the current path constraints

        // --------------- FORWARD KINEMATICS ----------------
        /* getFKine: computes forward kinematics
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
        const Eigen::MatrixXd getJacobian(const std::string &end_effector_link);
        const Eigen::MatrixXd getJacobian(); //end_effector_link_ is used as default

        //Get the pseudo-inverse of the Jacobian matrix
        const Eigen::MatrixXd getPseudoInverseJacobian(const std::string &end_effector_link);
        const Eigen::MatrixXd getPseudoInverseJacobian(); //end_effector_link_ is used as default


    private:
        void setTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory, const std::string &end_effector_link=""); //Set the planned trajectory

        bool checkTrajectory(); //Check if trajectory_ is still clear of obstacles
        void recalculateTrajectory(size_t start_index); //Recalculates the trajectory from the point at start_index onwards

        void mergeTrajectory(moveit_msgs::msg::RobotTrajectory &new_traj, size_t start_index); //Merge the old trajectory with the new one from start_index onwards

        // --------------- HELPER FUNCTIONS ----------------
        void updatePlannerParams(); //Update the planner parameters with values stored in params_
        geometry_msgs::msg::PoseStamped toPoseStamped(const Eigen::Isometry3d& pose, const std::string &frame_id=""); //Converts an Eigen pose to a PoseStamped message

        //ROS Node
        //NOTE: It's critical for this node to be always spinning!
        rclcpp::Node::SharedPtr node_;

        //Parameters for planning
        DynamicPlannerParams params_; //Dynamic planner parameters
        bool dynamic_behavior_; //Whether or not the planner should recalculate its paths based on updates of collision objects
        PlanningSpace planning_space_ = JOINTS_SPACE; //When recalculating trajectories, the space in which the planning is done

        //MoveIt2 interfaces
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
        std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
        std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

        //Robot model
        robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
        moveit::core::RobotModelPtr robot_model_;

        //Dynamic planner variables
        std::string planning_group_;
        std::vector<std::string> joint_names_;

        //Trajectory variables
        moveit_msgs::msg::RobotTrajectory robot_trajectory_;
        std::vector<double> final_joint_positions_; //End joint positions of the planned trajectory
        geometry_msgs::msg::Pose final_pose_; //End effector pose of the planned trajectory
        std::string traj_end_effector_link_; //End effector link of the planned trajectory
        size_t traj_current_index_ = 0; //Current index of the trajectory being executed
        bool is_moving = false;

        //Publishers
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_; //Joint state publisher for the fake controller
        
        //Default values for the 'plan' function
        std::string end_effector_link_ = "tool0";
        std::string world_frame_ = "world";
};

#endif //DYNAMIC_PLANNER_H