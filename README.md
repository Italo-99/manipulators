# Installation

### 1. Pre-requisites
 - Ubuntu 22.04 (LTS)
 - Ros 2 Humble
 - Moveit2 installed [(Guide)](https://moveit.ai/install-moveit2/source/)

### 2. Creating the workspace

    mkdir manipulators_ws
    cd manipulators_ws
    mkdir src
    colcon build

### 3. Downloading the packages

Inside the src folder of your workspace:

    git clone https://github.com/Italo-99/manipulators.git -b ros2-humble
    git clone https://github.com/Italo-99/motors_trajectory.git -b ros2-devel
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git -b humble ur_description
    git clone https://github.com/Projectredunimore/manipulator_interfaces.git

Download the ur drivers repository to wherever you want on your system, then only the ur_moveit_config package will be needed:

    git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver -b humble
    mv Universal_Robots_ROS2_Driver/ur_moveit_config <WORKSPACE_PATH>/src/ur_moveit_config

Do the same with this package and use only the robotiq_85_description package:

    git clone https://github.com/PickNikRobotics/robotiq_85_gripper.git
    mv robotiq_85_gripper/robotiq_85_description <WORKSPACE_PATH>/src/robotiq_85_description

Install the realsense package:

    sudo apt install ros-humble-realsense2-*

Install rviz visual tools:

    sudo apt install ros-humble-rviz-visual-tools

### 4. Fix known issues

1) The URDF files in the ur_description package have some links that are rotated 180 degrees which will make the manipulator work in unexpected ways, to fix this issues go to `ur_description/urdf/ur_macro.xacro` and make the following changes:

    At lines 153, 159, 343:

    ```diff
    - <origin xyz="0 0 0" rpy="0 0 ${pi}"/>

    + <origin xyz="0 0 0" rpy="0 0 0"/>
    ```

2) The default values for the RRTConnect planner are not optimized and make the manipulator move in very strange paths, to fix the issue go to `ur_moveit_config/config/ompl_planning.yaml` and make the following changes:

    At line 33:

    ```diff
    - range: 0.0

    + range: 0.1
    + max_num_iterations: 1000
    + goal_bias: 0.05
    ```

3) Further optimization for planning can be done by using a different kinematic solver and changing joint limits:

    In `ur_moveit_config/config/kinematics.yaml`:

    ```yaml
    /**:
    ros__parameters:
        robot_description_kinematics:
        ur_manipulator:
            kinematics_solver: pick_ik/PickIkPlugin
            kinematics_solver_timeout: 0.05
            kinematics_solver_attempts: 3
            mode: global
            position_scale: 1.0
            rotation_scale: 0.5
            position_threshold: 0.001
            orientation_threshold: 0.01
            cost_threshold: 0.001
            minimal_displacement_weight: 0.0
            gd_step_size: 0.0001
    ```

    In `ur_moveit_config/config/joint_limits.yaml` add the following limits to each joint:

    ```yaml
    min_position: -3.14
    max_position: 3.14
    ```

    Then intall pick_ik solver via apt:

        sudo apt install ros-humble-pick-ik

    More information can be found at [Pick ik kinematics solver](https://moveit.picknik.ai/main/doc/how_to_guides/pick_ik/pick_ik_tutorial.html)

## Useful packages

1) [Install drivers for xbox one controller](https://fostips.com/install-driver-xbox-one-controller-headset-ubuntu/)
2) [ARS Control ur drivers](https://github.com/ARSControl/ur_rtde_controller/tree/humble)

# Use

### Planner Launch

To start the planner use the planner.launch.py launch file:

    ros2 launch manipulators planner.launch.py

To launch the planner with robotiq_85_gripper attached as end effector:

    ros2 launch manipulators planner.launch.py description_path:="<workspace_path>/install/manipulators/share/manipulators/models/urdf/ur_robotiq_85_gripper.urdf.xacro" description_semantic_path:="<workspace_path>/install/manipulators/share/manipulators/models/srdf/ur_robotiq_85_gripper.srdf.xacro" gripper:=True

### Planner parameters

 - `rate`: Rate for the joint_state_publisher node (Hz).
 - `gui`: Whether to run joint_state_publisher with gui or not.
 - `publish_joint_states`: Whether to run joint_state_publisher node.
 - `rviz`: Whether to run rviz or not.
 - `ur_type`: Type of used UR robot (check list below).
 - `tf_prefix`: Prefix for each link and joint of the robot (not applied to joint states passed to manipulator_planner).
 - `rviz_config_path`: Absolute path to rviz config file.
 - `description_package`: Description package with robot URDF/XACRO files.
 - `description_path`: Absolute path to the URDF/XACRO description file of the robot.
 - `description_semantic_path`: Absolute path to the SRDF description file of the robot.
 - `moveit_config_package`: Name of the moveit config package for the robot.
 - `gripper`: Wether to enable gripper or not (supported gripper is robotiq_85_gripper).
 - `joint_limits_file`: Name of the joint limits file. IMPORTANT: This is NOT the joint_limits.yaml file in the moveit config package and should not be mistaken with it.
 - `kinematics_file`: Name of the kinematics file. IMPORTANT: This is NOT the kinematics.yaml file in the moveit_config package and should not be mistaken with it.

**NOTE**: Always use capital letters for `True` and `False` argument.

### The manipulator planner node

The manipulator planner node is used to elaborate trajectories, execute real time control and modify the planning scene. To run the node some parameters are required:

 - `manipulator_name`: A unique identifier for the manipulator.
 - `planning_group`: The planning group (specified in srdf).
 - `joint_names`: List of joint names of the planning group.
 - `ee_name`: Name of the end effector link.
 - `world_frame`: Cartesian point of reference.
 - `ros_freq`: Frequency at which the planner will operate.
 - `max_speed_ee`: Max speed the end effector can move during jacobian control.
 - `max_accel_ee`: Max acceleration the end effector can reach during jacobian control.
 - `max_spd_jnts`: Max speed joints can move during real time joint control.
 - `max_acc_jnts`: Max acceleration joints can reach during real time joint control.
 - `gripper_links`: Links of the gripper to disable their collision with objects attached to the end effector.
 - `position_tolerance`: Tolerance for tcp position.
 - `orientation_tolerance`: Tolerance for tcp orientation.
 - `joint_tolerance`: Tolerance for joint positions.
<br/>

 - `robot_description` : Parsed urdf description of the robot.
 - `robot_description_semantic`: Parsed srdf description of the robot.
 - `robot_description_kinematics`: Path to the kinematics.yaml file (from robot moveit config package).
 - `robot_description_planning`: Path to joint_limits.yaml file (from robot moveit config package).
 - `ompl_planning_pipeline_config`: Path to the ompl_planning.yaml file (from robot moveit config package).

The launch file `planner.launch.py` will automatically retrieve all these parameters. The first group of parameters (until `gripper_links`) is only used by the manipulator planner node, they can be found inside the `config/` directory as .yaml file, one for each `ur_type`. The last five parameters are more like "global" parameters, in the sense they are used by many nodes other than the dynamic planner, they are retrieved by the `get_ur_moveit_config` function inside `manipulators/launch_utils.py` and passed to the appropriate nodes.

### The manipulator menu node

The manipulator menu node can be used to perform different actions with the manipulator.

To run the manipulator menu:

    ros2 run manipulators manipulator_menu_user

### Manipulator menu params

Can be edited in `src/manipulator_menu_node_user.cpp` and  `src/manipulator_menu_node_nouser.cpp`:

 - `node_name`: Name for the manipulator menu node.
 - `ros_freq`: Frequency for the spinner.
 - `manipulator_name`: Name of the manipulator.
 - `planning_group`: Planning group specified in SRDF.
 - `gripper`: Wether to initialize clients for the robotiq_85_gripper.
 - `gripper_group`: Joint group name of the gripper.
 - `joint_names`: List of joint names in the planning_group.
 - `base_link_name`: Name of the base link to use as reference frame.

These parameters must match with the ones passed to manipulator_planner node.

### The joystick control node

The joystick control node can be used to control the manipulator movement in real time, in combination with the manipulator planner node it offers two modes:

1) **Real time joints control**: change the rotation of joints independently.
2) **Jacobian control**: control the speed of the end effector along each axis.

To run the joystick control node run the following two nodes:

    ros2 run joy game_controller_node

and

    ros2 launch manipulators joystick_controller.launch.py

### Joystick control parameters

Can be edited at `/config/joystick/generic.yaml`:

 - `manipulator_name`: Name of the manipulator.
 - `joint_names`: List of joints to be moved by the joystick.
 - `ros_freq`: Frequency for the spinner.
 - `gripper_group`: Joint group name of the gripper.
 - `vel_step`: How fast a movement in the joystick axis will make the end effector move during jacobian control.
 - `rot_step`: How fast a movement in the joystick axis will make the end effector rotate during jacobian control.
 - `js_step`: How fast a movement in the joystick axis will make the joints move during real time joints control.

### Real robot operation

To operate on a real robot first launch the appropriate driver.
In the case of [Ars control ur drivers](https://github.com/ARSControl/ur_rtde_controller/tree/humble) use:

    ros2 launch ur_rtde_controller rtde_controller.launch.py ROBOT_IP:=192.168.xx.xx enable_gripper:=true/false

Then you can launch the planner without joint state publisher as joint state feedback will be provided by real hardware:

    ros2 launch manipulators planner.launch.py publish_joint_states:=False ur_type:=<ur_type> gripper:=True/False

Finally launch the real_control_driver node:

    ros2 launch manipulators real_control_driver.launch.py ur_type:=<ur_type>

The real control driver parameters for each ur type can be found in `config/drivers/`, the parameters are:

 - `velocity_topic`: Where the velocities for joints will be published, depends on the driver used.
 - `joints_names_group`: List of joint names.
 - `kp`: Proportionality constant for acceleration.
 - `spinner_rate`: Frequency for control.
 - `min_motor_speed`: If required velocity is under this value motor will stop.

# Custom implementations

To implement the planner on a custom robot creating a custom launch file is advised, most of the setup will remain the same, what changes is mostly how different files (such as rdf descriptions and moveit configurations) will be retrieved, so you can create a function similar to `get_ur_moveit_params` from `manipulators/launch_utils.py` and maintain the rest of the launch file mostly unchanged.

To implement a custom manipulator menu you can create a node with this structure:

```c++
class MyManipulatorMenu {
    public:
        MyManipulatorMenu(ManipulatorMenuParams params, rclcpp::Node::SharedPtr node) : params_(params), node_(node) {
            //Constructor
            //...
        }

        ~MyManipulatorMenu(){
            delete menu_interface_;
        }

        void spinner(){
            manipulator_menu_->spinner();
        }

        void spinnerMenu()
        {
            // Setup a rate for ROS loop execution
            rclcpp::Rate r(params_.ros_freq);
            initializeMenu();

            std::thread spinner_thread = std::thread([this] {
                spinner();
            });

            while (rclcpp::ok())
            {
                // Display the user menu and process user choices
                menu_interface_->printMenu();
                int choice = menu_interface_->getUserChoice();
                RCLCPP_INFO(node_->get_logger(), "User choice: %d", choice);
                menu_interface_->processChoice(choice);

                // Wait for next loop time
                r.sleep();
            }

            // Shutdown ROS if Ctrl+C or Ctrl+D are pressed
            rclcpp::shutdown();
        }

    private:
        void initializeMenu(){
            manipulator_menu_ = std::make_shared<ManipulatorMenu>(params_, node_);
            menu_interface_ = new MenuUserInterface<MyManipulatorMenu>(this);

            //Custom menu interface implementation
            //...
        }

        rclcpp::Node::SharedPtr node_;
        ManipulatorMenuParams params_;

        std::shared_ptr<ManipulatorMenu> manipulator_menu_;
        MenuUserInterface *menu_interface_;
}

int main(int argc, char* argv[]) {
    ManipulatorMenuParams params;
    //Set custom parameters
    //...

    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(params.node_name);

    auto menu = std::make_shared<MyManipulatorMenu>(params, node);
    menu->spinnerMenu();

    rclcpp::shutdown();

    return 0;
}
```

To implement a custom joystick control node you can simply copy-paste the 'vanilla' JoystickController class and joystick_control_node.cpp, then remap the control scheme by editing the joyCallback method (refer to the two enums `ButtonsMap`, `AxesMap` and [Joy docs](https://index.ros.org/p/joy/) for more informations).


