# Installation

### 1. Pre-requisites
 - Ubuntu 22.04 (LTS)
 - Ros 2 Humble
 - Moveit2 installed [(Guide)](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)

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

### 4. Fix known issues

1) The URDF files in the ur_description package have some links that are rotated 180 degrees which will make the manipulator work in unexpected ways, to fix this issues go to ur_description/urdf/ur_macro.xacro and make the following changes:

    At lines 153, 159, 343:

    ```diff
    - <origin xyz="0 0 0" rpy="0 0 ${pi}"/>

    + <origin xyz="0 0 0" rpy="0 0 0"/>
    ```

2) The default values for the RRTConnect planner are not optimized and make the manipulator move in very unoptimized paths, to fix the issue go to ur_moveit_config/config/ompl_planning.yaml and make the following changes:

    At line 33:

    ```diff
    - range: 0.0

    + range: 0.1
    + max_num_iterations: 1000
    + goal_bias: 0.05
    ```

## Other

1) [Install drivers for xbox one controller](https://fostips.com/install-driver-xbox-one-controller-headset-ubuntu/)

# Use

### The manipulator planner node

The manipulator planner node is used to elaborate trajectories, execute real time control and modify the planning scene. To run the node some parameters are required:

 - `manipulator_name`: A unique identifier for the manipulator.
 - `planning_group`: The planning group (specified in srdf).
 - `joint_names`: List of joint names of the planning group.
 - `ee_name`: Name of the end effector link.
 - `base_link`: Name of the base link of the robot.
 - `world_frame`: Cartesian point of reference.
 - `ros_freq`: Frequency at which the planner will operate.
 - `max_speed_ee`: Max speed the end effector can move during jacobian control.
 - `max_accel_ee`: Max acceleration the end effector can reach during jacobian control.
 - `max_spd_jnts`: Max speed joints can move during real time joint control.
 - `max_acc_jnts`: Max acceleration joints can reach during real time joint control.
 - `gripper_links`: Links of the gripper to disable their collision with objects attached to the end effector.
 - `robot_description` : Parsed urdf description of the robot.
 - `robot_description_semantic`: Parsed srdf description of the robot.
 - `robot_description_kinematics`: Path to the kinematics.yaml file (from robot moveit config package).
 - `robot_description_planning`: Path to joint_limits.yaml file (from robot moveit config package).
 - `ompl_planning_pipeline_config`: Path to the ompl_planning.yaml file (from robot moveit config package).

To get the last 5 arguments which all refer either to the description or the moveit config packages you can use the method `get_ur_moveit_launch_params` from `manipulators/launch_utils.py`.

To start the planner use the planner.launch.py launch file:

    ros2 launch manipulators planner.launch.py

To launch with the robotiq85 gripper attached to the manipulator use:

    ros2 launch manipulators planner.launch.py description_path:="<workspace_path>/install/manipulators/share/manipulators/models/urdf/ur_robotiq_85_gripper.urdf.xacro" description_semantic_path:="<workspace_path>/install/manipulators/share/manipulators/models/srdf/ur_robotiq_85_gripper.srdf.xacro" gripper:=True

### The manipulator menu node

The manipulator menu node can be used to perform different actions with the manipulator.

To run the manipulator menu:

    ros2 run manipulators manipulator_menu_user

### The joystick control node

The joystick control node can be used to control the manipulator movement in real time, in combination with the manipulator planner node it offers two modes:

1) **Real time joints control**: change the rotation of joints independently.
2) **Jacobian control**: control the speed of the end effector along each axis.

To run the joystick control node run the following two nodes:

    ros2 run joy game_controller_node

and

    ros2 run manipulators joystick_controler_node

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

To implement a custom joystick control node you can simply copy-paste the 'vanilla' JoystickController class and joystick_control_node.cpp, then change the ManipulatorMenu instance with your custom menu.
To remap the control scheme edit the joyCallback method (use the two enums ButtonsMap and AxesMap and [Joy docs](https://index.ros.org/p/joy/) for more informations on how joystick controls are mapped).


