# Manipulators

[[Ars Control Lab page]](https://www.arscontrol.unimore.it/)
[[Italo Almirante]](https://www.arscontrol.unimore.it/italo-almirante/)
[[Andrea Pupa]](https://www.arscontrol.unimore.it/andrea-pupa/)

This repository is a universal package to plan and execute trajectory or move commands for all manipulators. It handles several comercial manipulators, but it is easy to use for custom robot configurations.

It is conceived for beginners who wants to include robots in their projects, without a deep knowledge of manipulator kinematics and dynamics.

This document explains how to use this pkg for simulations (even included into CoppeliaSim environment) and for the real hardware.

### Commercial manipulators supported

Universal robots (UR):

    ur3
    ur3e    
    ur5
    ur5e
    ur10
    ur10e
    ur16e
    ur20
    ur30

Other robots will be included in the future.

## Getting started

### Basic principles of the pkg

Each manipulator can plan a move in the joint and in the operative space thanks to some kinematics calculation already included in the pkg libraries. The default planning chaing goes from 'base_link' to 'tcp_gripper', which offsets from the robot's flange.

Autonomous and teleoperated routines are available to users.

### Installation

Prerequisites: UBUNTU 20.04 - ROS NOETICS. It's also adviced to create a ssh key to your GitHub account. 

Create your catkin workspace:

    sudo apt update
    mkdir -p catkin_ws/src
    cd catkin_ws/
    catkin build

Install ros packages:

    sudo apt install ros-noetic-chomp-motion-planner
    sudo apt install ros-noetic-geometric-shapes
    sudo apt install ros-noetic-moveit*
    sudo apt install ros-noetic-industrial-trajectory-filters
    sudo apt install ros-noetic-rviz
    sudo apt install ros-noetic-trac-ik*
    sudo apt install ros-noetic-ur-robot-driver

You can also use coppelia interface to run dynamic simulations. Coppelia instructions and manuals available [[here]](https://manual.coppeliarobotics.com/index.html). Then, execute the following commands (it's not mandatory for compiling):

    pip install coppeliasim_zmqremoteapi_client
    pip install xmlschema
    sudo apt install xsltproc
    cd catkin_ws/src
    git clone https://github.com/CoppeliaRobotics/simROS.git
    cd simROS
    git checkout coppeliasim-v4.6.0-rev0 (check coppelia version)
    cd ..
    cd ..
    export COPPELIASIM_ROOT_DIR=<your_directory_to_VREP>

If you want to use cameras mounted on the robot, you have to install the following pkgs (it's not mandatory for compiling):

    sudo apt install ros-noetic-realsense2-camera
    sudo apt install ros-noetic-realsense2-description

If you want to use a gripper mounted on the robot, you can install the following pkg to handle the robotiq-85-gripper (it's not mandatory for compiling):

    cd src
    git clone https://github.com/a-price/robotiq_arg85_description.git

Now, you can download the Universal Robot pkg for ROS, and a out custom code implementation of its drivers and IO:

    cd src
    git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git
    git clone https://github.com/ARSControl/ur_rtde_controller.git

Finally, you can download the libraries used by the repo:

    cd src
    git clone -b almirante/devel git@github.com:apupa/dynamic_planner.git
    git clone git@github.com:Italo-99/gripper.git
    git clone git@github.com:Italo-99/manipulators.git

Compile the pkg (it's suggested to compile one pkg at the time to avoid compiler bugs):

    cd catkin_ws/
    rosdep install --from-paths src --ignore-src -y
    catkin build <pkg_name>

or alternatively:

    catkin build

## How to guide

### Planner launch

You can launch the simulation of a planner of the desired robot, using a file in the folder "launch/robots", for example:

    roslaunch manipulators ur10e_planner.launch

If you want to run the real robot interface, you should run in a terminal the robot driver, for example for the UR manipulators:

    roslaunch ur_rtde_controller rtde_controller.launch ROBOT_IP:=<your_robot_ip> enable_gripper:=true/false

and run the robots launch file with the following args:

    roslaunch manipulators ur10e_planner.launch real_control:=true pub_js_state:=false

The main differences between the simulation and the hardware code are two: the joint state publisher node is launched only in sim, as the driver publishes motor's positions and speeds; the topic of the real command to the motor's driver is enabled or disabled.

### Planner parameters

You can read more about planner args in the configuration files .yaml in the folder "config".
You can read more about real drivers args in the configuration files .yaml in the folder "config/drivers".

### Coppelia start

To launch coppelia simulation, ensure that you scene is in the folder "scenes" and run:

    roslaunch manipulators coppelia_server.launch scene_name:=<file_name_of_your_scene>

You can open the desired scene directly within the software interface. Two example scenes are in the folder "scenes".

### Model parameters

In all robot launch files, there are lots of args (accurately commented) to handle simulation as you like. Nevertheless, due to the complexity of pkg integration, it's not suggested to change this file, but only to change some params values.

For example, the following args are set as default, but you can change their values (true or false, or string or floats) to insert a camera, a gripper or to change the tcp position. Offsets are in metres and rotations in radians

    name="camera"        value="false"      
    name="cam_x_offset"  value="0.0"
    name="cam_y_offset"  value="0.0"
    name="cam_z_offset"  value="0.0"
    name="cam_rx"        value="0.0"      
    name="cam_ry"        value="0.0"      
    name="cam_rz"        value="0.0"      
    name="cam_parent"    value="tool0"  
    name="ee"            value="false"          
    name="ee_parent"     value="tool0"   
    name="ee_z_offset"   value="0.0" 
    name="ee_rz"         value="0.0"       
    name="tcp_offset"    value="0.0"

### Control rate param

The arg which the planner is mostly sensitive to is "rate", which is the frequency of moveit state publishing, as well as the control rate of drivers and the sampling time of the trajectories and commands to the robot. By default, it is set to 500 Hz, which grants excellent control performances. If you need to change it, you can write the following command (for example):

    roslaunch manipulators ur10e_planner.launch rate:=100
 

### Interface menu node

The code which lets you to interact with the robot is the commands menu, implemente in the class <ManipulatorMenu>. To run it, execute the following command:

    rosrun manipulators manipulator_menu_node_user

which prints all the available functions to handle robots moves and test the functions you need. Each menu item calls a function within the library, and you can choose the action you need by reading the documentation (which are comments, as now) in the library "src/manipulator_menu.cpp".

Within the file "src/manipulator_menu_node_user.cpp", you can modify the list of the params of the interface as below:

    ManipulatorMenuParams params;
    params.node_name              = "manipulator_menu_node_user";
    std::string ee_joint_name     = "";
    params.ros_freq               = 10.;
    params.manipulator_name       = "manipulator";
    bool enable_coppelia          = false;
    bool enable_sim_gripper       = false;
    bool enable_real_gripper      = false;
    std::string gripper_topic     = "";

By modifying the above values, you can set a node name, a joint name for the ee, the frequency of commands publishing, the name of the manipultor (it must correspond to the name of your robot group in the .srdf file), enable coppelia, real or sim gripper (by providing the real topic/srv name).

## How to customize your robot control

If you have your own robot and you need to control it, you can create moveit config pkg using the [[moveit setup assistant]](http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html). The name of this pkg must be <your_robot_name>_moveit_config. It will have all planning configuration files which "manipulators" library uses to compute and execute trajectories.

To adapt the commands menu of the node "src/manipulator_menu_node_user.cpp" file, just change the name of the param <manipulator_name>.

You can use "launch/planner.launch" as file to include to launch your own robot, paying attention to the args to pass. Here below, the values of some params are suggested:
   
    <!-- Planner -->
    <arg name="robot_name"          default="your_robot_name" />
    <arg name="robotpkg_name"       default="$(find <your_robot_name>_moveit_config)" />
    <arg name="custom_pkg"          default="true" />
    <arg name="custom_plan_file"    default="true"  />
    <arg name="planning_group_file" default="$(find your_pkg)/config/$(arg robot_name).yaml" />
    <!-- Simulation -->
    <arg name="rviz"                default="true"  />
    <arg name="def_config"          default="false" />
    <arg name="debug"               default="false" />
    <arg name="sensor_manager"      default="false" />
    <!-- Whether publish joint state from moveit -->
    <arg name="pub_js_state"        default="true"/>
    <arg name="rate"                default="500"/>
    <!-- Choose the sampling duration of the planner -->
    <arg name="sample_duration"     default="0.002"/>

    <!-- Custom models -->
    <arg name="default_desc"  default="false" />
    <arg name="xacro_file"    default="$(find your_pkg)/models/urdf/$(arg robot_name).xacro"/>
    <arg name="srdf_file"     default="$(find your_pkg)/models/srdf/$(arg robot_name).srdf"/>

    <!-- If true, launch the trajectory converter node -->
    <arg name="real_control"        default ="false" />
    <arg name="drivers_config_file" default="$(find your_pkg)/config/drivers/$(arg robot_name).yaml"/>
  
To ensure that the whole config is compliant with the pkg structure, check the <planning_group_file>, the <drivers_config_file>, the <xacro_file> and the <srdf_file>. You can look at similar files in the library. It's extremely IMPORTANT to check the following setup:

1) in your srdf file, the group name must correspond to the manipulator_name in each yaml and launch file;
2) in your srdf file, comment the group declaration in each <group_state> declaration;
3) in your srdf file, comment/uncomment the collisions settings between the robot and the gripper to enable/disable collisions check.
4) in your srdf file, enable the chain as ' chain base_link="base_link" tip_link="tcp_gripper" ';
5) check that joints names are coherent with the model in every config file.

Once your planner is launched, you can create your own instance of the class <ManipulatorMenu>. You can create your class in your source (.cpp) file as below:

    YourManipulator::YourManipulator(const ManipulatorMenuParams& params) 
    {
    // Declaration of manipulator menu
    params_ = params;
    manipulator_menu_ = new ManipulatorMenu(params_);
    }

and pass the params through the node as below:

    #include "your_pkg/YourManipulator.h"

    int main(int argc, char** argv)
    {
        // Node name
        std::string node_name = "your_manipulator_node";
        
        // Initialize the ROS node
        ros::init(argc, argv, node_name);

        // Initialize ManipulatorMenuParams structure
        ManipulatorMenuParams params;
        params.node_name            = node_name;
        params.ros_freq             = 10;
        params.ee_joint_name        = "";
        params.manipulator_name     = "your_manipulator_name";
        params.enable_coppelia      = false;
        params.enable_sim_gripper   = false;
        params.enable_real_gripper  = false;
        params.gripper_topic        = "";

        // Instantiate the YourManipulator object with params
        YourManipulator manipulator(params);

        // Call the spinner for object related functions
        manipulator.spinner();

        // Call the spinner for menu commands display
        // manipulator.spinnerMenu();

        return 0;
    }

### Optimized compiler

If you want to speed your code running performances, you can leave the following lines in the "CMakeLists".txt of this package:

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

Or alternatively, you can change them with:

    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

Your code should run faster, despite the longer compiler time needed. By default, O3 is set. You can maually comment it if you don't want to use this functionality, but it's strongly suggested.

## Disclaimer

### Known Issues

Although you have installed all the required pkgs, it may happen that you cannot compile the ws at this first time. Just source the ws and try the compile again.

### Future developments

A new version with ROS2 Humble interface is on development. Contact us for collabs.

### Issues

Please if you have any issue in compiling the nodes or using them, you can open an issue on git or send me an email at my email address: italo.almirante@unimore.it.