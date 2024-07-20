# MANIPULATORS ROS PKG
This repository contains some useful ROS1 files to handle all comercial manipulators based on the dynamic planner class, carried out as research fellow activity Italo Almirante within the Ars Control Laboratory, with the collaboration of the professor Cristian Secchi and the post-doctoral researcher Andrea Pupa.

# Installation guide

Install packages:

    pip install coppeliasim_zmqremoteapi_client

    sudo apt install ros-noetic-chomp-motion-planner
    sudo apt install ros-noetic-moveit
    sudo apt install ros-noetic-moveit-collision-detection
    sudo apt install ros-noetic-moveit-planning-scene
    sudo apt install ros-noetic-moveit-planning-scene-interface
    sudo apt install ros-noetic-moveit-planning-scene-monitor
    sudo apt install ros-noetic-moveit-robot-model
    sudo apt install ros-noetic-moveit-robot-state
    sudo apt install ros-noetic-moveit-robot-trajectory
    sudo apt install ros-noetic-moveit-transforms
    sudo apt install ros-noetic-moveit-visual-tools
    sudo apt install ros-noetic-industrial-trajectory-filters

To avoid MoveIt! setup errors, run the following commands:

    sudo apt update
    cd your_ws/
    catkin clean
    catkin build