Some useful instructions for new ROS2 use.

1) To rebuild the ws
 
    rm -rf build install log
    colcon build

2) To rebuild a single pkg

    rm -rf build/<package_name> install/<package_name>
    colcon build --packages-select <package_name>

3) To install ROS2 drivers for UR

    sudo apt install ros-humble-ur

4) To convert xacro to urdf

    ros2 run xacro xacro ur.urdf.xacro  ur_type:=ur10e name:=ur10e_robot > ur10e.urdf

5) To install moveit:

    sudo apt ros-humble-moveit*

6) UR manipulators lib issues:

    overriding needed (colcon build --allow-overriding ur_description ur_moveit_config)
    Why CollisionObject is both an srv and a msg

7) ros2 topic pub --once /move_group/fake_controller_joint_states sensor_msgs/msg/JointState '{"header": {}, "name": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
  "position": [1.0, 0.5, -0.5, 1.2, -0.8, 0.3],
  "velocity": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "effort": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}'

8) To launch planning_context with sirio manipulator:

    ros2 launch manipulators planning_context.launch.py manipulator_type:='sirio' description_package:="sirio_manipulator" moveit_config_package:="sirio_manipulator_moveit_config" 
    description_path:="<path_to_ws>/install/sirio_manipulator/share/sirio_manipulator/models/urdf/sirio.xacro" 
    description_semantic_file:="sirio.srdf" 
    rviz_config_path:="<path_to_ws>/install/sirio_manipulator/share/sirio_manipulator/config/rviz/view_sirio.rviz"

9) To test sirio movement:

    ros2 topic pub --once /move_group/fake_controller_joint_states sensor_msgs/msg/JointState "{header: {}, name: ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"], 
    position: [0.0, -0.6, -0.35, 0.0, -0.45, 0.0]}"

## Setup TODO
   
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



    