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

8) To launch planner with robotiq85 gripper attached:
    ros2 launch manipulators planner.launch.py description_path:="/home/matteo/projectred_ws/install/manipulators/share/manipulators/models/urdf/ur_robotiq_85_gripper.urdf.xacro" description_semantic_path:="/home/matteo/projectred_ws/install/manipulators/share/manipulators/models/srdf/ur_robotiq_85_gripper.srdf.xacro"

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