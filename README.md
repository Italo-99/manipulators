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
