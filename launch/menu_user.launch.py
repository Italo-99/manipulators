from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package="manipulators",
        executable="manipulator_menu_user",
        name="manipulator_menu_user",
        output="screen",
        parameters=[{
            "manipulator_name": "manipulator",
            "rate" : 500,
            "joint_names" : ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        }]
    )

    return LaunchDescription([
        node
    ])