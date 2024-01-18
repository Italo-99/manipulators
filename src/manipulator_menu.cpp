#include "ros/ros.h"
#include <iostream>
#include "manipulators/ManipulatorPlanner.h"

class MenuHandler
{
public:

  ros::Publisher jointStatePublisher_;

  MenuHandler(ros::Publisher jointStatePublisher)
  {
    jointStatePublisher_ = jointStatePublisher;
  }

  void pubJointGoal()
  {
    std::vector<double> joints = {0.0,-1.57,+1.57,0.0,0.0,0.0};

    sensor_msgs::JointState jointStateMsg;
    jointStateMsg.header.stamp = ros::Time::now();
    jointStateMsg.name.push_back("shoulder_pan_joint");
    jointStateMsg.name.push_back("shoulder_lift_joint");
    jointStateMsg.name.push_back("elbow_joint");
    jointStateMsg.name.push_back("wrist_1_joint");
    jointStateMsg.name.push_back("wrist_2_joint");
    jointStateMsg.name.push_back("wrist_3_joint");
    for (unsigned int k = 0; k < joints.size(); k++) {jointStateMsg.position.push_back(joints[k]); }

    std::cout << "The value of positions size is: " <<jointStateMsg.position.size() << std::endl ;
    std::cout << "The value of names size is: " <<jointStateMsg.name.size() << std::endl;

    // Publish the JointState message
    jointStatePublisher_.publish(jointStateMsg);
  }

  void printMenu()
  {
    std::cout << "======= Menu =======\n";
    std::cout << "1. Option 1\n";
    std::cout << "2. Option 2\n";
    std::cout << "3. Option 3\n";
    std::cout << "4. Exit\n";
    std::cout << "=====================\n";
  }

  int getUserChoice()
  {
    int choice;
    std::cout << "Enter your choice: ";
    std::cin >> choice;
    return choice;
  }

  void processChoice(int choice)
  {
    switch (choice)
    {
    case 1:
      ROS_INFO("You selected Option 1");
      pubJointGoal();
      break;

    case 2:
      ROS_INFO("You selected Option 2");
      // Add your Option 2 logic here
      break;

    case 3:
      ROS_INFO("You selected Option 3");
      // Add your Option 3 logic here
      break;

    case 4:
      ROS_INFO("Exiting...");
      break;

    default:
      ROS_WARN("Invalid choice. Please choose a valid option.");
      break;
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manipulator_menu");
  ros::NodeHandle nh;

  ros::Publisher jstpub = nh.advertise<sensor_msgs::JointState>("/desired_joint_pose", 1); 

  MenuHandler menuHandler(jstpub);

  int userChoice = 0;

  while (userChoice != 4)
  {
    menuHandler.printMenu();
    userChoice = menuHandler.getUserChoice();
    menuHandler.processChoice(userChoice);
    ros::spinOnce();
  }

  return 0;
}
