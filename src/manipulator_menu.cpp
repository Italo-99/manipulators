#include "ros/ros.h"
#include <iostream>

class MenuHandler
{
public:
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
      // Add your Option 1 logic here
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
  ros::init(argc, argv, "menu_node");
  ros::NodeHandle nh;

  MenuHandler menuHandler;

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
