// NODE FILE TO LAUNCH A MANIPULATOR PLANNER INSTANCE

// IMPORT LIBRARIES
#include "manipulator_planner/ManipulatorPlanner.h"

// MAIN FUNCTION: this is a node
int main(int argc, char** argv)
{

  // Initialize node
  ros::init(argc, argv, "manipulator_planner");

  // Istantiate an object of the class ManipulatorPlanner
  ManipulatorPlanner ce;

  // Setup a rate for ROS loop execution
  ros::Rate r(200);

  // ROS loop
  while (ros::ok())
  {
    // Call the spinner for object related fuctions
    ce.spinner();

    // Wait for next loop time
    r.sleep();
  }

  // FILE END
  return 0;
}
