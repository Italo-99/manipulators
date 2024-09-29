cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(manipulators)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_planner
  geometry_msgs
  gripper
  message_generation
  roscpp
  rospy
  std_srvs
  std_msgs
)

add_service_files(
  FILES
  CoppeliaMenu.srv
  InvKine.srv
  Jacobian.srv
  FKine.srv
  PseudoInverse.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include srv
  LIBRARIES manipulator_menu
  ${THIS_PACKAGE_INCLUDE_DIRS}
)

include_directories(
  include
  srv
  ${catkin_INCLUDE_DIRS}
)

# Add manipulator class node
add_executable(manipulator_planner src/manipulator_planner.cpp src/manipulator_planner_node.cpp)
target_link_libraries(manipulator_planner ${catkin_LIBRARIES})
add_dependencies(manipulator_planner ${catkin_EXPORTED_TARGETS})

# Add manipulator_menu library
add_library(manipulator_menu src/manipulator_menu.cpp)
target_link_libraries(manipulator_menu ${catkin_LIBRARIES})
add_dependencies(manipulator_menu ${catkin_EXPORTED_TARGETS})

# Add manipulator menu node user
add_executable(manipulator_menu_node_user src/manipulator_menu_node_user.cpp)
target_link_libraries(manipulator_menu_node_user manipulator_menu ${catkin_LIBRARIES})
add_dependencies(manipulator_menu_node_user ${catkin_EXPORTED_TARGETS})

# Add manipulator menu node no user
add_executable(manipulator_menu_node_Nouser src/manipulator_menu_node_Nouser.cpp)
target_link_libraries(manipulator_menu_node_Nouser manipulator_menu ${catkin_LIBRARIES})
add_dependencies(manipulator_menu_node_Nouser ${catkin_EXPORTED_TARGETS})