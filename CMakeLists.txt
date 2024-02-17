cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(manipulators)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_srvs
  std_msgs
  geometry_msgs
  dynamic_planner
  message_generation
)

include_directories(
  include
  srv
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
  FILES
  CoppeliaMenu.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include srv
)

# Add manipulator class node
add_executable(manipulator_planner src/manipulator_planner.cpp src/manipulator_planner_node.cpp)
target_link_libraries(manipulator_planner ${catkin_LIBRARIES})
add_dependencies(manipulator_planner ${catkin_EXPORTED_TARGETS})

# Add manipulator menu node
add_executable(manipulator_menu src/manipulator_menu.cpp src/manipulator_menu_node.cpp)
target_link_libraries(manipulator_menu ${catkin_LIBRARIES})
add_dependencies(manipulator_menu ${catkin_EXPORTED_TARGETS})
