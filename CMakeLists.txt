cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(manipulators)

add_compile_options(-std=c++11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(catkin REQUIRED COMPONENTS
  dynamic_planner
  geometry_msgs
  gripper
  message_generation
  roscpp
  rospy
  std_msgs
)

add_service_files(
  FILES
  CoppeliaMenu.srv
  InvKine.srv
  Jacobian.srv
  FKine.srv
  PseudoInverse.srv
  ChangePlannerParams.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include srv
  LIBRARIES manipulator_menu
  ${THIS_PACKAGE_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
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

# Add driver_trajectory_converter library
add_library(driver_trajectory_converter src/driver_trajectory_converter.cpp)
target_link_libraries(driver_trajectory_converter ${catkin_LIBRARIES})
add_dependencies(driver_trajectory_converter ${catkin_EXPORTED_TARGETS})

# Add driver_trajectory_converter_node
add_executable(driver_trajectory_converter_node src/driver_trajectory_converter_node.cpp)
target_link_libraries(driver_trajectory_converter_node driver_trajectory_converter ${catkin_LIBRARIES})
add_dependencies(driver_trajectory_converter_node ${catkin_EXPORTED_TARGETS})

# Add joystick_controller library
add_library(joystick_controller src/joystick_controller.cpp)
target_link_libraries(joystick_controller manipulator_menu ${catkin_LIBRARIES})
add_dependencies(manipulator_menu ${catkin_EXPORTED_TARGETS})

# Add joy_arm_controller node
add_executable(joy_arm_controller src/joystick_controller.cpp)
target_link_libraries(joy_arm_controller joystick_controller ${catkin_LIBRARIES})
add_dependencies(joy_arm_controller ${catkin_EXPORTED_TARGETS})