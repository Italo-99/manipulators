cmake_minimum_required(VERSION 2.8.3)
project(manipulators)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  dynamic_planner
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add manipulator class node
add_executable(manipulator_planner src/manipulator_planner.cpp src/manipulator_planner_node.cpp)
target_link_libraries(manipulator_planner ${catkin_LIBRARIES})
add_dependencies(manipulator_planner ${catkin_EXPORTED_TARGETS})

# Add manipulator menu node
add_executable(manipulator_menu src/manipulator_menu.cpp src/manipulator_menu_node.cpp)
target_link_libraries(manipulator_menu ${catkin_LIBRARIES})
add_dependencies(manipulator_menu ${catkin_EXPORTED_TARGETS})
