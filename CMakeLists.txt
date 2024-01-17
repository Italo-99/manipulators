cmake_minimum_required(VERSION 2.8.3)
project(manipulator_planner)

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


add_executable(manipulator_planner src/manipulator_planner.cpp src/manipulator_planner_node.cpp)
target_link_libraries(manipulator_planner ${catkin_LIBRARIES})
add_dependencies(manipulator_planner ${catkin_EXPORTED_TARGETS})
