cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0048 NEW)
project(manipulators)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# Find dependencies
find_package(moveit_common REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(manipulator_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add the "time test" executable and link libraries and dependencies
# add_executable(time_uniform_sampling_test src/time_uniform_sampling_test.cpp)
# ament_target_dependencies(time_uniform_sampling_test
#   rclcpp
#   geometry_msgs
#   sensor_msgs
#   std_msgs
#   trajectory_msgs
#   moveit_core
#   moveit_msgs
#   moveit_ros_planning
#   moveit_ros_planning_interface
#   moveit_visual_tools
# )

# install(TARGETS time_uniform_sampling_test
#   DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

# add_library(dynamic_planner SHARED src/dynamic_planner.cpp)
# ament_export_targets(dynamic_planner)

# ament_target_dependencies(dynamic_planner
#     moveit_core
#     moveit_msgs
#     moveit_ros_planning
#     moveit_ros_planning_interface
#     moveit_visual_tools
#     geometry_msgs
#     trajectory_msgs
#     rclcpp
# )

# install(
#     TARGETS dynamic_planner
#     EXPORT dynamic_planner
#     LIBRARY DESTINATION lib
#     ARCHIVE DESTINATION lib
#     RUNTIME DESTINATION bin
#     INCLUDES DESTINATION include
# )

# install(TARGETS manipulator_planner
#   DESTINATION lib/${PROJECT_NAME}
# )

add_executable(manipulator_planner src/manipulator_planner.cpp src/manipulator_planner_node.cpp src/dynamic_planner.cpp)

ament_target_dependencies(manipulator_planner
    rclcpp
    geometry_msgs
    sensor_msgs
    std_msgs
    trajectory_msgs
    moveit_core
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    manipulator_interfaces
)

install(
    TARGETS manipulator_planner
    DESTINATION lib/${PROJECT_NAME}
)

# Create the pkg with the above setup
ament_package()