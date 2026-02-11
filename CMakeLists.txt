cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0048 NEW)
project(manipulators)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Comment these lines when debugging
# They should reduce memory usage during compilation 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-var-tracking-assignments")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-depth=256")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(manipulator_interfaces REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(graph_msgs REQUIRED)

find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(motors_trajectory REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(detection_interfaces REQUIRED)

# ---------------- VARIABLES -----------------

# Dependencies for libraries

set(MANIPULATOR_MENU_DEPENDS
    rclcpp
    geometry_msgs
    sensor_msgs
    std_msgs
    std_srvs
    trajectory_msgs
    moveit_msgs
    motors_trajectory
    yaml_cpp_vendor
    manipulator_interfaces
    detection_interfaces
    tf2_ros
    tf2_geometry_msgs
)

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

#Add directories to be installed
install(DIRECTORY
  config
  launch
  models
  DESTINATION share/${PROJECT_NAME}
)

##Install include directory
install(
    DIRECTORY include/
    DESTINATION include
)

#Set include dir
include_directories(include)

# Export include for use of in other packages
ament_export_include_directories(include)

# ---------------- LAUNCH UTILS -----------------

ament_python_install_package(${PROJECT_NAME})
ament_python_install_module(${PROJECT_NAME}/launch_utils.py)

# ---------------- MANIPULATOR PLANNER -----------------

add_executable(manipulator_planner src/manipulator_planner.cpp src/manipulator_planner_node.cpp src/dynamic_planner.cpp)

ament_target_dependencies(manipulator_planner
    rclcpp
    geometry_msgs
    sensor_msgs
    std_msgs
    trajectory_msgs
    moveit_core
    moveit_msgs
    graph_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    rviz_visual_tools
    manipulator_interfaces
)

target_link_libraries(manipulator_planner Eigen3::Eigen)

install(
    TARGETS manipulator_planner
    DESTINATION lib/${PROJECT_NAME}
)

# ---------------- MANIPULATOR MENU -----------------


add_executable(manipulator_menu_user src/manipulator_menu.cpp src/manipulator_menu_node_user.cpp)

ament_target_dependencies(manipulator_menu_user
    ${MANIPULATOR_MENU_DEPENDS}
)


target_link_libraries(manipulator_menu_user Eigen3::Eigen)

install(
    TARGETS manipulator_menu_user
    DESTINATION lib/${PROJECT_NAME}
)

#Export menu user interface library
add_library(menu_user_interface INTERFACE)
ament_export_targets(menu_user_interface)

install(
    TARGETS menu_user_interface
    EXPORT menu_user_interface
    INCLUDES DESTINATION include
)

#Export menu library
add_library(manipulator_menu SHARED src/manipulator_menu.cpp)
ament_export_targets(manipulator_menu)

target_link_libraries(manipulator_menu Eigen3::Eigen)

install(
    TARGETS manipulator_menu
    EXPORT manipulator_menu
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

ament_target_dependencies(manipulator_menu
    ${MANIPULATOR_MENU_DEPENDS}
)

# ---------------- DRIVER TRAJECTORY CONVERTER -----------------

add_executable(driver_trajectory_converter src/driver_trajectory_converter.cpp src/driver_trajectory_converter_node.cpp)

ament_target_dependencies(driver_trajectory_converter
    rclcpp
    std_msgs
    sensor_msgs
)

target_link_libraries(driver_trajectory_converter Eigen3::Eigen)

install(
    TARGETS driver_trajectory_converter
    DESTINATION lib/${PROJECT_NAME}
)

# ---------------- JOYSTICK CONTROLLER -----------------

add_executable(joystick_controller_node src/joystick_controller.cpp src/joystick_controller_node.cpp src/joystick_controller_mappings.cpp src/manipulator_menu.cpp)

ament_target_dependencies(joystick_controller_node
    ${MANIPULATOR_MENU_DEPENDS}
)


target_link_libraries(joystick_controller_node Eigen3::Eigen)

install(
    TARGETS joystick_controller_node
    DESTINATION lib/${PROJECT_NAME}
)

#Export library
add_library(joystick_controller SHARED src/joystick_controller.cpp)
ament_export_targets(joystick_controller)

install(
    TARGETS joystick_controller
    EXPORT joystick_controller
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

ament_target_dependencies(joystick_controller
    ${MANIPULATOR_MENU_DEPENDS}
)

# ------------------------------------------------------------

ament_export_dependencies(${MANIPULATOR_MENU_DEPENDS})

ament_package()
