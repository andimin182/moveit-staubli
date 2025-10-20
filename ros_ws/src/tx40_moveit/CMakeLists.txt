cmake_minimum_required(VERSION 3.8)
project(tx40_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tx40_msgs REQUIRED)
# find_package(moveit_task_constructor_core REQUIRED)

# Include headers
include_directories(
  include
)

add_executable(move_robot src/plan.cpp src/moveit_robot.cpp)
add_executable(plan_server_action src/plan_server_action.cpp)
#add_executable(mtc_node src/mtc_node.cpp)
ament_target_dependencies(move_robot rclcpp moveit_ros_planning_interface geometry_msgs moveit_visual_tools)
ament_target_dependencies(plan_server_action tx40_msgs rclcpp rclcpp_action moveit_ros_planning_interface moveit_visual_tools geometry_msgs)
#ament_target_dependencies(mtc_node rclcpp moveit_task_constructor_core moveit_ros_planning_interface moveit_visual_tools geometry_msgs)

install(
  TARGETS move_robot plan_server_action
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
