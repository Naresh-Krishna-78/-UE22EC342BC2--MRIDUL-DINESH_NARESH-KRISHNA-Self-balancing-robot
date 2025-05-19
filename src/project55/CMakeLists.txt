cmake_minimum_required(VERSION 3.8)
project(project55)

# Find necessary ROS 2 and other packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(xacro REQUIRED)

# Add the executable for your C++ controller node
add_executable(balance_controller src/balance_controller.cpp)

# Link libraries to your executable
ament_target_dependencies(balance_controller
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# Install the executable
install(TARGETS
  balance_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install world files
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files (if any - we created the dir but aren't using it yet)
# install(DIRECTORY
#   config
#   DESTINATION share/${PROJECT_NAME}/
# )

# Check for linters
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Standard ROS 2 cleanup
ament_package()
