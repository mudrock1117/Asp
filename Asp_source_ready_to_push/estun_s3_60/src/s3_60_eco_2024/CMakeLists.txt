cmake_minimum_required(VERSION 3.8)
project(s3_60_eco_2024)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

# Install directories
install(
  DIRECTORY
    launch
    config
    meshes
    urdf
    rviz
    textures
  DESTINATION
    share/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)
ament_package()
