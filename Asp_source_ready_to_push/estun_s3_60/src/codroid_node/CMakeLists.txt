cmake_minimum_required(VERSION 3.8)
project(codroid_node)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(codroid_msgs REQUIRED)
find_package(robotiq_interfaces REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/FK.py
  scripts/FK2.py
  scripts/FK3.py
  scripts/codroid_io.py
  scripts/codroid_move_testpp.py
  scripts/codroid_move_test.py
  scripts/codroid_control.py
  scripts/codroidtestmove.py
  scripts/MotionPlanner.py
  scripts/estun_mdh_fk.py
  scripts/v16_preflight.py
  scripts/M1.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights.
  # uncomment the line when a copyright and license is added to all source files.
  #set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips the linter which checks for common typos.
  # uncomment the line when a copyright and license is added to all source files.
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
