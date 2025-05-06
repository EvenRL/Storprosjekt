cmake_minimum_required(VERSION 3.8)
project(robot_controller_pkg)

# Enable strict compiler warnings for better code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Define the executable for the MoveIt controller
add_executable(moveit_controller src/moveit_controller.cpp)

# Include directories for the target
target_include_directories(moveit_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Set compile features (C99 and C++17)
target_compile_features(moveit_controller PUBLIC c_std_99 cxx_std_17)

# Link dependencies to the executable
ament_target_dependencies(
  moveit_controller
  rclcpp
  moveit_ros_planning_interface
)

# Install the executable
install(TARGETS moveit_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Build testing (if enabled)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright and cpplint checks (remove these lines when ready for a production repository)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # Automatically find and add test dependencies
  ament_lint_auto_find_test_dependencies()
endif()

# Mark the package for installation
ament_package()
