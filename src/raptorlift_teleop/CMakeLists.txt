cmake_minimum_required(VERSION 3.8)
project(raptorlift_teleop)

# C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Executables
add_executable(gamepad_teleop src/gamepad_teleop.cpp)
ament_target_dependencies(gamepad_teleop
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

add_executable(velocity_scaler src/velocity_scaler.cpp)
ament_target_dependencies(velocity_scaler
  rclcpp
  geometry_msgs
  std_msgs
)

# Install executables
install(TARGETS
  gamepad_teleop
  velocity_scaler
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
