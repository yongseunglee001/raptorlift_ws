cmake_minimum_required(VERSION 3.8)
project(raptorlift_hardware_bridge)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Try to find libmodbus (optional)
find_package(PkgConfig)
if(PkgConfig_FOUND)
  pkg_check_modules(MODBUS libmodbus)
endif()

# Sources
set(SOURCES src/hardware_bridge.cpp)

if(MODBUS_FOUND)
  message(STATUS "libmodbus found - enabling real hardware support")
  list(APPEND SOURCES src/modbus_driver.cpp)
  add_definitions(-DHAS_MODBUS)
  include_directories(${MODBUS_INCLUDE_DIRS})
  link_directories(${MODBUS_LIBRARY_DIRS})
else()
  message(WARNING "libmodbus NOT found - building simulation-only mode")
  message(WARNING "Install with: sudo apt-get install libmodbus-dev")
endif()

# Include directories
include_directories(include)

# Executable
add_executable(hardware_bridge ${SOURCES})
ament_target_dependencies(hardware_bridge
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

if(MODBUS_FOUND)
  target_link_libraries(hardware_bridge ${MODBUS_LIBRARIES})
endif()

# Install executable
install(TARGETS
  hardware_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install launch and config (if any)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
