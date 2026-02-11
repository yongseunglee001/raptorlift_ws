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

# Include directories
include_directories(include)

# Try to find libmodbus (optional)
if(MODBUS_FOUND)
  message(STATUS "libmodbus found - enabling real hardware support")

  # Build modbus_driver as a shared library (reusable by raptorlift_hardware)
  add_library(raptorlift_modbus_driver SHARED
    src/modbus_driver.cpp
  )
  target_include_directories(raptorlift_modbus_driver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${MODBUS_INCLUDE_DIRS}
  )
  target_link_libraries(raptorlift_modbus_driver ${MODBUS_LIBRARIES})
  ament_target_dependencies(raptorlift_modbus_driver rclcpp)
  target_compile_definitions(raptorlift_modbus_driver PUBLIC HAS_MODBUS)

  # Install the modbus driver library
  install(TARGETS raptorlift_modbus_driver
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
else()
  message(WARNING "libmodbus NOT found - building simulation-only mode")
  message(WARNING "Install with: sudo apt-get install libmodbus-dev")
endif()

# Executable
add_executable(hardware_bridge src/hardware_bridge.cpp)
ament_target_dependencies(hardware_bridge
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

if(MODBUS_FOUND)
  target_link_libraries(hardware_bridge raptorlift_modbus_driver)
  target_compile_definitions(hardware_bridge PRIVATE HAS_MODBUS)
else()
  target_include_directories(hardware_bridge PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
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

# Export library for other packages
if(MODBUS_FOUND)
  ament_export_include_directories(include)
  ament_export_libraries(raptorlift_modbus_driver)
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_dependencies(rclcpp)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
