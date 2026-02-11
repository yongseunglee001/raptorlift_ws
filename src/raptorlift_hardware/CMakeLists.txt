cmake_minimum_required(VERSION 3.8)
project(raptorlift_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(raptorlift_hardware_bridge REQUIRED)

# Optional: libmodbus for real hardware support
find_package(PkgConfig)
if(PkgConfig_FOUND)
  pkg_check_modules(MODBUS libmodbus)
endif()

# Build the hardware interface library
add_library(${PROJECT_NAME} SHARED
  src/raptorlift_hardware.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  raptorlift_hardware_bridge
)

# Link modbus driver library if available
if(MODBUS_FOUND)
  message(STATUS "libmodbus found - enabling real hardware support in ros2_control plugin")
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_MODBUS)
  target_include_directories(${PROJECT_NAME} PRIVATE ${MODBUS_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${MODBUS_LIBRARIES})
else()
  message(WARNING "libmodbus NOT found - ros2_control plugin will be simulation-only")
endif()

# Visibility control
target_compile_definitions(${PROJECT_NAME} PRIVATE "RAPTORLIFT_HARDWARE_BUILDING_DLL")

# Export plugin description
pluginlib_export_plugin_description_file(hardware_interface raptorlift_hardware.xml)

# Install
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  raptorlift_hardware_bridge
)

ament_package()
