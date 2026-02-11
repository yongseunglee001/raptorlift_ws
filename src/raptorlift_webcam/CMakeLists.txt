cmake_minimum_required(VERSION 3.8)
project(raptorlift_webcam)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/detection_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
