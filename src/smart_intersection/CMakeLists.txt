cmake_minimum_required(VERSION 3.8)
project(smart_intersection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)

install(DIRECTORY
  launch
  models
  worlds
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/smart_traffic_controller.py
  scripts/emergency_detector.py
  scripts/vehicle_spawner.py
  scripts/vehicle_controller.py
  scripts/vehicle_counter.py
  scripts/test_spawner.py
  scripts/simulation_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
