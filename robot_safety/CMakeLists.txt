cmake_minimum_required(VERSION 3.8)
project(robot_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

add_executable(safety_node src/safety_node.cpp)
ament_target_dependencies(safety_node
  rclcpp std_msgs std_srvs geometry_msgs
  sensor_msgs vision_msgs diagnostic_updater
)

install(TARGETS safety_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
