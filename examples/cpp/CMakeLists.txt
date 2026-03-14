cmake_minimum_required(VERSION 3.16)
project(aimdk_examples_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aimdk_msgs REQUIRED)

add_executable(demo src/demo.cpp)
ament_target_dependencies(demo rclcpp aimdk_msgs)

install(TARGETS
  demo
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
