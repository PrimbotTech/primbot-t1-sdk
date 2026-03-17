cmake_minimum_required(VERSION 3.16)
project(aimdk_examples_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aimdk_msgs REQUIRED)

file(GLOB EXAMPLE_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

set(EXAMPLE_TARGETS)
foreach(EXAMPLE_SOURCE ${EXAMPLE_SOURCES})
  get_filename_component(EXAMPLE_TARGET ${EXAMPLE_SOURCE} NAME_WE)

  file(READ "${EXAMPLE_SOURCE}" SOURCE_CONTENT)
  string(FIND "${SOURCE_CONTENT}" "int main(" MAIN_POS)
  if(MAIN_POS EQUAL -1)
    message(WARNING "Skip ${EXAMPLE_SOURCE}: no main() entry point.")
    continue()
  endif()

  add_executable(${EXAMPLE_TARGET} ${EXAMPLE_SOURCE})
  ament_target_dependencies(${EXAMPLE_TARGET} rclcpp aimdk_msgs)
  list(APPEND EXAMPLE_TARGETS ${EXAMPLE_TARGET})
endforeach()

install(TARGETS ${EXAMPLE_TARGETS}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
