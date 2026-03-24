cmake_minimum_required(VERSION 3.16)
project(aimdk_examples_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aimdk_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core videoio)
find_package(ruckig REQUIRED)

set(EXAMPLE_TARGETS
  audio_control
  demo
  get_audio_stream
  get_jpg
  get_joint_state
  get_pmu_state
  get_touch_state
  get_video_stream
  joint_control
  mc_locomotion_velocity
  play_audio
  play_emotion
  play_lights
  set_mc_action
  set_preset_motion
)

foreach(EXAMPLE_TARGET ${EXAMPLE_TARGETS})
  set(EXAMPLE_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/src/${EXAMPLE_TARGET}.cpp")
  if(NOT EXISTS "${EXAMPLE_SOURCE}")
    message(FATAL_ERROR "Missing example source: ${EXAMPLE_SOURCE}")
  endif()

  add_executable(${EXAMPLE_TARGET} ${EXAMPLE_SOURCE})
  ament_target_dependencies(${EXAMPLE_TARGET} rclcpp aimdk_msgs)
  if(EXAMPLE_TARGET STREQUAL "get_video_stream")
    target_include_directories(${EXAMPLE_TARGET} PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${EXAMPLE_TARGET} ${OpenCV_LIBS})
  endif()
  if(EXAMPLE_TARGET STREQUAL "joint_control")
    target_link_libraries(${EXAMPLE_TARGET} ruckig::ruckig)
  endif()
endforeach()

install(TARGETS ${EXAMPLE_TARGETS}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
