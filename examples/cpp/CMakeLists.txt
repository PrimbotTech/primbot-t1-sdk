
cmake_minimum_required(VERSION 3.16)
project(aimdk_examples_cpp)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aimdk_msgs REQUIRED)
find_package(OpenCV 4.13.0 EXACT REQUIRED CONFIG COMPONENTS core imgproc imgcodecs videoio)
find_package(ruckig_for_primebot REQUIRED)
set(EXAMPLE_TARGETS
  get_audio_stream
  get_bms_state
  get_jpg
  get_touch_state
  get_video_stream
  joint_control
  mc_locomotion_velocity
  play_audio
  play_emotion
  play_lights
  set_mc_action
  set_preset_motion
  volume_control
  upper_body_control
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
    set_target_properties(${EXAMPLE_TARGET} PROPERTIES
      INSTALL_RPATH "\$ORIGIN/../../../aimdk_opencv/lib;\$ORIGIN/..")
  endif()
  if(EXAMPLE_TARGET STREQUAL "joint_control")
    target_link_libraries(${EXAMPLE_TARGET} ruckig_for_primebot::ruckig_for_primebot)
  endif()
endforeach()
install(TARGETS ${EXAMPLE_TARGETS}
  DESTINATION lib/${PROJECT_NAME})
ament_package()