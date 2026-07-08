cmake_minimum_required(VERSION 3.16)
project(aimdk_examples_cpp)

set(OpenCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../opencv/lib/cmake/opencv4")
set(ruckig_for_primebot_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../ruckig_for_primebot/lib/cmake/ruckig_for_primebot")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aimdk_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core videoio)
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
  endif()

  if(EXAMPLE_TARGET STREQUAL "joint_control")
    target_link_libraries(${EXAMPLE_TARGET} ruckig_for_primebot::ruckig_for_primebot)
  endif()

  if(EXAMPLE_TARGET STREQUAL "get_video_stream" OR
     EXAMPLE_TARGET STREQUAL "joint_control")
    set_target_properties(${EXAMPLE_TARGET} PROPERTIES
      INSTALL_RPATH "$ORIGIN"
      BUILD_WITH_INSTALL_RPATH TRUE
    )
  endif()
endforeach()

install(TARGETS ${EXAMPLE_TARGETS}
  DESTINATION lib/${PROJECT_NAME})


file(GLOB _OPENCV_LIBS
  "${CMAKE_CURRENT_SOURCE_DIR}/../opencv/lib/libopencv_*.so*")
if(_OPENCV_LIBS)
  install(FILES ${_OPENCV_LIBS} DESTINATION lib/${PROJECT_NAME})
endif()

file(GLOB _RUCKIG_LIBS
  "${CMAKE_CURRENT_SOURCE_DIR}/../ruckig_for_primebot/lib/libruckig_*.so*")
if(_RUCKIG_LIBS)
  install(FILES ${_RUCKIG_LIBS} DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
