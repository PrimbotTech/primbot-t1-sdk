cmake_minimum_required(VERSION 3.16)
project(aimdk_msgs VERSION 1.0.0)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# 收集所有 msg / srv / action 文件
file(GLOB_RECURSE msg_files    "*.msg")
file(GLOB_RECURSE srv_files    "*.srv")
file(GLOB_RECURSE action_files "*.action")

set(normalized_interface_base "${CMAKE_CURRENT_BINARY_DIR}/rosidl_interfaces")
set(interface_files)

foreach(f ${msg_files})
  get_filename_component(name "${f}" NAME)
  set(dst_rel "msg/${name}")
  configure_file("${f}" "${normalized_interface_base}/${dst_rel}" COPYONLY)
  list(APPEND interface_files "${normalized_interface_base}:${dst_rel}")
endforeach()

foreach(f ${srv_files})
  get_filename_component(name "${f}" NAME)
  set(dst_rel "srv/${name}")
  configure_file("${f}" "${normalized_interface_base}/${dst_rel}" COPYONLY)
  list(APPEND interface_files "${normalized_interface_base}:${dst_rel}")
endforeach()

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${interface_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()