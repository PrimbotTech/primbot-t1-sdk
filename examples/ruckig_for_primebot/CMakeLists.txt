cmake_minimum_required(VERSION 3.15)

project(ruckig_for_primebot VERSION 0.15.5 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

include(GNUInstallDirs)

option(BUILD_PYTHON_MODULE "Build Python wrapper with nanobind" ON)
option(BUILD_SHARED_LIBS "Build as shared library" ON)

# --- Core Library ---
add_library(ruckig_for_primebot
  src/ruckig/brake.cpp
  src/ruckig/position_first_step1.cpp
  src/ruckig/position_first_step2.cpp
  src/ruckig/position_second_step1.cpp
  src/ruckig/position_second_step2.cpp
  src/ruckig/position_third_step1.cpp
  src/ruckig/position_third_step2.cpp
  src/ruckig/velocity_second_step1.cpp
  src/ruckig/velocity_second_step2.cpp
  src/ruckig/velocity_third_step1.cpp
  src/ruckig/velocity_third_step2.cpp
)

target_compile_features(ruckig_for_primebot PUBLIC cxx_std_17)
target_include_directories(ruckig_for_primebot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(ruckig_for_primebot PUBLIC)

if(MSVC)
  target_compile_definitions(ruckig_for_primebot PUBLIC _USE_MATH_DEFINES)
endif()

# Alias for internal use within workspace
add_library(ruckig_for_primebot::ruckig_for_primebot ALIAS ruckig_for_primebot)


# --- Python Module ---
if(BUILD_PYTHON_MODULE)
  find_package(Python 3.8 REQUIRED COMPONENTS Interpreter Development.Module)
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
  find_package(nanobind CONFIG REQUIRED HINTS ${nanobind_ROOT})

  nanobind_add_module(python_ruckig NB_STATIC src/wrapper/python.cpp)
  target_compile_features(python_ruckig PUBLIC cxx_std_17)
  target_link_libraries(python_ruckig PUBLIC ruckig_for_primebot)

  set_target_properties(python_ruckig PROPERTIES OUTPUT_NAME ruckig_for_primebot)
  set_target_properties(python_ruckig PROPERTIES ARCHIVE_OUTPUT_NAME python_ruckig)

  install(TARGETS python_ruckig LIBRARY DESTINATION .)

  # --- AUTOMATION: Copy the compiled module to the python examples directory ---
  add_custom_command(TARGET python_ruckig POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    $<TARGET_FILE:python_ruckig>
    ${CMAKE_CURRENT_SOURCE_DIR}/../python/
    COMMENT "Deploying ruckig_for_primebot module to python examples folder..."
  )
endif()


# --- Installation ---
include(CMakePackageConfigHelpers)

# Install headers
install(DIRECTORY include/ruckig DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Install library
install(TARGETS ruckig_for_primebot
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install CMake config files for finding this package
set(ruckig_INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

install(EXPORT ${PROJECT_NAME}-targets
  DESTINATION ${ruckig_INSTALL_CONFIGDIR}
  NAMESPACE ${PROJECT_NAME}::
)

configure_package_config_file(cmake/${PROJECT_NAME}-config.cmake.in ${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION ${ruckig_INSTALL_CONFIGDIR}
)

write_basic_package_version_file(${PROJECT_NAME}-config-version.cmake
  VERSION ${ruckig_VERSION}
  COMPATIBILITY AnyNewerVersion
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION ${ruckig_INSTALL_CONFIGDIR}
)

install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/package.xml"
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}
)

ament_package()
