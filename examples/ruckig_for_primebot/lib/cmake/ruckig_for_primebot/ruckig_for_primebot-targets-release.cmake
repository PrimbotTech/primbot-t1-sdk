#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ruckig_for_primebot::ruckig_for_primebot" for configuration "Release"
set_property(TARGET ruckig_for_primebot::ruckig_for_primebot APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ruckig_for_primebot::ruckig_for_primebot PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libruckig_for_primebot.so"
  IMPORTED_SONAME_RELEASE "libruckig_for_primebot.so"
  )

list(APPEND _cmake_import_check_targets ruckig_for_primebot::ruckig_for_primebot )
list(APPEND _cmake_import_check_files_for_ruckig_for_primebot::ruckig_for_primebot "${_IMPORT_PREFIX}/lib/libruckig_for_primebot.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
