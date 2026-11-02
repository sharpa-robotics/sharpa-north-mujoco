#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "SharpaMujoco::Core" for configuration "RelWithDebInfo"
set_property(TARGET SharpaMujoco::Core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(SharpaMujoco::Core PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELWITHDEBINFO "CXX"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/avatar_mujoco_core.lib"
  )

list(APPEND _cmake_import_check_targets SharpaMujoco::Core )
list(APPEND _cmake_import_check_files_for_SharpaMujoco::Core "${_IMPORT_PREFIX}/lib/avatar_mujoco_core.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
