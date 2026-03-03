#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "deadwheel_msgs::deadwheel_msgs__rosidl_generator_py" for configuration "Debug"
set_property(TARGET deadwheel_msgs::deadwheel_msgs__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(deadwheel_msgs::deadwheel_msgs__rosidl_generator_py PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_DEBUG "deadwheel_msgs::deadwheel_msgs__rosidl_generator_c;Python3::Python;deadwheel_msgs::deadwheel_msgs__rosidl_typesupport_c;std_msgs::std_msgs__rosidl_generator_c;std_msgs::std_msgs__rosidl_typesupport_fastrtps_c;std_msgs::std_msgs__rosidl_typesupport_fastrtps_cpp;std_msgs::std_msgs__rosidl_typesupport_introspection_c;std_msgs::std_msgs__rosidl_typesupport_c;std_msgs::std_msgs__rosidl_typesupport_introspection_cpp;std_msgs::std_msgs__rosidl_typesupport_cpp;std_msgs::std_msgs__rosidl_generator_py;builtin_interfaces::builtin_interfaces__rosidl_generator_c;builtin_interfaces::builtin_interfaces__rosidl_typesupport_fastrtps_c;builtin_interfaces::builtin_interfaces__rosidl_typesupport_introspection_c;builtin_interfaces::builtin_interfaces__rosidl_typesupport_c;builtin_interfaces::builtin_interfaces__rosidl_typesupport_fastrtps_cpp;builtin_interfaces::builtin_interfaces__rosidl_typesupport_introspection_cpp;builtin_interfaces::builtin_interfaces__rosidl_typesupport_cpp;builtin_interfaces::builtin_interfaces__rosidl_generator_py"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libdeadwheel_msgs__rosidl_generator_py.so"
  IMPORTED_SONAME_DEBUG "libdeadwheel_msgs__rosidl_generator_py.so"
  )

list(APPEND _cmake_import_check_targets deadwheel_msgs::deadwheel_msgs__rosidl_generator_py )
list(APPEND _cmake_import_check_files_for_deadwheel_msgs::deadwheel_msgs__rosidl_generator_py "${_IMPORT_PREFIX}/lib/libdeadwheel_msgs__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
