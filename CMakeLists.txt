cmake_minimum_required(VERSION 3.8)
project(tinymovr_ros2)

#============================================================================
# FIND DEPENDENCIES
#============================================================================
# Find necessary ROS2 packages as declared in package.xml.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control_cmake REQUIRED)

#============================================================================
# DEFINE SOURCE FILES
#============================================================================
# Define a variable with all source files for the hardware interface library
# to keep the add_library command clean.
set(TINYMOVR_LIB_SOURCES
  src/tinymovr/can.cpp
  src/tinymovr/comms.cpp
  src/tinymovr/commutation_sensor.cpp
  src/tinymovr/controller.cpp
  src/tinymovr/current.cpp
  src/tinymovr/external_spi.cpp
  src/tinymovr/hall.cpp
  src/tinymovr/homing.cpp
  src/tinymovr/motor.cpp
  src/tinymovr/onboard.cpp
  src/tinymovr/position_sensor.cpp
  src/tinymovr/position.cpp
  src/tinymovr/scheduler.cpp
  src/tinymovr/select.cpp
  src/tinymovr/setup.cpp
  src/tinymovr/stall_detect.cpp
  src/tinymovr/tinymovr.cpp
  src/tinymovr/traj_planner.cpp
  src/tinymovr/user_frame.cpp
  src/tinymovr/velocity.cpp
  src/tinymovr/voltage.cpp
  src/tinymovr/watchdog.cpp
  src/socketcan_cpp/socketcan_cpp.cpp
  src/tinymovr_hardware.cpp
  )

#============================================================================
# BUILD PLUGIN LIBRARY TARGET
#============================================================================
# Build the hardware interface as a shared library so it can be loaded as a plugin.
add_library(${PROJECT_NAME}_control_interface SHARED
  ${TINYMOVR_LIB_SOURCES}
)

# This definition is used by the visibility_control.hpp header
# to correctly export symbols from the shared library.
target_compile_definitions(${PROJECT_NAME}_control_interface
  PRIVATE "TINYMOVR_ROS2_BUILDING_LIBRARY"
)

# Specify include directories for the library target.
target_include_directories(${PROJECT_NAME}_control_interface
  PUBLIC
    # Paths for downstream packages that use this library
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    # Paths for compiling this library's own source files
    ${CMAKE_CURRENT_SOURCE_DIR}/include/tinymovr
    ${CMAKE_CURRENT_SOURCE_DIR}/include/socketcan_cpp
)

# Link the library against its dependencies.
ament_target_dependencies(${PROJECT_NAME}_control_interface
  rclcpp
  hardware_interface
  pluginlib
)

# Export the plugin description file for pluginlib to discover this library.
pluginlib_export_plugin_description_file(hardware_interface examples/config/tinymovr_ros2_control.xml)

#============================================================================
# INSTALLATION RULES
#============================================================================
# These rules are mandatory for ROS2 to find and use the built targets and files.

# Install the shared library plugin.
install(TARGETS ${PROJECT_NAME}_control_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install all header files from the 'include' directory.
install(DIRECTORY include/
  DESTINATION include
)

#============================================================================
# INSTALL EXAMPLE FILES
# Install the example configurations and launch files so they can be used.
#============================================================================
install(DIRECTORY examples/description/
  DESTINATION share/${PROJECT_NAME}/description
)
install(DIRECTORY examples/config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY examples/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

#============================================================================
# FINAL PACKAGE COMMAND
#============================================================================
# This must be the last command in the file. It finalizes the package build process.
ament_package()