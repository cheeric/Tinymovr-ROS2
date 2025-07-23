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
# GENERATE VISIBILITY CONTROL HEADER
#============================================================================
# This generates the "visibility_control.h" header required for exporting
# library symbols correctly on different platforms.
ament_generate_export_header(${PROJECT_NAME})

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

# Specify include directories for the library target.
# This includes the project's 'include' folder and the build folder for generated headers.
target_include_directories(${PROJECT_NAME}_control_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
)

# Link the library against its dependencies.
ament_target_dependencies(${PROJECT_NAME}_control_interface
  rclcpp
  hardware_interface
  pluginlib
)

# Export the plugin description file for ros2_control to discover this library.
ros2_control_export_plugin_description_file(${PROJECT_NAME}_control_interface hardware_interface)

#============================================================================
# BUILD EXECUTABLE TARGET
#============================================================================
# Build the standalone node that hosts the controller manager.
add_executable(control_node src/control_node_main.cpp)

# Link the executable against its dependencies.
ament_target_dependencies(control_node
  rclcpp
)

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

# Install the standalone executable node.
install(TARGETS control_node
  DESTINATION lib/${PROJECT_NAME}
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