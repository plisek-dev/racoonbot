cmake_minimum_required(VERSION 3.16)
project(racoonbot_pkg LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Find required dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add pigpio dependency (manual find using find_library)
find_library(PIGPIO_LIB pigpio)
if (NOT PIGPIO_LIB)
  message(FATAL_ERROR "PIGPIO library not found")
endif()

# COMPILE
add_library(
  racoonbot_pkg
  SHARED
  hardware/racoonbot_system.cpp
)

target_compile_features(racoonbot_pkg PUBLIC cxx_std_17)
target_include_directories(racoonbot_pkg PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/racoonbot_pkg>
)

ament_target_dependencies(
  racoonbot_pkg PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the DLL but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "racoonbot_pkg_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface racoonbot_pkg.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/racoonbot_pkg
)
install(
  DIRECTORY description/ros2_control description/urdf description/rviz description/meshes
  DESTINATION share/racoonbot_pkg
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/racoonbot_pkg
)
install(TARGETS racoonbot_pkg
  EXPORT export_racoonbot_pkg
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Link pigpio library
target_link_libraries(racoonbot_pkg PRIVATE ${PIGPIO_LIB})

# EXPORTS
ament_export_targets(export_racoonbot_pkg HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
