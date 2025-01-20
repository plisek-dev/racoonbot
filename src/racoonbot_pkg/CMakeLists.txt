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

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## Compile
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

target_compile_definitions(racoonbot_pkg PRIVATE "racoonbot_pkg_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface racoonbot_pkg.xml)

# Install headers
install(
  DIRECTORY hardware/include/
  DESTINATION include/racoonbot_pkg
)

# Install other resources
install(
  DIRECTORY description/ros2_control description/urdf description/rviz description/meshes
  DESTINATION share/racoonbot_pkg
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/racoonbot_pkg
)

# Install library
install(TARGETS racoonbot_pkg
  EXPORT export_racoonbot_pkg
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Link to RPi.GPIO if required
find_library(WIRINGPI_LIB wiringPi)
if (WIRINGPI_LIB)
  target_link_libraries(racoonbot_pkg PRIVATE ${WIRINGPI_LIB})
else()
  message(FATAL_ERROR "WiringPi library not found")
endif()

## Export targets and dependencies
ament_export_targets(export_racoonbot_pkg HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
