cmake_minimum_required(VERSION 3.16)
project(jetracer_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

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

# libi2c provides i2c_smbus_* helpers (Debian/Ubuntu: libi2c-dev)
find_library(I2C_LIB i2c)
if(NOT I2C_LIB)
  message(FATAL_ERROR "libi2c not found. Install with: apt install libi2c-dev")
endif()

add_library(jetracer_control SHARED
  src/jetracer_hardware.cpp
)
target_include_directories(jetracer_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/jetracer_control>
)
ament_target_dependencies(jetracer_control PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(jetracer_control PUBLIC ${I2C_LIB})

# Export the hardware plugin to pluginlib
pluginlib_export_plugin_description_file(hardware_interface jetracer_control.xml)

install(
  DIRECTORY include/
  DESTINATION include/jetracer_control
)
install(
  DIRECTORY config launch description
  DESTINATION share/jetracer_control
)
install(TARGETS jetracer_control
  EXPORT export_jetracer_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_jetracer_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
