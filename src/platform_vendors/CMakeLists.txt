cmake_minimum_required(VERSION 3.5)
project(formula_vendors)

find_package(ament_cmake REQUIRED)

# Wskazujemy CMake, aby wszedł do folderu z paczką lidarową i ją zbudował
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ldlidar_stl_ros2/CMakeLists.txt")
  add_subdirectory(third_party/ldlidar_stl_ros2)
else()
  message(FATAL_ERROR "Lidar config not found")
endif()


ament_package()