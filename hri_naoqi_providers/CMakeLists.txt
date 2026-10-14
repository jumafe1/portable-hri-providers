cmake_minimum_required(VERSION 3.16)
project(hri_naoqi_providers)

find_package(ament_cmake REQUIRED)
find_package(capabilities2_runner REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hri_capability_interfaces REQUIRED)
find_package(naoqi_utilities_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tinyxml2 REQUIRED)

add_library(
  nao_speak_runner SHARED
  src/nao_speak_runner.cpp
  src/naoqi_move_relative_runner.cpp
  src/naoqi_move_velocity_runner.cpp
)
target_compile_features(nao_speak_runner PUBLIC cxx_std_17)
target_compile_options(nao_speak_runner PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  nao_speak_runner
  capabilities2_runner
  geometry_msgs
  hri_capability_interfaces
  naoqi_utilities_msgs
  nav_msgs
  pluginlib
  rclcpp
  std_msgs
)
target_link_libraries(nao_speak_runner tinyxml2::tinyxml2)

pluginlib_export_plugin_description_file(capabilities2_runner plugins.xml)

install(TARGETS nao_speak_runner LIBRARY DESTINATION lib)
install(
  DIRECTORY capability_providers
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_motion_lease test/test_motion_lease.cpp)
  target_include_directories(test_motion_lease PRIVATE src)
  target_compile_features(test_motion_lease PRIVATE cxx_std_17)
  target_compile_options(test_motion_lease PRIVATE -Wall -Wextra -Wpedantic)
  ament_target_dependencies(test_motion_lease capabilities2_runner rclcpp)
  target_link_libraries(test_motion_lease tinyxml2::tinyxml2)
  add_test(NAME motion_lease COMMAND test_motion_lease)
endif()

ament_package()
