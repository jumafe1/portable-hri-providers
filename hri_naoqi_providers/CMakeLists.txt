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
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tinyxml2 REQUIRED)

add_library(
  nao_speak_runner SHARED
  src/nao_speak_runner.cpp
  src/naoqi_move_relative_runner.cpp
  src/naoqi_move_velocity_runner.cpp
  src/naoqi_orient_camera_runner.cpp
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
  sensor_msgs
  std_msgs
  trajectory_msgs
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
  add_executable(test_velocity_source_guard test/test_velocity_source_guard.cpp)
  target_include_directories(test_velocity_source_guard PRIVATE src)
  target_compile_features(test_velocity_source_guard PRIVATE cxx_std_17)
  target_compile_options(test_velocity_source_guard PRIVATE -Wall -Wextra -Wpedantic)
  ament_target_dependencies(test_velocity_source_guard rclcpp)
  add_test(NAME velocity_source_guard COMMAND test_velocity_source_guard)
  add_executable(test_velocity_gid test/test_velocity_gid.cpp)
  target_compile_features(test_velocity_gid PRIVATE cxx_std_17)
  target_compile_options(test_velocity_gid PRIVATE -Wall -Wextra -Wpedantic)
  ament_target_dependencies(test_velocity_gid rclcpp geometry_msgs)
  add_executable(test_motion_lease test/test_motion_lease.cpp)
  target_include_directories(test_motion_lease PRIVATE src)
  target_compile_features(test_motion_lease PRIVATE cxx_std_17)
  target_compile_options(test_motion_lease PRIVATE -Wall -Wextra -Wpedantic)
  ament_target_dependencies(test_motion_lease capabilities2_runner rclcpp)
  target_link_libraries(test_motion_lease tinyxml2::tinyxml2)
  add_test(NAME motion_lease COMMAND test_motion_lease)
  add_executable(test_orient_camera test/test_orient_camera.cpp)
  target_include_directories(test_orient_camera PRIVATE src)
  target_compile_features(test_orient_camera PRIVATE cxx_std_17)
  target_compile_options(test_orient_camera PRIVATE -Wall -Wextra -Wpedantic)
  ament_target_dependencies(test_orient_camera capabilities2_runner hri_capability_interfaces
    pluginlib rclcpp sensor_msgs trajectory_msgs)
  target_link_libraries(test_orient_camera tinyxml2::tinyxml2)
  add_test(NAME orient_camera COMMAND test_orient_camera)
  set_tests_properties(orient_camera PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=195;ROS_LOCALHOST_ONLY=1" TIMEOUT 60)
endif()

ament_package()
