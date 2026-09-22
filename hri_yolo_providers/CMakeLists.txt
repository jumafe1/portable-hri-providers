cmake_minimum_required(VERSION 3.16)
project(hri_yolo_providers)

find_package(ament_cmake REQUIRED)
find_package(capabilities2_runner REQUIRED)
find_package(hri_capability_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(yolo_msgs REQUIRED)

add_library(yolo_detect_people_runner SHARED src/yolo_detect_people_runner.cpp)
target_compile_features(yolo_detect_people_runner PUBLIC cxx_std_17)
target_compile_options(yolo_detect_people_runner PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  yolo_detect_people_runner
  capabilities2_runner
  hri_capability_interfaces
  pluginlib
  rclcpp
  yolo_msgs
)
target_link_libraries(yolo_detect_people_runner tinyxml2::tinyxml2)

pluginlib_export_plugin_description_file(capabilities2_runner plugins.xml)

install(TARGETS yolo_detect_people_runner LIBRARY DESTINATION lib)
install(
  DIRECTORY capability_providers
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
