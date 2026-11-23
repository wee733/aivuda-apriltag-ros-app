cmake_minimum_required(VERSION 3.16)
project(tag_scale_filter LANGUAGES C CXX)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(isaac_ros_apriltag_interfaces REQUIRED)
find_package(tf2_msgs REQUIRED)

add_library(tag_scale_filter SHARED src/tag_scale_filter.cpp)
target_compile_features(tag_scale_filter PRIVATE cxx_std_17)
target_link_libraries(tag_scale_filter PRIVATE
  rclcpp::rclcpp rclcpp_components::component
  ${isaac_ros_apriltag_interfaces_TARGETS} ${tf2_msgs_TARGETS})
rclcpp_components_register_nodes(tag_scale_filter "aivuda::TagScaleFilterNode")
install(TARGETS tag_scale_filter LIBRARY DESTINATION lib)

add_executable(watch_detector src/watch_detector.c)
install(TARGETS watch_detector DESTINATION lib/${PROJECT_NAME})
ament_package()
