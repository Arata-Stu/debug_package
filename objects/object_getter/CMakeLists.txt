cmake_minimum_required(VERSION 3.8)
project(object_getter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ai_object_interface REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(object_info_node
  src/object_getter_node.cpp
)

ament_target_dependencies(object_info_node rclcpp std_msgs geometry_msgs ai_object_interface)


ament_auto_add_executable(position_record_node
  src/position_recorder_node.cpp
)


ament_target_dependencies(position_record_node rclcpp  ai_object_interface)

ament_auto_add_executable(object_save_node
  src/object_save_node.cpp
)


ament_target_dependencies(object_save_node rclcpp  ai_object_interface)


install(TARGETS
  object_info_node
  object_save_node
  position_record_node
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package()
