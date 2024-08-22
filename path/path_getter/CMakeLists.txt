cmake_minimum_required(VERSION 3.8)
project(path_getter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(simple_path_interface REQUIRED)

ament_auto_find_build_dependencies()


ament_auto_add_executable(path_get_node
  src/path_get_node.cpp
)

ament_target_dependencies(path_get_node
  rclcpp
  autoware_auto_planning_msgs
  simple_path_interface
)

ament_auto_add_executable(path_save_node
  src/path_save_node.cpp
)

ament_target_dependencies(path_save_node
  rclcpp
  simple_path_interface
)

install(TARGETS
  path_get_node
  path_save_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
