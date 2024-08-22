cmake_minimum_required(VERSION 3.8)
project(control_getter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(control_get_node
  src/control_get_node.cpp
)

install(TARGETS
  control_get_node
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package()
