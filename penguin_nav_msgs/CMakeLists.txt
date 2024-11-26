cmake_minimum_required(VERSION 3.8)
project(penguin_nav_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AdjustWaypoints.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)


ament_package()
