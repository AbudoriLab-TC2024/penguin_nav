cmake_minimum_required(VERSION 3.8)
project(penguin_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})

ament_auto_add_executable(pose_to_tf src/pose_to_tf.cpp)

ament_auto_add_executable(tf_to_pose src/tf_to_pose.cpp)

install(PROGRAMS
  penguin_nav/follow_path.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
