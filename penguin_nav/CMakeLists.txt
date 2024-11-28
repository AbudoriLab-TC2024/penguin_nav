cmake_minimum_required(VERSION 3.8)
project(penguin_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})

ament_auto_add_executable(waypoint_obstacle_avoidance
  src/main.cpp
  src/util.cpp
  src/waypoint_obstacle_avoidance.cpp)

target_compile_definitions(waypoint_obstacle_avoidance
  PRIVATE "DOCTEST_CONFIG_DISABLE")

install(PROGRAMS
  penguin_nav/follow_path.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  ament_auto_add_executable(test_executable
    src/waypoint_obstacle_avoidance.cpp
    src/util.cpp
    test/test_main.cpp)

  ament_add_test(test_doctest
    COMMAND $<TARGET_FILE:test_executable> --reporters=junit --out=${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_doctest.xml
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
endif()

ament_auto_package()
