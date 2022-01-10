cmake_minimum_required(VERSION 3.5)
project(replay_motion)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kuka_sunrise REQUIRED)
find_package(kuka_sunrise_interfaces REQUIRED)

get_filename_component(PARENT_DIR ${kuka_sunrise_DIR} DIRECTORY)
get_filename_component(PARENT_DIR ${PARENT_DIR} DIRECTORY)
get_filename_component(kuka_sunrise_INCLUDES ${PARENT_DIR} DIRECTORY)

include_directories(include ${kuka_sunrise_INCLUDES}/include)

add_executable(replay_motion src/ReplayMotion.cpp)
ament_target_dependencies(replay_motion rclcpp sensor_msgs kuka_sunrise kuka_sunrise_interfaces)

install(TARGETS replay_motion
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_copyright()
  ament_cppcheck(--language=c++)
  ament_pep257()
  ament_flake8()
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()
  ament_xmllint()
endif()

ament_package()
