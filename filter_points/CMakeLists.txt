cmake_minimum_required(VERSION 3.5)
project(filter_points)

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

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(utils SHARED
  src/GeometryUtils.cpp)
ament_target_dependencies(utils rclcpp)

add_executable(filter_points src/FilterPoints.cpp)
ament_target_dependencies(filter_points rclcpp visualization_msgs std_srvs)
target_link_libraries(filter_points utils)

install(TARGETS filter_points
    DESTINATION lib/${PROJECT_NAME})

ament_export_targets(utils HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

install(
  DIRECTORY include/utils
  DESTINATION include
)

install(
  TARGETS utils
  EXPORT utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


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
