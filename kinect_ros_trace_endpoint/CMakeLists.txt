cmake_minimum_required(VERSION 3.5)
project(kinect_ros_trace_endpoint)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(kroshu_ros2_core REQUIRED)
find_package(moveit_kinematics)
find_package(sensor_msgs)

add_executable(moveit_with_markerpos src/MoveItWithMarkerPosNode.cpp)
ament_target_dependencies(moveit_with_markerpos
  moveit_ros_planning_interface
  rclcpp
  geometry_msgs
  visualization_msgs
  control_msgs
  moveit_kinematics
)

add_executable(marker_publisher src/MarkerPublisher.cpp)
ament_target_dependencies(marker_publisher
  visualization_msgs
  sensor_msgs
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  kroshu_ros2_core
)

add_executable(pose_publisher src/PosePublisher.cpp)
ament_target_dependencies(pose_publisher
  visualization_msgs
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  kroshu_ros2_core
)
target_link_libraries(marker_publisher ${kroshu_ros2_core_LIBRARIES})

target_include_directories(marker_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${kroshu_ros2_core_INCLUDE_DIRS})

install(TARGETS moveit_with_markerpos marker_publisher pose_publisher
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
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
