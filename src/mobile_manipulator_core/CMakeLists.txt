cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ── moveit_smoke_test ──────────────────────────────────────────────────────
add_executable(moveit_smoke_test src/moveit_smoke_test.cpp)
target_include_directories(moveit_smoke_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_smoke_test PUBLIC cxx_std_17)
ament_target_dependencies(moveit_smoke_test
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS moveit_smoke_test
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
