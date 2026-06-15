cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(mm_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

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

# ── grasp_server ──────────────────────────────────────────────────────────────
add_executable(grasp_server src/grasp_server.cpp)
target_include_directories(grasp_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(grasp_server PUBLIC cxx_std_17)
ament_target_dependencies(grasp_server
  rclcpp
  std_msgs
  std_srvs
)

# ── state_manager_node ─────────────────────────────────────────────────────
add_executable(state_manager_node src/state_manager_node.cpp)
target_include_directories(state_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(state_manager_node PUBLIC cxx_std_17)
ament_target_dependencies(state_manager_node
  rclcpp
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2
  mm_interfaces
)
target_link_libraries(state_manager_node yaml-cpp)

# ── tool_server_node ───────────────────────────────────────────────────────
add_executable(tool_server_node src/tool_server_node.cpp)
target_include_directories(tool_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tool_server_node PUBLIC cxx_std_17)
ament_target_dependencies(tool_server_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  std_msgs
  nav2_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2
  mm_interfaces
)

# ── tool_test ──────────────────────────────────────────────────────────────
add_executable(tool_test src/tool_test.cpp)
target_include_directories(tool_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tool_test PUBLIC cxx_std_17)
ament_target_dependencies(tool_test
  rclcpp
  mm_interfaces
)

install(TARGETS
    moveit_smoke_test
    grasp_server
    state_manager_node
    tool_server_node
    tool_test
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
