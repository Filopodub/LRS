cmake_minimum_required(VERSION 3.8)
project(my_drone_control)

# Suppress standard CMake root variable policy warnings for Jazzy/PCL
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Find System PCL dependency
find_package(PCL REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# -----------------------------------------------------------------------------
# Node 1: takeoff_drone
# -----------------------------------------------------------------------------
add_executable(takeoff_drone src/takeoff_drone.cpp)
<<<<<<< HEAD
target_include_directories(takeoff_drone PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(takeoff_drone rclcpp mavros_msgs std_msgs)
install(TARGETS takeoff_drone
=======
ament_target_dependencies(takeoff_drone
  rclcpp
  mavros_msgs
  std_msgs
)

# -----------------------------------------------------------------------------
# Node 2: map_planner_node
# -----------------------------------------------------------------------------
add_executable(map_planner_node src/map_planner_node.cpp)
target_link_libraries(map_planner_node
  ${PCL_LIBRARIES}
)
ament_target_dependencies(map_planner_node
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
)

# -----------------------------------------------------------------------------
# Installation
# -----------------------------------------------------------------------------
install(TARGETS
  takeoff_drone
  map_planner_node
>>>>>>> 6c98e7b51f01338ecce985fa490b6dc1ee794d91
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Testing
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # skip linters that require git repo or copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
