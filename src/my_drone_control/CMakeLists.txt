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

# 1. Find ROS 2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)

# 2. Find System PCL Dependency
find_package(PCL REQUIRED COMPONENTS common io filters)

# Include PCL headers
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# -----------------------------------------------------------------------------
# Node 1: takeoff_drone
# -----------------------------------------------------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/takeoff_drone.cpp")
  add_executable(takeoff_drone src/takeoff_drone.cpp)
  ament_target_dependencies(takeoff_drone
    rclcpp
    mavros_msgs
    std_msgs
  )
endif()

# -----------------------------------------------------------------------------
# Node 2: map_planner_node
# -----------------------------------------------------------------------------
add_executable(map_planner_node src/map_planner_node.cpp)

# Pass ROS 2 header directories to target
ament_target_dependencies(map_planner_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  visualization_msgs
)

# Link system PCL libraries
target_link_libraries(map_planner_node
  ${PCL_LIBRARIES}
)

# -----------------------------------------------------------------------------
# Installation
# -----------------------------------------------------------------------------
install(TARGETS
  map_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

if(TARGET takeoff_drone)
  install(TARGETS takeoff_drone
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY launch config maps missions
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Testing
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()