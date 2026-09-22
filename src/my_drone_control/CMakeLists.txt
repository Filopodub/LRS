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
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Find System PCL dependency
find_package(PCL REQUIRED)

# Include PCL headers
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Define executable
add_executable(map_planner_node src/map_planner_node.cpp)

# Link ROS 2 and PCL libraries
target_link_libraries(map_planner_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(map_planner_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

install(TARGETS
  map_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()