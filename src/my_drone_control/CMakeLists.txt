cmake_minimum_required(VERSION 3.8)
project(my_drone_control)

<<<<<<< HEAD
=======
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

>>>>>>> origin/main
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

<<<<<<< HEAD
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(takeoff_drone src/takeoff_drone.cpp)
ament_target_dependencies(takeoff_drone rclcpp mavros_msgs std_msgs)
install(TARGETS takeoff_drone
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
=======
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
>>>>>>> origin/main
