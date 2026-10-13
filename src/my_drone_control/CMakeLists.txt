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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlannedCommand.msg"
  "msg/PlannedMission.msg"
)

# -----------------------------------------------------------------------------
# Library: flight_assistant
# -----------------------------------------------------------------------------
add_library(flight_assistant_lib src/flight_assistant.cpp)
ament_target_dependencies(flight_assistant_lib
  rclcpp
  geometry_msgs
)
target_include_directories(flight_assistant_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(flight_assistant_lib PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_RPATH "$ORIGIN"
)

# -----------------------------------------------------------------------------
# Node: takeoff_drone
# -----------------------------------------------------------------------------
add_executable(takeoff_drone src/takeoff_drone.cpp)
target_link_libraries(takeoff_drone
  flight_assistant_lib
)
ament_target_dependencies(takeoff_drone
  rclcpp
  std_msgs
  geometry_msgs
  mavros_msgs
)
rosidl_get_typesupport_target(takeoff_cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(takeoff_drone
  "${takeoff_cpp_typesupport_target}"
)
set_target_properties(takeoff_drone PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_RPATH "$ORIGIN"
)

# -----------------------------------------------------------------------------
# Node: mission_demo_node
# -----------------------------------------------------------------------------
add_executable(mission_demo_node src/mission_demo_node.cpp)
ament_target_dependencies(mission_demo_node
  rclcpp
)
rosidl_get_typesupport_target(mission_demo_cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mission_demo_node
  "${mission_demo_cpp_typesupport_target}"
)

add_executable(map_planner_node src/map_planner_node.cpp)
ament_target_dependencies(map_planner_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  visualization_msgs
)
target_link_libraries(map_planner_node
  ${PCL_LIBRARIES}
)
target_include_directories(map_planner_node SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
rosidl_get_typesupport_target(map_planner_cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(map_planner_node "${map_planner_cpp_typesupport_target}")
add_dependencies(map_planner_node ${PROJECT_NAME}__rosidl_generator_cpp)

# -----------------------------------------------------------------------------
# Installation
# -----------------------------------------------------------------------------
install(TARGETS
  flight_assistant_lib
  takeoff_drone
  mission_demo_node
  map_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

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