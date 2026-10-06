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
find_package(mavros_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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
# Node: flight_assistant_node
# -----------------------------------------------------------------------------
add_executable(flight_assistant_node src/flight_assistant_node.cpp)
target_link_libraries(flight_assistant_node
  flight_assistant_lib
)
set_target_properties(flight_assistant_node PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_RPATH "$ORIGIN"
)

# -----------------------------------------------------------------------------
# Node: navigation_node
# -----------------------------------------------------------------------------
add_executable(navigation_node src/navigation.cpp)
target_link_libraries(navigation_node
  flight_assistant_lib
)
ament_target_dependencies(navigation_node
  rclcpp
  std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(navigation_node
  "${cpp_typesupport_target}"
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
  mavros_msgs
)
set_target_properties(takeoff_drone PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_RPATH "$ORIGIN"
)

# -----------------------------------------------------------------------------
# Installation
# -----------------------------------------------------------------------------
install(TARGETS
  flight_assistant_lib
  flight_assistant_node
  navigation_node
  takeoff_drone
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
