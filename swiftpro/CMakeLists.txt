cmake_minimum_required(VERSION 3.5)
project(swiftpro)

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

find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/SwiftproState.msg"
  "msg/Position.msg"
  "msg/Angle4th.msg"
  "msg/Status.msg")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files})

add_executable(swiftpro_write_node src/swiftpro_write_node.cpp)
add_executable(swiftpro_moveit_node src/swiftpro_moveit_node.cpp)
add_executable(swiftpro_rviz_node src/swiftpro_rviz_node.cpp)
add_executable(swiftpro_read_node src/swiftpro_read_node.cpp)

ament_target_dependencies(swiftpro_write_node rclcpp serial)
ament_target_dependencies(swiftpro_moveit_node rclcpp sensor_msgs)
ament_target_dependencies(swiftpro_rviz_node rclcpp tf2_geometry_msgs sensor_msgs)
ament_target_dependencies(swiftpro_read_node rclcpp serial)

install(TARGETS
  swiftpro_write_node
  swiftpro_moveit_node
  swiftpro_rviz_node
  swiftpro_read_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})

rosidl_target_interfaces(swiftpro_write_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(swiftpro_moveit_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(swiftpro_rviz_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(swiftpro_read_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

#ament_export_dependencies(
#  rosidl_default_runtime
#  std_msgs
#)

ament_package()
