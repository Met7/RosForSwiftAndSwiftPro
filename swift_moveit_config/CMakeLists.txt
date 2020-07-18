cmake_minimum_required(VERSION 3.5)
project(swift_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})

ament_package()

