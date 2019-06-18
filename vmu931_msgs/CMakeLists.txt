cmake_minimum_required(VERSION 3.5)
project(vmu931_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robotnik_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(NOT WIN32)
  add_definitions(-std=c++11)
endif()

set(msg_files msg/VMUState.msg)

## Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs robotnik_msgs
)

ament_export_dependencies(std_msgs robotnik_msgs)
ament_package()

