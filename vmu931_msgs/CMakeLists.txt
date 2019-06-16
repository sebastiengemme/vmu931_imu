cmake_minimum_required(VERSION 2.8.3)
project(vmu931_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robotnik_msgs REQUIRED)
  message_generation
)

if(NOT WIN32)
  add_definitions(-std=c++11)
endif()

set(msg_files msg/State.msg)

## Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs 
)

ament_export_dependencies(std_msgs)
ament_package()

