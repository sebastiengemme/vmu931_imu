cmake_minimum_required(VERSION 2.8.3)
project(vmu931_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  robotnik_msgs
  message_generation
)

add_message_files(
   DIRECTORY msg
   FILES
   State.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   robotnik_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs robotnik_msgs
)
