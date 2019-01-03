LICENSE = "BSD"
LIC_FILES_CHKSUM = "file://package.xml;beginline=9;endline=9;md5=d566ef916e9dedc494f5f793a6690ba5"

ROS_SPN = "vmu931_msgs"

SUMMARY = "ROS package vmu931 message definitions"
DESCRIPTION = "The vmu931_msg package"
SECTION = "devel"
DEPENDS = "robotnik-msgs std-msgs message-generation"
RRECOMMENDS_${PN} = "robotnik-msgs std-msgs message-runtime"

require vmu931.inc

