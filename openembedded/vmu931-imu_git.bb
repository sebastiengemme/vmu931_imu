LICENSE = "BSD"
LIC_FILES_CHKSUM = "file://package.xml;beginline=10;endline=10;md5=d566ef916e9dedc494f5f793a6690ba5"

ROS_SPN = "vmu931_imu"

SUMMARY = "ROS package vmu931_imu"
DESCRIPTION = "The vmu931_imu package"
# ROS_MAINTAINER = "Juan Manuel Navarrete <jmnavarrete@robotnik.es>"
# ROS_MAINTAINER += "Román Navarro <rnavarro@robotnik.es>"
SECTION = "devel"
DEPENDS = "roscpp sensor-msgs vmu931-msgs genmsg rospy std-msgs message-generation"
RRECOMMENDS_${PN} = "roscpp python-pyserial sensor-msgs vmu931-msgs rospy std-msgs message-runtime"

require vmu931.inc

