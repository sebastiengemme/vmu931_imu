# Recipe created by recipetool
# This is the basis of a recipe and may need further editing in order to be fully functional.
# (Feel free to remove these comments when editing.)

# WARNING: the following LICENSE and LIC_FILES_CHKSUM values are best guesses - it is
# your responsibility to verify that the values are complete and correct.
LICENSE = "BSD"
LIC_FILES_CHKSUM = "file://package.xml;beginline=14;endline=14;md5=d566ef916e9dedc494f5f793a6690ba5"

SRC_URI = "git://github.com/RobotnikAutomation/robotnik_msgs.git;protocol=https"

# Modify these as desired
PV = "0.2.4+git${SRCPV}"
SRCREV = "e8113fd15b3650d683a077a795be2a1a13f4dbeb"

S = "${WORKDIR}/git"

inherit catkin

# This is a Catkin (ROS) based recipe
# ROS package.xml format version 1

SUMMARY = "ROS package robotnik_msgs"
DESCRIPTION = "The robotnik_msgs package. Common messages and services used by some Robotnik's packages."
# ROS_AUTHOR = "Román Navarro <rnavarro@robotnik.es>"
# ROS_MAINTAINER = "Angel Soriano <asoriano@robotnik.es>"
# ROS_MAINTAINER += "Álvaro Villena <avillena@robotnik.es>"
# ROS_MAINTAINER += "David Redó <dredo@robotnik.es>"
# ROS_MAINTAINER += "Alejandro Arnal <aarnal@robotnik.es>"
# ROS_MAINTAINER += "Marc Bosch <mbosch@robotnik.es>"
# ROS_MAINTAINER += "Román Navarro <rnavarro@robotnik.es>"
SECTION = "devel"

ROS_SPN = "robotnik_msgs"

DEPENDS += "std-msgs"
DEPENDS += "message-generation"
DEPENDS += "actionlib-msgs"
RDEPENDS_${PN} += "std-msgs"
RDEPENDS_${PN} += "actionlib-msgs"
RDEPENDS_${PN} += "message-runtime"


