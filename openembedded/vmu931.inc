SRC_URI = "git://github.com/sebastiengemme/vmu931_imu.git;protocol=https;branch=develop"

PV = "1.0.0+git${SRCPV}"
SRCREV = "${AUTOREV}"

S = "${WORKDIR}/git/${ROS_SPN}"

inherit catkin

