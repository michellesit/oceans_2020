cmake_minimum_required(VERSION 2.8.3)
project(trash_utils)

find_package(catkin REQUIRED COMPONENTS)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uuv_gazebo_worlds
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS 
	src/trash_utils/finder_utils.py
	src/trash_utils/haversine_dist.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY src
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
