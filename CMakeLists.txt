cmake_minimum_required(VERSION 2.8.3)
project(mc_rosgraph_manipulator)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

catkin_python_setup()

catkin_package()

# INSTALL
#catkin_install_python(PROGRAMS scripts/rosgraph_manipulator_example.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY config 
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
