cmake_minimum_required(VERSION 3.0.2)
project(voxel_carving)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  red_msgs
  rospy
  std_msgs
  sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES voxel_carving
#  CATKIN_DEPENDS red_msgs rospy stdmsgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

find_package( OpenCV REQUIRED )
include_directories( 
    include
    ${OpenCV_INCLUDE_DIRS}
                      )


catkin_install_python(PROGRAMS scripts/imageSaver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

