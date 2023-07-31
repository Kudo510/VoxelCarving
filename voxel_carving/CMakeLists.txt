cmake_minimum_required(VERSION 3.0.2)
project(voxel_carving)

set(OpenCV_INCLUDE_DIRS
  /usr/local/include
  /usr/local/include/opencv2
)

find_package(catkin REQUIRED COMPONENTS
  red_msgs
  rospy
  roscpp
  std_msgs
  sensor_msgs
  OpenCV
)

catkin_python_setup()

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS red_msgs rospy roscpp
)

# required packages
FIND_PACKAGE(VTK REQUIRED)
# required macros and variables
INCLUDE(${VTK_USE_FILE})



include_directories( 
    ${catkin_INCLUDE_DIRS}
    include
                      )



add_executable(
  voxel_carver 
  src/voxelCarver.cpp
  )
target_link_libraries(voxel_carver ${catkin_LIBRARIES}
                                  ${OpenCV_INCLUDE_DIRS}
                                  ${YAML_CPP_LIBRARIES}
                                  ${VTK_LIBRARIES})


catkin_install_python(PROGRAMS scripts/imageSaver.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

