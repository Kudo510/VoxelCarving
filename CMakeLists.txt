CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(VoxelCarving)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# required packages
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(VTK REQUIRED)
FIND_PACKAGE(aruco REQUIRED)
# required macros and variables
INCLUDE(${VTK_USE_FILE})

FILE(
    GLOB SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp 
    ${CMAKE_SOURCE_DIR}/src/*.h 
)

# Create the executable
ADD_EXECUTABLE(${PROJECT_NAME} ${SOURCES})

# Link libraries
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
    ${VTK_LIBRARIES}
)

ADD_EXECUTABLE( marker_generator ${CMAKE_SOURCE_DIR}/ar_src/marker_generator.cpp)
TARGET_LINK_LIBRARIES( marker_generator ${OpenCV_LIBS}  ${aruco_LIBS})
