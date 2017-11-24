cmake_minimum_required(VERSION 2.8.3)
project(pses_kinect_utilities)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  pcl_ros
  image_transport
  sensor_msgs
  dynamic_reconfigure
  image_geometry
  nodelet
)

find_package(OpenCV)
find_package(Boost REQUIRED)
find_package(OpenCL REQUIRED)
include_directories(${OpenCL_INCLUDE_DIRS})
link_directories(${OpenCL_LIBRARY})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME})

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/depth_image_to_pcl.cpp
  src/ocl_library_wrapper.cpp
  src/nodelets/median_filter.cpp
  src/nodelets/point_cloud_xyz.cpp
  )


## Specify libraries to link a library or executable target against
#target_link_libraries(filter_lib ${catkin_LIBRARIES} ${opencv_LIBRARIES} ${PCL_LIBRARIES} ${OpenCL_LIBRARY})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
