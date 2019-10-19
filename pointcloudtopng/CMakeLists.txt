cmake_minimum_required(VERSION 2.8.3)
project(pointcloudtopng)

SET( CMAKE_BUILD_TYPE "Release" )

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
    pcl_ros
    pcl_conversions
    cv_bridge 
    image_transport
)


## find PCL
find_package(PCL REQUIRED)

find_package(OpenCV REQUIRED)

include_directories (${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Localization.msg
   PerceptionMsg.msg
   PerceptionListMsg.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
  # Or other packages containing msgs
 )

catkin_package(
		CATKIN_DEPENDS  std_msgs geometry_msgs message_runtime
)


include_directories(
 include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_definitions( ${EIGEN_DEFINITIONS} )
add_definitions( ${PCL_DEFINITIONS} )

## Declare a C++ library
#add_library(

 #)

add_executable(pointcloudtopng
		src/test.cpp
		)
target_link_libraries(pointcloudtopng  ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_dependencies(pointcloudtopng ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

