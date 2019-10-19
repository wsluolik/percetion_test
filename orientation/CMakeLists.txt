cmake_minimum_required(VERSION 2.8.3)
project(orientation)

SET( CMAKE_BUILD_TYPE "Release" )
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	     roscpp
	     rospy
	     std_msgs
	     geometry_msgs
	     message_generation
		)


## find PCL
find_package(PCL REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})

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
)

add_definitions( ${EIGEN_DEFINITIONS} )

## Declare a C++ library
#add_library(

 #)

add_executable(orientation
		src/test.cpp
		)
add_dependencies(orientation ${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS})
target_link_libraries(orientation ${catkin_LIBRARIES} )

