cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME "whiskeye_plugin")
project(${PROJECT_NAME})

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    message(STATUS "Compiler supports cxx11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    message(STATUS "Compiler supports c++0x")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_msgs
  roscpp
  std_msgs
  image_transport
)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# find_package(OGRE)
find_package(gazebo REQUIRED)
find_package(ignition-math4 REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(DIRECTORY msg
          FILES
          bridge_u.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES whiskeye_plugin
  CATKIN_DEPENDS gazebo_ros gazebo_msgs roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIR}
  ${SDFormat_LIBRARIES} 
)

## Declare a cpp library
 add_library(whiskeye_plugin
   src/libwhiskeye_gazebo.cpp
 )

#link_directories(
#  ${OGRE_LIBRARY_DIRS}
#)

## Specify libraries to link a library or executable target against
 target_link_libraries(whiskeye_plugin
   ${catkin_LIBRARIES}
   ${GAZEBO_LIBRARIES}
   ${SDFormat_LIBRARIES} 
   ignition-math4::ignition-math4
   
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS whiskeye_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
