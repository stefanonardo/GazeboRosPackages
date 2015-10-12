cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_control)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  control_toolbox
  controller_manager
  hardware_interface 
  transmission_interface
  pluginlib
  joint_limits_interface
  urdf
  angles
)

# Depend on system install of Gazebo 
find_package(gazebo REQUIRED)
find_package(OGRE)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    controller_manager
    pluginlib
    transmission_interface
  INCLUDE_DIRS include
  DEPENDS gazebo
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
)

include_directories(include
  ${TBB_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Libraries
add_library(${PROJECT_NAME} src/gazebo_ros_control_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(default_robot_hw_sim src/default_robot_hw_sim.cpp)
target_link_libraries(default_robot_hw_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

## Install
install(TARGETS ${PROJECT_NAME} default_robot_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(FILES robot_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

