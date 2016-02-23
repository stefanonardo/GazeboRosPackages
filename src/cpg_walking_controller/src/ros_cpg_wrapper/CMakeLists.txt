cmake_minimum_required(VERSION 2.8.3)
project(ros_cpg_wrapper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roscpp_serialization
  actionlib_msgs
  std_msgs
)

## Both Boost.python and Python libs are required.
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)

add_action_files(
  DIRECTORY action
  FILES ros_cpg_wrapper.action
)

add_action_files(
  DIRECTORY action
  FILES ros_cpg_parameters.action
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES cpg_control _cpg_control_wrapper_cpp
        CATKIN_DEPENDS roscpp actionlib_msgs
)

###########
## Build ##
###########

# include Boost and Python.
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        )


## Declare a cpp library
add_library(cpg_control src/cpg_control.cpp)
add_library(_cpg_control_wrapper_cpp src/cpg_control_wrapper.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(cpg_control ${catkin_LIBRARIES})
target_link_libraries(_cpg_control_wrapper_cpp cpg_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Don't prepend wrapper library name with lib and add to Python libs.
set_target_properties(_cpg_control_wrapper_cpp PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
        ) 

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
