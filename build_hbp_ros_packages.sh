#!/bin/bash -e
# ------------------------------------------------------------------
INFO_TEXT="\nBuild HBP ROS packages (original or customized ROS packages\n\n" 
INFO_TEXT+="When deploying, this script needs to be run on an HBP virtual machine \n"
INFO_TEXT+="that can access /nfs4/bbp.epfl.ch." 
INFO_TEXT+="For example, bbpce021.epfl.ch is a good place where to launch this script.\n\n"
INFO_TEXT+="Before running that script, run\n" 
INFO_TEXT+="chmod a+w /nfs4/bbp.epfl.ch/sw/neurorobotics/ros-hbp-packages \n"
INFO_TEXT+="on a machine where you are logged with your username and where "
INFO_TEXT+="the nfs4 is mounted.\n\n"
INFO_TEXT+="After the script execution, run the opposite:\n"
INFO_TEXT+="chmod a-w /nfs4/bbp.epfl.ch/sw/neurorobotics/ros-hbp-packages."

# Some unusual yum packages are needed to build this:
DEPENDENCIES="tinyxml-devel freeimage-devel cmake log4cxx-devel libuuid-devel poco-devel yaml-cpp-devel"
BUILD_DIRECTORY="catkin_ws"


display_usage() {
  echo -e "\n$INFO_TEXT"
  echo -e "\nUsage:\n$0 [-h|--help] [-l|--local-build] [-d|--deploy] [-y|--yum-install-dependencies]\n"
  echo -e "Options:"
  echo -e " -l|--local-build:\n  builds GazeboRospackages in GazeboRosPackages/$BUILD_DIRECTORY \n"
  echo -e " -d|--deploy:\n   builds GazeboRospackages and installs it in /nfs4 \n"
  echo -e " -y|--yum-install-dependencies:\n   installs $DEPENDENCIES \n"
}

if [[ $# == 0 ]]; then
  display_usage
  exit 1
fi

while [[ $# > 0 ]]
do
key="$1"

case $key in
    -h|--help)
    display_usage
    exit 0
    ;;
    -l|--local-build)
    DEPLOY="false"
    ;;
    -d|--deploy)
    DEPLOY="true"
    ;;
    -y|-yum-install-dependencies)
    INSTALL_DEPENDENCIES="true"
    ;;
    *)
    display_usage # unknown option
    exit 1
    ;;
esac
shift # past argument or value
done

if [ "$INSTALL_DEPENDENCIES" == "true" ]; then
  echo "\nInstalling build dependencies \n"
  yum install $DEPENDENCIES
fi

# We need python 2.7
source /opt/rh/python27/enable

NFS4_NEUROROBOTICS=/nfs4/bbp.epfl.ch/sw/neurorobotics

# Get the proper modules
export MODULEPATH=$MODULEPATH:$NFS4_NEUROROBOTICS/modulefiles
module load boost/1.55zlib-rhel6-x86_64-gcc4.4
module load ros/hydro-rhel6-x86_64-gcc4.4
module load gazebo/4.0-rhel6-x86_64-gcc4.8.2
module load opencv/2.4.9-rhel6-x86_64-gcc4.8.2
module load sdf/2.0-rhel6-x86_64-gcc4.4
module load tbb/4.0.5-rhel6-x86_64-gcc4.4
module load ogre/1.8.1-rhel6-x86_64-gcc4.8.2
module load ros-thirdparty/hydro-rhel6-x86_64-gcc4.4

source $ROS_SETUP_FILE
source $ROS_THIRDPARTY_PACKAGES_SETUP_FILE

# Create a python venv in order to get the bases ROS install tools
virtualenv build_venv
. build_venv/bin/activate

USE_DEV_PI="-i http://bbpgb019.epfl.ch:9090/simple"

pip install $USE_DEV_PI catkin_pkg
pip install $USE_DEV_PI empy
pip install $USE_DEV_PI PyYAML
pip install $USE_DEV_PI rospkg
pip install $USE_DEV_PI netifaces
pip install $USE_DEV_PI rosinstall
pip install $USE_DEV_PI rosinstall_generator

echo -e "\nCleaning $BUILD_DIRECTORY\n" 
rm -rf $BUILD_DIRECTORY
mkdir $BUILD_DIRECTORY
cd $BUILD_DIRECTORY
cp -R ../src .
cd src
echo -e "\ncatkin init\n" 
catkin_init_workspace
cd ..

# WARNING: There is problem with cmake: After installing the thirdparty packages fixed paths are written to the file /nfs4/bbp.epfl.ch/sw/neurorobotics/ros-thirdparty/hydro/rhel-6.5-x86_64/gcc-4.4.7/x86_64/share/joint_limits_interface/cmake/joint_limits_interfaceConfig.cmake
# TODO: Manually remove '<custom_build_folder>/CLE/build/src/ros_control/hardware_interface/include' from line 96: 'set(_include_dirs "include; ...'
BOOST_ROOT=$NFS4_NEUROROBOTICS/boost/1.55-zlib/rhel-6.5-x86_64/gcc-4.4.7/x86_64/
echo -e "\nBuilding GazeboRospackages\n" 
catkin_make \
-DBoost_INCLUDE_DIR=$BOOST_INCLUDEDIR \
-DBoost_LIBRARY_DIRS=$BOOST_LIBDIR \
-DBoost_NO_BOOST_CMAKE=true \
-DTBB_INCLUDE_DIR=$TBB_INCLUDE_DIR \
-DBOOST_SIGNALS_NO_DEPRECATION_WARNING=true

if [ "$DEPLOY" == "true" ]; then
  echo -e "\nInstalling build in $NFS4_NEUROROBOTICS\n"
  catkin_make \
  -DCATKIN_ENABLE_TESTING=0 \
  -DCMAKE_INSTALL_PREFIX=$NFS4_NEUROROBOTICS/ros-hbp-packages/hydro/rhel-6.5-x86_64/gcc-4.4.7/x86_64/ \
  install
fi

