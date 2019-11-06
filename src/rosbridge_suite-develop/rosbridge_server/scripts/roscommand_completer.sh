#!/usr/bin/env bash


export COMP_CWORD=${2}
export COMP_WORDS=(${1})

# load ros completion functions
if [ -e "/opt/ros/melodic/share/rosbash/rosbash" ]
then
    . /opt/ros/kinetic/share/rosbash/rosbash
else
    if [ -e "/opt/ros/kinetic/share/rosbash/rosbash" ]
    then
        . /opt/ros/kinetic/share/rosbash/rosbash
    else
        (>&2 echo "ROS kinetic|melodic not found!")
        exit 1
    fi
fi

# ${COMP_WORDS[0]} is the ros command (eg 'rosmsg', 'rostopic') 
# so we'll evaluate_{roscmd} (eg roscomplete_rosmsg, roscomplete_rostopic)
eval "_roscomplete_${COMP_WORDS[0]}"
printf '%s\n' "${COMPREPLY[@]}"