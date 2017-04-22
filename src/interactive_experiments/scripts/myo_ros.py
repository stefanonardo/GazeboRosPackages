# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END 
#!/usr/bin/python

import rospy
import std_msgs
import time

from PyoConnect import *
myo = Myo(sys.argv[1] if len(sys.argv) >= 2 else None)
''' ============================== '''

rospy.init_node('myo_ros')
startR, startP, startY = -1.4, -1.0, 0.0
start_real = {}
start_real['r'], start_real['p'], start_real['y'] = 0.0, 1.0, 0

# construct publishers for arm joints
pub1 = rospy.Publisher('/robot/hollie_real_left_arm_4_joint/cmd_pos', std_msgs.msg.Float64, queue_size=10) # roll (elbow rotation == palm rotation)
pub2 = rospy.Publisher('/robot/hollie_real_left_arm_3_joint/cmd_pos', std_msgs.msg.Float64, queue_size=10) # pithc (elbow flexion)
pub3 = rospy.Publisher('/robot/hollie_real_left_arm_1_joint/cmd_pos', std_msgs.msg.Float64, queue_size=10) # yaw (shoulder)

# lock for not starting publishers before calibration and unlock are done
publishingToRobot = True

# Center is set when you first double tap to unlock.
# It is kept unlocked forever
def onUnlock():
    myo.rotSetCenter()
    X, Y = 0, 0
    print('reset')
    myo.unlock("hold")

# in case you end up pointing to weird directions,
# just point to where you want the center to be and double tap again
def onPoseEdge(pose, edge):
    global publishingToRobot, time_start
    print(pose, edge)
    if (pose == 'doubleTap') and (edge == "on"):
        myo.rotSetCenter()

def onPeriodic():

    r, p, y = myo.rotRoll(), myo.rotPitch(), myo.rotYaw()

    # control simulated robot
    r = startR - r * 2
    p = startP + p
    y = startY + -y * 1.3

    r = -r
    #y = -y
    p = -p

    global publishingToRobot
    if publishingToRobot:
        pub1.publish(r)
        pub2.publish(p)
        pub3.publish(y)

# Comment out below the events you are not using
#myo.onLock = onLock
myo.onUnlock = onUnlock
myo.onPoseEdge = onPoseEdge
myo.onPeriodic = onPeriodic
#myo.onWear = onWear
#myo.onUnwear = onUnwear
#myo.onEMG = onEMG
#myo.onBoxChange = onBoxChange

myo.connect()
time_start = time.time()
while True:
    myo.run()
    myo.tick()
