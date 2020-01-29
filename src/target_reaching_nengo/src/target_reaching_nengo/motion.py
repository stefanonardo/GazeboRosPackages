#!/usr/bin/env python

import nengo
import numpy as np

import rospy
from std_msgs.msg import Float64

class Motion(object):
    def __init__(self, joints, start, end, label, joint_mapping):
        self._joints = joints
        self._start = start
        self._end = end
        self._label = label
        self._joint_mapping = joint_mapping
        self.u_indice = 0.0
        self.pub_input= rospy.Publisher('input' , Float64, queue_size=1)
        self.pub_f_u= rospy.Publisher('f_u' , Float64, queue_size=1)

    def get_u(self, x):
            self.pub_f_u.publish(x)
            self.u_indice = self._data[0, (np.abs(self._data[0] - x)).argmin()]
            u_index = np.where(self._data[0] == self.u_indice)
            res = self._data[1,u_index]
            self.pub_input.publish(res)
            return res

    def get_indice(self, t):
        return self.u_indice

    #ORGINAL
    def map_voluntary(self, u):
        res=[]
        for i in range(len(self._joints)):
            if self._joint_mapping is not None: tmp = u * self._joint_mapping[i]
            else: tmp = u
            tmp = tmp * (self._end[i] - self._start[i])
            tmp = tmp + self._start[i]
            res.append(tmp)
        return res

    def map_voluntary_1(self, u):
        return u * (self._end[0] - self._start[0])   + self._start[0]

    def map_voluntary_2(self, u):
        return u * (self._end[1] - self._start[1])   + self._start[1]
