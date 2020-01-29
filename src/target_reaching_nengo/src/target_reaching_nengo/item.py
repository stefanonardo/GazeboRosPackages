#!/usr/bin/env python

from geometry_msgs.msg import Point

class Item(object):
    def __init__(self, name):
        self.position           = Point()
        self.orientation        = None
        self.name               = name
        self.polar_pos          =  [0.0, 0.0, 0.0] # r, theta, phi of TCP
        self.vector_to_shoulder = None
