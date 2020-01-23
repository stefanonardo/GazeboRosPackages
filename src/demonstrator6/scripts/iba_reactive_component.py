#!/usr/bin/env python

import rospy
from external_module_interface.external_module import ExternalModule

from geometry_msgs.msg import Point
from std_msgs.msg import Bool


class IBAReactiveComponent(ExternalModule):
    def __init__(self, module_name=None, steps=1):
        super(IBAReactiveComponent, self).__init__(module_name, steps)
        self.steps = steps

    def initialize(self):
        self.obj_min = 90
        self.obj_max = 110
        self.US = None

        rospy.Subscriber("/obj_pos", Point, self.obj_pos_callback)
        self.react_pub = rospy.Publisher('/reactive_trigger', Bool, queue_size=10)

    def run_step(self):
        if self.US is not None:
            trig_msg = Bool()
            trig_msg.data = self.US
            self.react_pub.publish(trig_msg)

    def obj_pos_callback(self, msg):
        obj_pos = msg.x
        self.US = int( obj_pos > self.obj_min and obj_pos < self.obj_max )


if __name__ == "__main__":
    m = IBAReactiveComponent(module_name='reactive_module', steps=1)
    rospy.spin()
