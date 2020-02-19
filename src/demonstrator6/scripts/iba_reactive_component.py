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
        self.obj_min = 80 # could change it such that if it jumps from e.g 115 -> 85 it still gives a pulse..?
        self.obj_max = 100 # old 90-110
        self.US = False # None

        rospy.Subscriber("/obj_pos", Point, self.obj_pos_callback)
        self.react_pub = rospy.Publisher('/reactive_trigger', Bool, queue_size=10)
        self.US_steps_left = 0
        self.had_US = False

    def run_step(self):
        trig_msg = Bool()
        trig_msg.data = int(self.US)
        self.react_pub.publish(trig_msg)

        if self.US: # trigger steps countdown
            self.US_steps_left -= 1
            if self.US_steps_left == 0:
                self.US = False

    def obj_pos_callback(self, msg):
        obj_pos = msg.x
        if not self.had_US and (obj_pos > self.obj_min and obj_pos < self.obj_max): # If object in zone and we're not already triggering
            self.US = True
            self.US_steps_left = 10 # how many steps to trigger for
            self.had_US = True
        
        if obj_pos > self.obj_max: # reset flag to allow detection in zone again
            self.had_US = False


if __name__ == "__main__":
    m = IBAReactiveComponent(module_name='reactive_module', steps=1)
    rospy.spin()
