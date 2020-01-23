#!/usr/bin/env python

import rospy
from external_module_interface.external_module import ExternalModule

from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool


class IBAStatsCollector(ExternalModule):
    def __init__(self, module_name=None, steps=1):
        super(IBAStatsCollector, self).__init__(module_name, steps)

    def initialize(self):
        self.trial_num = 0
        self.last_obj_x = None

        self.had_US = False
        self.had_CR = False
        self.trial_start_time = rospy.Time().now()
        self.CR_trigger_time = None
        self.US_time = None

        self.had_US_vec = []
        self.had_CR_vec = []
        self.CR_latency_vec = [] # trial start to CR trigger
        self.ISI_vec = []

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)
        rospy.Subscriber("/reactive_trigger", Bool, self.reactive_trigger_callback)
        rospy.Subscriber("/adaptive_trigger", Bool, self.adaptive_trigger_callback)

    def model_states_callback(self, msg):
        try:
            obj_index = msg.name.index('ball')
            obj_pos = msg.pose[obj_index].position

            if self.last_obj_x is not None:
                if obj_pos.x - self.last_obj_x < -0.1:
                    self.trial_num += 1

                    if self.CR_trigger_time is None:
                        CR_latency = None
                    else:
                        CR_latency = (self.CR_trigger_time - self.trial_start_time).to_sec()

                    if self.US_time is None:
                        ISI = None
                    else:
                        ISI = (self.US_time - self.trial_start_time).to_sec()

                    print("End of trial. Had US: ", self.had_US, " had CR:", self.had_CR, 'latency: ', CR_latency, 'ISI: ', ISI)
                    self.had_US_vec.append(self.had_US)
                    self.had_CR_vec.append(self.had_CR)
                    self.CR_latency_vec.append(CR_latency)
                    self.ISI_vec.append(ISI)

                    self.had_US = False
                    self.had_CR = False
                    print("Start of new trial, ", self.trial_num)
                    self.trial_start_time = rospy.Time().now()
                    self.CR_trigger_time = None
                    self.US_time = None

            self.last_obj_x = obj_pos.x

        except ValueError:
            print("Obj not found")

    def adaptive_trigger_callback(self, msg):
        self.CR = msg.data
        if not self.had_CR and self.CR:
            self.had_CR = True
            self.CR_trigger_time = rospy.Time().now()
        
        
    def reactive_trigger_callback(self, msg):
        self.US = msg.data
        if not self.had_US and self.US:
            self.had_US = True
            self.US_time = rospy.Time().now()

    def shutdown(self):
        import pickle
        import os
        fname = os.getcwd() + '/resources/cerebellum_stats.p'
        pickle.dump({'had_US': self.had_US_vec, 'had_CR': self.had_CR_vec}, open(fname, 'wb'))

if __name__ == "__main__":

    m = IBAStatsCollector(module_name='stats_collector', steps=1)

    rospy.spin()
