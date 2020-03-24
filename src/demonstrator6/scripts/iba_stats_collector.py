#!/usr/bin/env python

import collections
import numpy as np

import rospy
import rospkg
from external_module_interface.external_module import ExternalModule

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, Bool

Trial = collections.namedtuple('Trial', ['start_time', 'had_US', 'had_CR', 'CR_latency', 'ISI', 'CR', 'US', 'CS', 'obj_pos_px', 'obj_pos_gt', 'CR_obj_pos'])

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
        self.CR_vec = []
        self.US_vec = []
        self.CS_vec = []
        self.obj_pos_px_vec = [] # pixel location
        self.obj_pos_gt_vec = [] # ground truth
        self.CR_obj_pos = None

        self.trials = []
        self.save_file_num = 0
        import time
        self.timestr = time.strftime("%Y%m%d-%H%M%S")

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)
        rospy.Subscriber("/reactive_trigger", Bool, self.reactive_trigger_callback)
        rospy.Subscriber("/adaptive_trigger", Bool, self.adaptive_trigger_callback)
        rospy.Subscriber("/latent", Float32MultiArray, self.latent_callback)
        rospy.Subscriber("/obj_pos", Point, self.obj_pos_callback)

    def run_step(self):
        for m in [self.database_resp.m1, self.database_resp.m2, self.database_resp.m3]: 
            if len(m) > 2 and m[1] == 0: # workaround to select the one with "ID 0". Firs element is ??, second is what I set in the other node.
                self.CR_vec += m[2:]

    def model_states_callback(self, msg):
        # Given a new box position
        try:
            obj_index = msg.name.index('ball')
            obj_pos = msg.pose[obj_index].position

            self.obj_pos_gt_vec.append( ( (rospy.Time().now()-self.trial_start_time).to_sec(), [obj_pos.x, obj_pos.y, obj_pos.z] ) )

            if self.last_obj_x is not None:
                if obj_pos.x - self.last_obj_x < -0.1: # detects new trial if x has moved too fast in negative direction
                    self.trial_num += 1

                    # Calculate stats for previous trial
                    if self.CR_trigger_time is None:
                        CR_latency = None
                    else:
                        CR_latency = (self.CR_trigger_time - self.trial_start_time).to_sec()

                    if self.US_time is None:
                        ISI = None
                    else:
                        ISI = (self.US_time - self.trial_start_time).to_sec()

                    rospy.loginfo("[Stats collector] Detected end of trial. Had US: %s, had CR: %s, latency: %s, ISI: %s" % (self.had_US, self.had_CR, CR_latency, ISI) )

                    # Save trial stats
                    t = Trial(self.trial_start_time.to_sec(), self.had_US, self.had_CR, CR_latency, ISI, self.CR_vec[:], self.US_vec[:], self.CS_vec[:], self.obj_pos_px_vec[:], self.obj_pos_gt_vec[:], self.CR_obj_pos)
                    self.trials.append(t)

                    # Reset trial variables
                    self.had_US = False
                    self.had_CR = False
                    self.trial_start_time = rospy.Time().now()
                    self.CR_trigger_time = None
                    self.US_time = None
                    self.CR_vec = []
                    self.US_vec = []
                    self.CS_vec = []
                    self.obj_pos_px_vec = []
                    self.obj_pos_gt_vec = []
                    self.CR_obj_pos = None

                    self.save_trials_cond()

                    rospy.loginfo("[Stats collector] Start of new trial: %d" % self.trial_num)


            self.last_obj_x = obj_pos.x

        except ValueError:
            print("Obj not found")

    def save_trials_cond(self):
        save_trial_interval = 100
        if len(self.trials) >= save_trial_interval:
            self.save_trials()
            

    def save_trials(self):
        import pickle
        fname = rospkg.RosPack().get_path('demonstrator6') + '/scripts/resources/cerebellum_stats_' + self.timestr + '_' + str(self.save_file_num) + '.p'
        pickle.dump(self.trials, open(fname, 'wb'))
        self.trials = []
        self.save_file_num += 1

    def adaptive_trigger_callback(self, msg):
        self.CR = msg.data
        if not self.had_CR and self.CR:
            self.had_CR = True
            self.CR_trigger_time = rospy.Time().now()
            self.CR_obj_pos = self.last_obj_x
        
    def reactive_trigger_callback(self, msg):
        self.US = msg.data
        self.US_vec.append( ( (rospy.Time().now()-self.trial_start_time).to_sec(), int(self.US) ) )
        if not self.had_US and self.US:
            self.had_US = True
            self.US_time = rospy.Time().now()

    def latent_callback(self, msg):
        dims = tuple(msg.layout.dim[i].size for i in range(len(msg.layout.dim)))
        a = np.reshape(msg.data, dims) # has the shape defined in the message
        self.CS = a.flatten().tolist() # however, we just flatten it here
        self.CS_vec.append( ( (rospy.Time().now()-self.trial_start_time).to_sec(), self.CS ) )

    def obj_pos_callback(self, msg):
        self.obj_pos_px = msg.x
        self.obj_pos_px_vec.append( ( (rospy.Time().now()-self.trial_start_time).to_sec(), self.obj_pos_px ) )

    def shutdown(self):
        self.save_trials()

if __name__ == "__main__":

    m = IBAStatsCollector(module_name='stats_collector', steps=1)

    rospy.spin()
