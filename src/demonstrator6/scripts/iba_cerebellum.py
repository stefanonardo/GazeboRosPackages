#!/usr/bin/env python
import numpy as np
import os
import pickle

import rospy
import rospkg
from external_module_interface.external_module import ExternalModule

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

from cerebellum.cerebellum import Cerebellum

class IBACerebellum(ExternalModule):
    def __init__(self, module_name=None, steps=1):
        super(IBACerebellum, self).__init__(module_name, steps)
        self.steps = steps

    def initialize(self):
        desired_Ts = 1e-3 # with rounding we might not get this exact Ts, since the IBA only divides the CLE time step by powers of 2
        n_inputs = 320
        n_bases = 25
        beta = 2e-4  
        kc = 0.05
        delta = 650 # units of Ts - TODO: correct when real Ts is different? e.g. when real Ts will be 1.25 ms, we need to put 400 for 500 ms delay

        self.CS = None
        self.obj_pos = None
        self.US = 0

        self.output = None
        self.CR = False
        self.CR_threshold = 1.0 #0.5

        cle_Ts = 20e-3 # Has to match the CLE timestep set in "bibi_configuration.bibi"
        self.n_steps_per_cle_step = int(cle_Ts/desired_Ts/self.steps) # assuming self.steps contains the steps given at initialization
        Ts = cle_Ts / self.n_steps_per_cle_step / self.steps # the resulting Ts
        rospy.loginfo("[Cerebellum] Timestep set to " + str(Ts))
        rospy.loginfo("[Cerebellum] Going to run " + str(self.n_steps_per_cle_step) + " steps per CLE step")

        self.use_trained_model = False
        self.save_model = True
        self.debug_logging = False
        
        self.basepath = rospkg.RosPack().get_path('demonstrator6') + '/scripts'

        self.c = Cerebellum(Ts, n_inputs, n_bases, beta/n_inputs, kc, delta) 
        self.c_out_share = [0]

        if self.use_trained_model:
            self.c.import_state( pickle.load( open( self.basepath + '/resources/cerebellum_state.p', "rb" ) ) )
            rospy.loginfo("[Cerebellum] Weights loaded from save.")
        else:
            rospy.loginfo("[Cerebellum] Initialized with random weights.")


        rospy.Subscriber("/latent", Float32MultiArray, self.latent_callback) # MF - CS
        rospy.Subscriber("/reactive_trigger", Bool, self.reactive_trigger_callback) # US

        self.trig_pub = rospy.Publisher('/adaptive_trigger', Bool, queue_size=10) # CR

        # For log
        if self.debug_logging:
            self.c_out_vec = []
            self.US_vec = []
            self.CR_vec = []

    def run_step(self):
        ## Main cerebellum code
        if self.CS is not None: # wait for first input
            self.c_out_share = [0]
            
            crs = np.zeros(self.n_steps_per_cle_step, dtype=bool)
            for i in range(self.n_steps_per_cle_step):
                self.c.step(self.CS, self.US)
                self.output = self.c.output
                crs[i] = self.output > self.CR_threshold

                #print("C output: " + str(self.output) )
                self.c_out_share.append(self.output)
                
                # LOG
                if self.debug_logging:
                    self.c_out_vec.append(self.output)
                    self.US_vec.append(self.US)
                    self.CR_vec.append(crs[i])
            self.CR = crs.any()
 
        # Publish
        trig_msg = Bool()
        trig_msg.data = self.CR
        self.trig_pub.publish(trig_msg)

    def share_module_data(self):
        if self.CS is not None: # wait for first input
            self.module_data = self.c_out_share

    def shutdown(self):
        ## LOG time series data
        if self.debug_logging:
            fname = self.basepath + '/resources/cerebellum_log.p'
            pickle.dump( {'US_vec': np.array(self.US_vec),
                'CR_vec': np.array(self.CR_vec),
                'c_out_vec': np.array(self.c_out_vec),
                },
                open( fname, "wb" ) ) 

        # Save model parameters if it is a new one.
        if self.save_model:
            fname = self.basepath + '/resources/cerebellum_state.p'
            pickle.dump( self.c.export_state(), open(fname, "wb" ) )

    def latent_callback(self, msg):
        dims = tuple(msg.layout.dim[i].size for i in range(len(msg.layout.dim)))
        a = np.reshape(msg.data, dims) # has the shape defined in the message
        self.CS = a.flatten() # however, we just flatten it here

    def reactive_trigger_callback(self, msg):
        self.US = msg.data


if __name__ == "__main__":
    m = IBACerebellum(module_name='cerebellum_module', steps=1)
    rospy.spin()
