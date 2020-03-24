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
    """ IBA module for the cerebellum.

        Receives contextual information (CS: conditioned stimuli) and error signal (US: unconditioned stimuli).
        
        Learns from trial to trial to trigger to avoid the US.

        Sends a boolean trigger signal.
    """
    def __init__(self, module_name=None, steps=1):
        super(IBACerebellum, self).__init__(module_name, steps)
        self.steps = steps

    def initialize(self):
        desired_Ts = 1e-3 # with rounding we might not get this exact Ts, since the IBA only divides the CLE time step by powers of 2
        n_inputs = 320 # Number of inputs
        n_bases = 25 # Number of filters in the filterbank.
        beta = 2e-4 # Learning rate
        kc = 0.05 # Strength of the NOI (nuclei-olivary inhibition). Determines extinction time.
        desired_delta = 500 # The required time between triggering and effective prevention of the US. In units of desired_Ts.


        self.CS = None
        self.US = 0

        self.output = None
        self.CR = False
        self.CR_threshold = 1.0

        cle_Ts = 20e-3 # Has to match the CLE timestep set in "bibi_configuration.bibi"
        self.n_steps_per_cle_step = int(cle_Ts/desired_Ts/self.steps) # self.steps contains the steps given at initialization
        Ts = cle_Ts / self.n_steps_per_cle_step / self.steps # the resulting Ts

        # When real Ts is different from desired, the delta has to be calculated in units of real Ts
        delta = int(desired_delta * desired_Ts / Ts) # in units of Ts

        rospy.loginfo("[Cerebellum] Timestep set to " + str(Ts))
        rospy.loginfo("[Cerebellum] delta set to " + str(delta))
        rospy.loginfo("[Cerebellum] Going to run " + str(self.n_steps_per_cle_step) + " steps per CLE step")

        self.use_trained_model = False # Whether to use a previously saved model state
        self.save_model = True # Whether to save the model state at deinitialization
        self.debug_logging = False
        
        self.basepath = rospkg.RosPack().get_path('demonstrator6') + '/scripts'

        # Initialize cerebellum
        self.c = Cerebellum(Ts, n_inputs, n_bases, beta/n_inputs, kc, delta) 

        # Output vector for sharing cerebellum output in between CLE timesteps
        self.c_out_share = [0]

        if self.use_trained_model:
            self.c.import_state( pickle.load( open( self.basepath + '/resources/cerebellum_state.p', "rb" ) ) )
            rospy.loginfo("[Cerebellum] Weights loaded from save.")
        else:
            rospy.loginfo("[Cerebellum] Initialized with random weights.")


        rospy.Subscriber("/latent", Float32MultiArray, self.latent_callback) # MF - CS
        rospy.Subscriber("/reactive_trigger", Bool, self.reactive_trigger_callback) # US

        self.trig_pub = rospy.Publisher('/adaptive_trigger', Bool, queue_size=10) # CR

        # For debug logging
        if self.debug_logging:
            self.c_out_vec = []
            self.US_vec = []
            self.CR_vec = []

    def run_step(self):
        ## Main cerebellum code
        if self.CS is not None: # wait for first input
            self.c_out_share = [0]

            # Step cerebellum a number of times to match CLE timestep
            crs = np.zeros(self.n_steps_per_cle_step, dtype=bool)
            for i in range(self.n_steps_per_cle_step):
                self.c.step(self.CS, self.US)
                self.output = self.c.output
                crs[i] = self.output > self.CR_threshold

                self.c_out_share.append(self.output) # saving of data from each cerebellum timestep
                
                # DEBUG LOG
                if self.debug_logging:
                    self.c_out_vec.append(self.output)
                    self.US_vec.append(self.US)
                    self.CR_vec.append(crs[i])
            self.CR = crs.any() # trigger in this CLE timestep if the threshold was crossed at any time in the duration
 
        # Publish
        trig_msg = Bool()
        trig_msg.data = self.CR
        self.trig_pub.publish(trig_msg)

    def share_module_data(self):
        if self.CS is not None: # wait for first input
            self.module_data = self.c_out_share # share cerebellum output with other IBA modules

    def shutdown(self):
        ## Log time series data
        if self.debug_logging:
            fname = self.basepath + '/resources/cerebellum_log.p'
            pickle.dump( {'US_vec': np.array(self.US_vec),
                'CR_vec': np.array(self.CR_vec),
                'c_out_vec': np.array(self.c_out_vec),
                },
                open( fname, "wb" ) ) 

        # Save model parameters
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
