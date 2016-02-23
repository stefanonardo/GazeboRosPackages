'''
Implementation of Hopf oscillator that is in the paper that is 
"Pattern generators with sensory feedback for the control of
quadruped locomotion", Ludovic Righetti and Auke Jan Ijspeert

numOsc: Number of oscillators 
h: step coefficient of forward Euler
alpha, beta, mu, w_stance, w_swing, b, F oscillator parameters from the paper
feedback: Indicates that whether the force feedback (F: force) will be used
gait: There are four different gaits determined by coupling matrix K
time_interval: Sampling time of oscillator outputs
KK: Coefficient of coupling matrix

The initial values of oscillators are random 
@author:Emec Ercelik
'''
import numpy as np
import matplotlib
matplotlib.use('Agg') # Must be before importing matplotlib.pyplot or pylab!
import matplotlib.pyplot as plt
import time as ttt

import rospy
import sys

class Hopf:
    def __init__(self,numOsc=4,h=1e-3,alpha=5.,beta=50.,mu=1.,w_stance=10.,w_swing=1.,b=10.,F=300,\
                 feedback=0,gait=0,time_interval=1e-3,KK=1.):
        self.h=h # Step size of forward Euler
        self.numOsc=numOsc # number of oscillator outputs
        self.alpha=alpha # oscollator parameter
        self.beta=beta # Oscillator parameter
        self.mu=mu # amplitude of oscillation
        self.w_stance=w_stance # Changes the frequency of stance phase
        self.w_swing=w_swing # Changes the frequency of stance phase
        self.b=b # Changes the shape of periodic signal
        self.F=F # Feedback force 
        self.feedback=feedback # If the feedback network will be used equals to 1
        self.iter_num=int(time_interval/h) # calculates how many iteration is need to sample once in time_interval
        self.KK=KK # The scale variable of coupling matrix
        self.maxx=0. # stores the maximum value during oscillation
        self.minn=1. # stores minimum value during oscillation
        self.gait=gait

        self.K = self._initialize_gait(numOsc)

        self._initialize_oscillation_variables()
        
        # Arrays to record the state variables
        self.xRec=np.array(self.x)
        self.yRec=np.array(self.y)
        self.rRec=np.array(self.r)

        rospy.loginfo("Hopf oscillator init; about to call _synchronize_oscillator")
        self._oscillator_synchronized = False
        self._synchronize_oscillator()
        rospy.loginfo("_synchronize_oscillator is finished")

    def _synchronize_oscillator(self):
        # Solve oscillator network for 200 iterations to stablize it without recording
        rospy.loginfo("Hopf oscillator init; 200 iterations without recording")
        for i in range(200):
            self.iterate(0)

        rospy.loginfo("Hopf oscillator init; 100 iterations with recording")
        # Solve oscillator for 100 and store in order to use for delayed networks
        for i in range(100):
            self.iterate(1)
            self._oscillator_synchronized = True

    def _initialize_oscillation_variables(self):
        # Initialization of oscillator variables
        self.x = np.zeros((self.numOsc, 1)) + np.random.rand(self.numOsc, 1)
        self.y = np.zeros((self.numOsc, 1)) + np.random.rand(self.numOsc, 1)
        self.u = np.zeros((self.numOsc, 1))
        self.r = np.sqrt(self.x ** 2 + self.y ** 2)
        self.w = self.w_stance / (np.exp(-self.b * self.y) + 1) + self.w_swing / (np.exp(self.b * self.y) + 1)

    def _initialize_gait(self, numOsc):
        # Coupling matrices for different gaits
        self.gaitType=['Trot','Pace','Bound','Walk']
        if self.numOsc == 4:
            if self.gait == 0:
                K = self.KK * np.array([[0, -1, -1, 1], [-1, 0, 1, -1], [-1, 1, 0, -1], [1, -1, -1, 0]])  # Trot
            elif self.gait == 1:
                K = self.KK * np.array([[0, -1, 1, -1], [-1, 0, -1, 1], [1, -1, 0, -1], [-1, 1, -1, 0]])  # Pace
            elif self.gait == 2:
                K = self.KK * np.array([[0, 1, -1, -1], [1, 0, -1, -1], [-1, -1, 0, 1], [-1, -1, 1, 0]])  # Bound
            else:
                K = self.KK * np.array([[0, -1, 1, -1], [-1, 0, -1, 1], [-1, 1, 0, -1], [1, -1, -1, 0]])  # Walk
        else:
            K = self.KK * (1 - np.eye(numOsc))
        return K

    # Solve the differential equations of Hopf oscillator by forward Euler method
    def iterate(self,Record=0):
        #if not self._oscillator_synchronized:
        #    self._synchronize_oscillator()
        #    self._oscillator_synchronized = True

        if self.feedback==1:
            for ii in range(self.numOsc):
                if (self.y[ii] < 0.25 * self.r[ii]) and (self.y[ii] > -0.25 * self.r[ii]):
                    self.u[ii] = -self.w[ii] * self.x[ii] - self.K.dot(self.y)[ii]
                elif (self.y[ii] > 0. and self.x[ii] < 0.) or (self.y[ii] < 0. and self.x[ii] > 0.):
                    self.u[ii] = -np.sign(self.y[ii]) * self.F
            else:
                self.u[ii] = 0.
        for i in range(self.iter_num):
            try:
                temp_x = self.h * (self.alpha * (self.mu-self.r**2) * self.x - self.w * self.y)
                self.y += self.h * (self.beta * (self.mu-self.r**2) * self.y + self.w * self.x + self.K.dot(self.y) + self.u)
                self.x += temp_x
                self.w = self.w_stance / (np.exp(-self.b * self.y) + 1) + self.w_swing / (np.exp(self.b * self.y) + 1)
                self.r = np.sqrt(self.x**2 + self.y**2)
                for i in range(self.numOsc):
                    self.maxx = np.max((self.maxx,self.y[i]))
                    self.minn = np.min((self.minn,self.y[i]))
            except:
                pass

        self.Record = Record
        if self.Record == 1:
            self.xRec = np.hstack((self.xRec,self.x))
            self.yRec = np.hstack((self.yRec,self.y))
            self.rRec = np.hstack((self.rRec,self.r))

    def plot(self,plot_type=2,save_fig=1,f_val=-1.,vel=-1.):
        if self.Record==1:
            timeStop=self.xRec.shape[1]-1
            time=np.arange(0,timeStop+1)
            pltxAxisStep=(timeStop)/10
            if plot_type==1:
                ####Plot1##############################
                plt.figure(1,figsize=(15,10))
                plt.suptitle('Hopf Oscillator\n'+\
                'h={0:.3f}, alpha={1:.2f}, beta={2:.2f},'.format(self.h,self.alpha,self.beta)+\
                ' mu={0:.2f}, w_stance={1:.2f}, w_swing={2:.2f}, b={3:.2f}'\
                .format(self.mu,self.w_stance,self.w_swing,self.b)+\
                '\nGait Type={0:s}'.format(self.gaitType[self.gait]))
                plt.subplot(421)
                plt.plot(time,self.xRec[0,:],'b',label='x1')
                plt.xticks(np.arange(0,timeStop,pltxAxisStep))
                plt.xlabel('Time step')
                plt.ylabel('Value of state variables')
                plt.legend()
                plt.subplot(423)
                plt.plot(time,self.xRec[1,:],'k',label='x2')
                plt.xticks(np.arange(0,timeStop,pltxAxisStep))
                plt.legend()
                plt.subplot(425)
                plt.plot(time,self.xRec[2,:],'g',label='x3')
                plt.xticks(np.arange(0,timeStop,pltxAxisStep))
                plt.legend()
                plt.subplot(427)
                plt.plot(time,self.xRec[3,:],'m',label='x4')
                plt.xticks(np.arange(0,timeStop,pltxAxisStep))
                plt.legend()
                plt.xlabel('Time step')
                plt.ylabel('Value of state variables')
                plt.subplot(422)
                plt.plot(time,self.yRec[0,:],'b',label='y1')
                plt.xticks(np.arange(0,timeStop,pltxAxisStep))
                plt.legend()
                plt.subplot(424)
                plt.plot(time,self.yRec[1,:],'k',label='y2')
                plt.xticks(np.arange(0,timeStop,pltxAxisStep))
                plt.legend()
                plt.subplot(426)
                plt.plot(time,self.yRec[2,:],'g',label='y3')
                plt.xticks(np.arange(0,timeStop,pltxAxisStep))
                plt.legend()
                plt.subplot(428)
                plt.plot(time,self.yRec[3,:],'m',label='y4')
                plt.xticks(np.arange(0,timeStop,pltxAxisStep))
                plt.legend()
            elif plot_type==2:
                ####Plot2##############################
                plt.figure(2,figsize=(15,10))
                plt.suptitle('Hopf Oscillator\n'+\
                'h={0:.3f}, alpha={1:.2f}, beta={2:.2f},'.format(self.h,self.alpha,self.beta)+\
                ' mu={0:.2f}, w_stance={1:.2f}, w_swing={2:.2f}, b={3:.2f}'\
                .format(self.mu,self.w_stance,self.w_swing,self.b)+\
                '\nGait Type={0:s}, f_val={1:.2f}, vel={2:.2f}'.format(self.gaitType[self.gait],f_val,vel))
                plt.subplot(211)
                plt.plot(time,self.xRec[0,:],'b',label='x1')
                plt.plot(time,self.xRec[1,:],'k',label='x2')
                plt.plot(time,self.xRec[2,:],'g',label='x3')
                plt.plot(time,self.xRec[3,:],'m',label='x4')
                plt.xticks(np.arange(0,timeStop,pltxAxisStep))
                plt.xlabel('Time step')
                plt.ylabel('Value of state variables')
                plt.legend()
                plt.subplot(212)
                plt.plot(time,self.yRec[0,:],'b',label='y1')
                plt.plot(time,self.yRec[1,:],'k',label='y2')
                plt.plot(time,self.yRec[2,:],'g',label='y3')
                plt.plot(time,self.yRec[3,:],'m',label='y4')
                plt.xticks(np.arange(0,timeStop,pltxAxisStep))
                plt.xlabel('Time step')
                plt.ylabel('Value of state variables')
                plt.legend()
            elif plot_type==3:
                ####Plot3##############################
                plt.figure(3)
                plt.plot(self.rRec[0,:])
                plt.plot(self.rRec[1,:])
                plt.plot(self.rRec[2,:])
                plt.plot(self.rRec[3,:])
            else:
                print('Wrong Figure Type!')
                
            if save_fig==1:
                plt.savefig('hopf_osc_mouse'+str(ttt.clock())+'.png')
                plt.clf()
            elif save_fig==0:
                plt.show()
                pass
        else:
            print('No records!')
    # Outputs the oscillator state with a given delay and scaled into 0-1 range            
    def output(self,theta=0):
        y=np.zeros((4,1)) 
        theta=int(np.abs(theta))
        try:
            y=self.yRec[:,-(theta+1)]
        except:
            y=self.y
            print('No delayed or recorded data!')
        output=[]
        temp_y=(y-self.minn)/(self.maxx-self.minn)
        for i in range(self.numOsc):
            output.append(float(temp_y[i]))
        return output

# Example Implementation
##kwargs={'numOsc':4,'h':1e-3,'alpha':5.,'beta':50.,'mu':1.,'w_stance':10.,'w_swing':4.,'b':10.,'F':300,\
##                 'feedback':0,'gait':0}
##
##osc=Hopf(**kwargs)
##xRec=np.array(osc.x)
##yRec=np.array(osc.y)
##rRec=np.array(osc.r)
##stopTime=20000
##timeStop=stopTime
##for t in range(stopTime):
##    osc.iterate(1)
##
##osc.plot()W

