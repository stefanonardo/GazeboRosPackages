from gazebo_msgs.msg import JointStates
from std_msgs.msg import Float64

from HopfCpg2 import Hopf
import numpy as np
import rospy
import time

THIGH_R = 'thigh_R'

THIGH_L = 'thigh_L'

SHIN_R = 'shin_R'

SHIN_L = 'shin_L'

SHIN_LOWER_R = 'shin_lower_R'

SHIN_LOWER_L = 'shin_lower_L'

UPPER_ARM_R = 'upper_arm_R'

UPPER_ARM_L = 'upper_arm_L'

FOREARM_R = 'forearm_R'

FOREARM_L = 'forearm_L'

WRIST_R = 'wrist_L'

WRIST_L = 'wrist_R'

NODE_NAME = 'cpg_controller'

TOPIC_NAME = 'joints_angles'


class CpgRosPublisher:

    def __init__(self):
        self._W=np.array([1,1, 1,1, 1,1, 1,1, 1,1, 1,1]) # To scale CPG outputs

        self._hopf = Hopf()
        #self._hopf = Hopf(numOsc=4,h=0.1,alpha=10.,beta=100.,mu=1.5,w_stance=20.,w_swing=2.,b=15.,F=500,feedback=1,gait=0,time_interval=1e-3,KK=2.)

        self._ros_communicator = None
        self._joints_angles =  {WRIST_L:0,
                                WRIST_R:0,
                                FOREARM_L:0,
                                FOREARM_R:0,
                                UPPER_ARM_L:0,
                                UPPER_ARM_R:0,
                                SHIN_LOWER_L:0,
                                SHIN_LOWER_R:0,
                                SHIN_L:0,
                                SHIN_R:0,
                                THIGH_L:0,
                                THIGH_R:0, }
        self._ros_communicator = self._init_ros_communicator()

        self.param_0 = 35.726967357221966
        self.param_1 = 32.37623408752182
        self.param_2 = 33.32390979478251

    def _init_ros_communicator(self):
        pub = rospy.Publisher(TOPIC_NAME, JointStates, queue_size=10)
        rospy.init_node(NODE_NAME, anonymous=True) # , log_level=rospy.INFO, disable_rostime=False, disable_rosout=False, disable_signals=False)
        return pub

    def _output(self, theta):
        return self._hopf.output(theta)

    def _iterate(self, record):
        self._hopf.iterate(record)

    def update_joints_angles(self):
        self._iterate(1)
        #for i in range(0,19):
        #    self._iterate(1)

        #sol[iter_num]=[35.726967357221966, 32.37623408752182, 33.32390979478251] # good forelimb movement

        ################## Good Solutions #############################################
        #sol[iter_num]=[44.088288177553075, 48.32771467664401, 49.837549072058955]
        #sol[iter_num]=[38.30034932498564, 39.40135279952095, 41.57580154142303]
        #sol[iter_num]=[34.918902319724594, 35.480521137532726, 37.312393133772865]
        #sol[iter_num]=[38.515436495270855, 37.92404585515604, 38.886806900956714]
        #sol[iter_num]=[39.33195329541886, 38.671654800521566, 40.33367542244673]
        #sol[iter_num]=[30.199275815199094, 35.27451821741849, 37.619997203859356]
        #sol[iter_num]=[31.09382155810416, 35.329746870272885, 37.382940536172114]
        #sol[iter_num]=[38.76707318910695, 36.70915821508438, 41.740970135881135]
        #sol[iter_num]=[35.836551964584345, 38.261542134264765, 38.806209713194974]
        #sol[iter_num]=[33.79255714474338, 35.784899298125524, 41.51244075801343]
        #sol[iter_num]=[38.661965513825415, 38.5521316658321, 38.52560174606816]
        #sol[iter_num]=[33.36433834795138, 34.01895302178556, 39.86310650847313] # ***
        #sol[iter_num]=[36.25712482055094, 40.72082845839144, 45.8131340448264] # ***
        #sol[iter_num]=[35.95293636897409, 37.6522863083644, 40.460397705903894] # ***
        #sol[iter_num]=[36.91218484840258, 38.644333277494134, 41.991003324255765] # **
        #sol[iter_num]=[37.336518536381135, 38.49114108706281, 41.022320502576825]#**
        #sol[iter_num]=[35.726967357221966, 32.37623408752182, 33.32390979478251] # good forelimb movement
        #sol[iter_num]=[37.018671098831796, 35.50199670084039, 36.31114802783345]
        #sol[iter_num]=[56.58107205263647, 39.52357242848775, 39.16020275187317]# hind legs standing
        #sol[iter_num]=[52.84444362840668, 51.26648249091807, 53.53713396091275]#7.52
        #sol[iter_num]=[46.133945073226016, 45.401545978467404, 49.461338809091764]# 5.64
        #sol[iter_num]=[52.11203590174404, 51.784622844277955, 51.28631653726013]
        #sol[iter_num]=[48.34876927351339, 52.54013208320516, 54.00200388652281]
        ###############################################################################

        y = self._output(self.param_0) + self._output(self.param_1) + self._output(self.param_2)
        #y = self._output(44.088288177553075) + self._output(48.32771467664401) + self._output(49.837549072058955)
        for i in range(len(y)):
            y[i] = 1 - y[i] # toggle the oscillator outputs: in this way i get a better locomotion

        yy = np.hstack((y[0],y[2],y[4],y[6],y[8],y[10],y[1],y[3],y[5],y[7],y[9],y[11])) # Combine CPG outputs to create referans
        angles = self._W * yy # Generate reference by scaled CPG outputs
        self._joints_angles[WRIST_L] = angles[0]
        self._joints_angles[WRIST_R] = angles[1]
        self._joints_angles[FOREARM_L] = angles[2]
        self._joints_angles[FOREARM_R] = angles[3]
        self._joints_angles[UPPER_ARM_L] = angles[4]
        self._joints_angles[UPPER_ARM_R] = angles[5]
        self._joints_angles[SHIN_LOWER_L] = angles[6]
        self._joints_angles[SHIN_LOWER_R] = angles[7]
        self._joints_angles[SHIN_L] = angles[8]
        self._joints_angles[SHIN_R] = angles[9]
        self._joints_angles[THIGH_L] = angles[10]
        self._joints_angles[THIGH_R] = angles[11]

    def get_joint_angle(self,angle_key):
        # TODO handle no such key in the dict
        return self._joints_angles[angle_key]

    def get_all_joints_angles(self):
        return  self._joints_angles.values()

    def _prepare_ros_message(self):
        message = JointStates()
        for joint, angle in self._joints_angles.iteritems():
            message.name.append(joint)
            message.position.append(angle)

        return message

    def publish_all_joints_angles(self):
        all_joints_angles = self._prepare_ros_message()
        self._ros_communicator.publish(all_joints_angles)

    def publish_angles_to_mouse_model(self):
        for joint, angle in self._joints_angles.iteritems():
            message = Float64()
            message.data = angle
            message_publisher = rospy.Publisher('cheesy5/' + joint  + '/cmd_pos', Float64, queue_size=10)
            message_publisher.publish(message)
            #message_publisher_vel = rospy.Publisher('cheesy5/' + joint  + '/cmd_vel', Float64, queue_size=10)
            #message_publisher_vel.publish(message)
            #time.sleep(0.02)

    def shutdown_node(self):
        rospy.signal_shutdown("ros_cpg_wrapper shutdown requested")


