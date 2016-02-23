#!/usr/bin/python

import numpy as np
import rospy
import sys
from gazebo_msgs.msg import JointStates
import time

import actionlib
import ros_cpg_wrapper.msg
import ros_cpg_wrapper.msg

from CpgRosPublisher import CpgRosPublisher
from ros_cpg_wrapper import _cpg_control_wrapper_py


# from dynamic_reconfigure.server import Server as DynamicReconfigureServer
# from cpg_ros_node.cfg import cpgRosNodeConfig as ConfigType

class RosCpgWrapperAction(object):
    # create messages that are used to publish feedback/result
    _feedback = ros_cpg_wrapper.msg.ros_cpg_wrapperFeedback()
    _result = ros_cpg_wrapper.msg.ros_cpg_wrapperResult()

    def __init__(self, name, node):
        rospy.loginfo("Start action server...")
        self._action_name = name
        self._node = node
        self._as = actionlib.SimpleActionServer(self._action_name, ros_cpg_wrapper.msg.ros_cpg_wrapperAction,
                                                execute_cb=self.action_callback, auto_start=False)

        self._as_parameters = actionlib.SimpleActionServer(self._action_name + "_Parameters",
                                                           ros_cpg_wrapper.msg.ros_cpg_parametersAction,
                                                           execute_cb=self.parameters_callback, auto_start=False)

    def action_callback(self, goal):
        rospy.loginfo("action_callback for ros_cpg_wrapper")
        if goal.MessageID == "pause_cpg":
            # decide whether CPG should pause, resume or shutdown
            rospy.loginfo('Pausing CPG run')
            self._node.pause_cpg = True
        elif goal.MessageID == "start_cpg":
            # resume the CPG
            rospy.loginfo('Resuming CPG run')
            self._node.pause_cpg = False
        elif goal.MessageID == "shutdown_cpg":
            rospy.loginfo("Shutdown CPG wrapper")
            self._node.pause_cpg = True
            self._node.cpg_ros_publisher.shutdown_node()
        else:
            rospy.loginfo('goal.MessageId is not valid: %s', goal.MessageID)

        self._as.set_succeeded()

    def parameters_callback(self, goal):
        rospy.loginfo("parameters_callback for ros_cpg_wrapper")
        if len(goal.cpg_parameters) == 3:
            rospy.loginfo("goal.cpg_parameters OK. New parameters are: %f, %f, %f", goal.cpg_parameters[0],
                          goal.cpg_parameters[1], goal.cpg_parameters[2])
            self._node.cpg_param_0 = goal.cpg_parameters[0]
            self._node.cpg_param_1 = goal.cpg_parameters[1]
            self._node.cpg_param_2 = goal.cpg_parameters[2]
        else:
            rospy.loginfo('goal.cpg_parameters is not valid')

        self._as_parameters.set_succeeded()

    def start_server(self):
        self._as.start()
        self._as_parameters.start()
        rospy.loginfo('Action_Server is up')


class CpgRosNode:
    def __init__(self):
        rospy.loginfo("Init CpgRosNode")
        self.cpg_ros_publisher = CpgRosPublisher()
        self.rate = float(rospy.get_param('~rate', '250.0'))
        self._cpg_control_wrapper = _cpg_control_wrapper_py.CpgController()
        # Create a dynamic reconfigure server.
        # self.server = DynamicReconfigureServer(ConfigType, self.reconfigure)
        self.iterations = self._cpg_control_wrapper.GetNumIterations().data
        self.pause_iterations = 10
        self.pause_cpg = False

        rospy.loginfo("Start action_server in CpgRosNode")

        self.as_instance = RosCpgWrapperAction(rospy.get_name(), self)

        self.cpg_param_0 = 35.726967357221966
        self.cpg_param_1 = 32.37623408752182
        self.cpg_param_2 = 33.32390979478251

        self.as_instance.start_server()

        rospy.loginfo("Start server")

        while not rospy.is_shutdown():
            # rospy.loginfo("Loop at rate %i", self.rate)
            if (self.pause_cpg == True):
                if self.rate:
                    rospy.sleep(1 / self.rate)
                else:
                    rospy.sleep(1 / 50.0)

            else:
                # rospy.loginfo("Iterations for CPG controller: %s -- as int: %i", str(self._cpg_control_wrapper.GetNumIterations()), self._cpg_control_wrapper.GetNumIterations().data)

                while self.iterations > 0:
                    # rospy.loginfo("pause_cpg flag = %i", self.pause_cpg)
                    if (self.pause_cpg == True):
                        rospy.loginfo("Pause CPG, cancel loop")
                        break
                    # iter_start = rospy.Time.now()

                    if self.cpg_param_0 != self.cpg_ros_publisher.param_0:
                        self.cpg_ros_publisher.param_0 = self.cpg_param_0

                    if self.cpg_param_1 != self.cpg_ros_publisher.param_1:
                        self.cpg_ros_publisher.param_1 = self.cpg_param_1

                    if self.cpg_param_2 != self.cpg_ros_publisher.param_2:
                        self.cpg_ros_publisher.param_2 = self.cpg_param_2

                    self.cpg_ros_publisher.update_joints_angles()
                    # iter_intermediate_1 = rospy.Time.now()
                    # iter_update_angles = iter_intermediate_1 - iter_start
                    # rospy.loginfo("Duration update_joint_angles: %i sec., %i ms.", iter_update_angles.secs, (iter_update_angles.nsecs / 1000000))
                    self.iterations = self.iterations - 1
                    # rospy.loginfo("Remaining iterations in current run: %i; ran one iteration of CPG controller", self.iterations)
                    self.cpg_ros_publisher.publish_angles_to_mouse_model()
                    # iter_end = rospy.Time.now()
                    # iter_difference = iter_end - iter_start
                    # rospy.loginfo("Duration of step: %i sec., %i ms.", iter_difference.secs, (iter_difference.nsecs / 1000000))

                if self.iterations == 0:
                    rospy.loginfo("Current CPG run is through. Restart...")
                    # rospy.loginfo("Pause for %i iterations", self.pause_iterations)
                    # while self.pause_iterations > 0:
                    #    self.pause_iterations = self.pause_iterations - 1
                    #    if self.rate:
                    #        rospy.sleep(1/self.rate)
                    #    else:
                    #        rospy.sleep(1/50.0)

                    self.iterations = self._cpg_control_wrapper.GetNumIterations().data
                    # self.pause_iterations = 10

                if self.rate:
                    rospy.sleep(1 / self.rate)
                else:
                    rospy.sleep(1 / 50.0)

                    # Create a callback function for the dynamic reconfigure server.
                    # def reconfigure(self, config, level):
                    #    # Fill in local variables with values received from dynamic reconfigure clients (typically the GUI).
                    #    self.iterations = config['iterations']
                    #    self.cpg_param_0 = config['cpg_param_0']
                    #    self.cpg_param_1 = config['cpg_param_1']
                    #    self.cpg_param_2 = config['cpg_param_2']

                    #    # Return the new variables.
                    #    return config


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    # rospy.init_node('CpgRosNode')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = CpgRosNode()
    except rospy.ROSInterruptException:
        pass
