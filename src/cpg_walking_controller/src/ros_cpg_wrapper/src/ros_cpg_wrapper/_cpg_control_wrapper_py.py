from StringIO import StringIO

import rospy
from std_msgs.msg import Int64

from ros_cpg_wrapper._cpg_control_wrapper_cpp import CpgControllerWrapper
from ros_cpg_wrapper.CpgRosPublisher import CpgRosPublisher

class CpgController(object):
    def __init__(self):
        self._cpg_controller = CpgControllerWrapper()
        self._cpg_ros_publisher = CpgRosPublisher()

    def _to_cpp(self, msg):
        """Return a serialized string from a ROS message

        Parameters
        ----------
        - msg: a ROS message instance.
        """
        buf = StringIO()
        msg.serialize(buf)
        return buf.getvalue()

    def _from_cpp(self, str_msg, cls):
        """Return a ROS message from a serialized string

        Parameters
        ----------
        - str_msg: str, serialized message
        - cls: ROS message class, e.g. sensor_msgs.msg.LaserScan.
        """
        msg = cls()
        return msg.deserialize(str_msg)

    def GetNumIterations(self):
        """Get number of iterations

        Return a std_msgs/Int64 instance.

        Parameters
        ----------
        """
        str_numIterations = self._cpg_controller.GetNumIterations()
        return self._from_cpp(str_numIterations, Int64)

    def GetCpgParameter0(self):
        """Get CPG parameter 0

        Return std_msgs/Float64 with CPG parameter 0

        Parameters
        ----------
        None
        """
        str_cpgParam = self._cpg_controller.GetCpgParameter0()
        return self._from_cpp(str_cpgParam, Float64)
        
    def GetCpgParameter1(self):
        """Get CPG parameter 1

        Return std_msgs/Float64 with CPG parameter 1

        Parameters
        ----------
        None
        """
        str_cpgParam = self._cpg_controller.GetCpgParameter1()
        return self._from_cpp(str_cpgParam, Float64)
        
    def GetCpgParameter2(self,msg):
        """Get CPG parameter 2

        Return std_msgs/Float64 with CPG parameter 2

        Parameters
        ----------
        None
        """
        str_cpgParam = self._cpg_controller.GetCpgParameter2()
        return self._from_cpp(str_cpgParam, Float64)