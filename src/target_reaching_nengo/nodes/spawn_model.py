#!/usr/bin/env python

from gazebo_msgs.srv import SpawnEntity, SpawnEntityRequest
import numpy as np
import rospy
from rospy import ServiceProxy, wait_for_service
import rospkg

model_name = "model"
model_sdf_xml = """
<?xml version='1.0'?>
<sdf version='1.5'>
  <model name='{model_name}'>
    <pose>0 0 0 0 0 0</pose>
    <link name='{model_name}'>
      <inertial>
        <mass>0.057</mass>
      </inertial>
      <collision name='collision'>
        <geometry>
          <box>
            <size>.5 .5 .5</size>
          </box>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <box>
            <size>.5 .5 .5</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/{color}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

class SpawnGazeboSDFModel:
    def __init__(self, model_name, sdf_xml):
        self._model_name = model_name
        self._sdf_spawn_proxy = rospy.ServiceProxy('/gazebo/spawn_sdf_entity',
                                               SpawnEntity, persistent=True)
        self._model_msg = SpawnEntityRequest()
        self._model_msg.entity_name = self._model_name
        self._sdf_xml = sdf_xml
        self._model_colors = ['Green']  #, 'Red', 'Blue', 'Orange', 'Yellow', 'Purple', 'Turquoise']
        #self._model_msg.entity_xml = sdf_xml.format(model_name=model_name, color=np.random.choice(self._model_colors, size=1)[0])
        self._model_msg.initial_pose.position.x = 0.0
        self._model_msg.initial_pose.position.y = 0.0
        self._model_msg.initial_pose.position.z = 0.55
        self._model_msg.reference_frame = "world"

    def spawn(self):
        #self._model_msg.entity_xml = self._sdf_xml.format(model_name=model_name, color=np.random.choice(self._model_colors, size=1)[0])
        self._sdf_spawn_proxy(self._model_msg)
        return 'success'

def main(argv=None):
    rospy.init_node("SpawnGazeboSDFModel")
    rospack = rospkg.RosPack()
    #model_path = rospack.get_path('iiwa_description') + '/sdf/iiwa14.sdf'
    model_path = rospack.get_path('target_reaching_experiments') + '/gazebo_models/target_reaching_subject.sdf'
    with open(model_path, "r") as model_file:
        product_xml = model_file.read()
    spawn_sdf = SpawnGazeboSDFModel("my_model", product_xml)
    rospy.loginfo("SpawnGazeboSDFModel initialized")
    spawn_sdf.spawn()
    rate = rospy.Rate(30)
    #while not rospy.is_shutdown():
        #rate.sleep()

if __name__ == "__main__":
    main()
