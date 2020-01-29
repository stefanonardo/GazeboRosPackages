#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, LinkStates
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

def to_point_stamped(frame_id, position):
    return PointStamped(header=Header(stamp=rospy.Time.now(),
                        frame_id=frame_id),
                        point=position)

def get_sphere_marker(frame_id, scale=0.2, transparency=0.7):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.a = transparency
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0
    return marker

#def get_model_state(model_name, reference_frame):
    #model_state = ModelState()
    #model_state.model_name = model_name
    #model_state.pose.orientation.x = 0.0
    #model_state.pose.orientation.y = 0.0
    #model_state.pose.orientation.z = 0.0
    #model_state.pose.orientation.w = 1.0
    #model_state.twist.linear.z = 0.0
    #model_state.twist.angular.x = 0.0
    #model_state.twist.angular.y = 0.0
    #model_state.twist.angular.z = 0.0
    #model_state.reference_frame = reference_frame
    #return model_state

class PubKukaTarget:
    def __init__(self):

        #self.target_base_x = rospy.get_param('~target_base_x', 0.5)
        #self.target_base_y = rospy.get_param('~target_base_y', 0.4)
        #self.target_base_z = rospy.get_param('~target_base_z', 1.3)

        target_frame = rospy.get_param('~target_frame', 'iiwa_link_0')
        self.target_position = to_point_stamped(target_frame, Point(-0.42, 0.18, 1.18))

        #gazebo_target_name = rospy.get_param('~gazebo_target_name', 'target_reaching_subject')
        #self.gazebo_target_link_name = gazebo_target_name + '::' + gazebo_target_name

        #self.pub_gazebo_sphere = rospy.get_param('~pub_gazebo_sphere', False)
        #if self.pub_gazebo_sphere:
            #rospy.wait_for_service('/gazebo/set_model_state')
            #self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        #self.target_model_state = get_model_state(gazebo_target_name, target_frame)

        #self.target_position = None
        #self.get_error_cb = False
        #self.target_set = False
        #self.target_position_index = 0
        #self.target_position_dict = {
            #0: self.set_center,
            #1: self.set_near_up,
            #2: self.set_near_left,
            #3: self.set_near_down,
            #4: self.set_near_right,
            #5: self.set_far_up,
            #6: self.set_far_left,
            #7: self.set_far_down,
            #8: self.set_far_right
        #}

        #self.use_sphere_as_target_position = rospy.get_param('~use_sphere_as_target_position', False)
        #if self.use_sphere_as_target_position:
            #self.get_model_state = rospy.Subscriber('/gazebo/link_states', LinkStates, self.gazebo_link_states_cb, queue_size=1)
        #else:
            #self.set_next_target_position()
            #error_topic = rospy.get_param('~error_topic', '/error')
            #self.error_sub = rospy.Subscriber(error_topic, Float64MultiArray, self.error_cb, queue_size=1)

        target_position_topic = rospy.get_param('~target_position_topic', '/target_position')
        self.target_position_pub = rospy.Publisher(target_position_topic, PointStamped, queue_size=1)

        sphere_marker_pub_topic = rospy.get_param('~sphere_marker_pub_topic', '/sphere_marker')
        self.sphere_marker_pub = rospy.Publisher(sphere_marker_pub_topic, Marker, queue_size=1)
        self.sphere_marker = get_sphere_marker(target_frame)

    def gazebo_link_states_cb(self, link_states):
        sphere_position = None
        try:
            sphere_position = link_states.pose[link_states.name.index(self.gazebo_target_link_name)].position
        except ValueError as e:
            rospy.loginfo(str(e))
            return
        if sphere_position is not None and self.target_model_state.pose.position != sphere_position:
            self.target_position = to_point_stamped(self.target_model_state.reference_frame, sphere_position)

    def set_next_target_position(self):
        if self.target_position_index >= len(self.target_position_dict):
            self.target_position_index = 0
        if self.target_set:
            return
        self.target_position_dict[self.target_position_index]()
        self.target_position_index += 1
        self.target_set = True

    def error_cb(self, error_msg):
        errors = error_msg.data
        non_zero_errors = filter(lambda x : abs(x) > 0, errors)
        if len(non_zero_errors) > 0:
            return
        else:
            self.target_set = False
            self.set_next_target_position()
            rate = rospy.Rate(1)
            rate.sleep()
        if not self.get_error_cb:
            self.get_error_cb = True

    def set_pos(self, x, y, z):
        self.target_model_state.pose.position.x = x + self.target_base_x
        self.target_model_state.pose.position.y = y + self.target_base_y
        self.target_model_state.pose.position.z = z + self.target_base_z
        if self.pub_gazebo_sphere:
            self.set_model_state(self.target_model_state)
        self.target_position = to_point_stamped(self.target_model_state.reference_frame, self.target_model_state.pose.position)

    def set_center(self):
        rospy.loginfo('Setting target positon to: center')
        self.set_pos(0.0, 0.0, 0.0)

    def set_near_left(self):
        rospy.loginfo('Setting target positon to: NEAR left')
        self.set_pos(-0.1, 0.2, 0.0)

    def set_near_right(self):
        rospy.loginfo('Setting target positon to: NEAR right')
        self.set_pos(0.0, -0.25, 0.0)

    def set_near_down(self):
        rospy.loginfo('Setting target positon to: NEAR down')
        self.set_pos(0.0, 0.0, -0.1)

    def set_near_up(self):
        rospy.loginfo('Setting target positon to: NEAR up')
        self.set_pos(0.0, 0.0, 0.2)

    def set_far_left(self):
        rospy.loginfo('Setting target positon to: FAR left')
        self.set_pos(0.0, 0.2, 0.0)

    def set_far_right(self):
        rospy.loginfo('Setting target positon to: FAR right')
        self.set_pos(0.25, -0.25, 0.0)

    def set_far_down(self):
        rospy.loginfo('Setting target positon to: FAR down')
        self.set_pos(0.15, 0.1, -0.15)

    def set_far_up(self):
        rospy.loginfo('Setting target positon to: FAR up')
        self.set_pos(0.15, 0.1, 0.2)

    def publish_target_position(self):
        if self.target_position is not None:
            self.target_position_pub.publish(self.target_position)
            self.sphere_marker.pose.position = self.target_position.point
            self.sphere_marker_pub.publish(self.sphere_marker)

if __name__== '__main__':
    rospy.init_node('pub_target')
    pub_target = PubKukaTarget()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        pub_target.publish_target_position()
        rate.sleep()
