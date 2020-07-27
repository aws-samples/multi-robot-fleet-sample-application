#!/usr/bin/env python
#
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

"""
This nodes job is to collect position info and sends it over /client_robot_data via rosbridge.
To grab data of all other robots as a client, subscribe to /client_robot_data and remap
it to /remapped_client_robot_data. If you are a server, you have all the data in
/client_robot_data already
"""

import rospy
import roslibpy
import json
from std_msgs.msg import String
import tf
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates


class SendData:
    def __init__(self):
        self.data_to_rosbridge = {}
        self.data_to_rosbridge['name'] = rospy.get_param('ROBOT_NAME')
        self.data_to_rosbridge['robot_sdf_file'] = rospy.get_param('ROBOT_SDF_FILE')
        self.data_to_rosbridge['navigation_pose'] = {}
        self.data_to_rosbridge['gazebo_pose'] = {}
        self.rosbridge_ip = rospy.get_param('ROSBRIDGE_IP')

        self.rosbridge_state = rospy.get_param('ROSBRIDGE_STATE')
        self.client = roslibpy.Ros(host=self.rosbridge_ip, port=9090)
        self.client.run()

        if self.rosbridge_state == 'CLIENT':
            self.remap_pub = rospy.Publisher('remapped_client_robot_data', String, queue_size=1)
            self.robot_data_listener = roslibpy.Topic(self.client, 'client_robot_data', 'std_msgs/String')
            self.robot_data_listener.subscribe(self.remap_subscriber)

        self.gazebo_model_state_sub = rospy.Subscriber('gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)
        self.current_model_state = None
        self.init_rosbridge_talkers()

    def model_states_callback(self, msg):
        self.current_model_state = msg

    def init_rosbridge_talkers(self):
        self.talker = roslibpy.Topic(self.client, 'client_robot_data', 'std_msgs/String')

    def remap_subscriber(self, msg):
        data = msg['data']
        self.remap_pub.publish(data)

    def main(self):
        rate = rospy.Rate(10.0)
        listener = tf.TransformListener()

        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('map', '/base_link', rospy.Time(0))
                nav_pose = {}
                nav_pose['x'] = trans[0]
                nav_pose['y'] = trans[1]
                nav_pose['z'] = trans[2]

                nav_pose['qx'] = rot[0]
                nav_pose['qy'] = rot[1]
                nav_pose['qz'] = rot[2]
                nav_pose['qw'] = rot[3]

                self.data_to_rosbridge['navigation_pose'] = nav_pose
                
                gazebo_pose = {} 
                gazebo_model_index = self.current_model_state.name.index("/")  # Looking for main robot which in under namespace "/"
                gz_pose = self.current_model_state.pose[gazebo_model_index]
                gazebo_pose['x'] = gz_pose.position.x
                gazebo_pose['y'] = gz_pose.position.y
                gazebo_pose['z'] = gz_pose.position.z

                gazebo_pose['qx'] = gz_pose.orientation.x
                gazebo_pose['qy'] = gz_pose.orientation.y
                gazebo_pose['qz'] = gz_pose.orientation.z
                gazebo_pose['qw'] = gz_pose.orientation.w
                self.data_to_rosbridge['gazebo_pose'] = gazebo_pose

                self.talker.publish(roslibpy.Message( {'data': json.dumps(self.data_to_rosbridge)} ))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("[client_rosbridge] TF exception in gathering current position")

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('client_rosbridge')
    send_data = SendData()
    send_data.main()
