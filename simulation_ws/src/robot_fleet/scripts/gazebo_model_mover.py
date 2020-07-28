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
This node subscribes to rostopics sent over rosbridge from clients.
It then sets up the corresponding static gazebo model that has the move plugin attached
and also publishes the rostopic that the plugin subscribes to move it.

The topic that it subscribes to is call /client_robot_data for robots running with
rosbridge clients. It connects to /remapped_client_robot_data in case of server.

It has the capability to recognize stale data and remove the objects.
"""

import rospy
import json
import rospkg
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from subprocess32 import check_output
from gazebo_msgs.srv import SpawnModelRequest,SpawnModel, DeleteModel, DeleteModelRequest
from gazebo_msgs.msg import ModelState


class MasterGazeboMover:
    def __init__(self):
        self.robot_name = rospy.get_param('ROBOT_NAME')
        self.rosbridge_state = rospy.get_param('ROSBRIDGE_STATE')
        self.sdf_file_path = rospy.get_param('ROBOT_SDF_FILE')
        self.use_custom_move_plugin = rospy.get_param('use_custom_move_object_gazebo_plugin')
        self.init_pubs()
        self.init_subs()
        self.all_data = {}
        self.stale_msg_cleaner_timer = rospy.Timer(rospy.Duration(5) , self.timer_callback)
        self.rospack = rospkg.RosPack()

        if self.rosbridge_state == 'SERVER':
            self.robot_data_sub = rospy.Subscriber('client_robot_data', String, self.data_callback, queue_size=1)
        elif self.rosbridge_state == 'CLIENT':
            self.robot_data_sub = rospy.Subscriber('remapped_client_robot_data', String, self.data_callback, queue_size=1)
        else:
            rospy.logerr("Unexpected ROSBRIDGE_STATE. Ensure either SERVER or CLIENT")

        self.path_xacro = check_output(["which", "xacro" ]).strip('\n')

    def init_pubs(self):
        if not self.use_custom_move_plugin:
            self.gazebo_set_state_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        self.all_active_data_pub = rospy.Publisher('all_active_data_debug', String, queue_size=1)

    def timer_callback(self, event):
        """In this callback, we find out all the stale data and remove the corresponding models."""
        if bool(self.all_data):  # if dict not empty
            for name, data_w_time in self.all_data.iteritems():
                if rospy.Time.now() - data_w_time[1] > rospy.Duration(10):  # If data is staler than 30 secs
                    rospy.logwarn("Removing {} from data since its stale".format(name))
                    self.all_data.pop(name)
                    
                    request = DeleteModelRequest()
                    request.model_name = name

                    response =  self.delete_model(request)
                    rospy.loginfo(response)
                    break  # Since we are popping an element, it will mess up the for loop. Remove one at a time.

    def create_pub(self, robot_name):
        return rospy.Publisher(robot_name + '/topic_to_move', Pose, queue_size=1)

    def init_subs(self):
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        self.add_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        rospy.wait_for_service('/gazebo/delete_model')
        self.delete_model = rospy.ServiceProxy('/gazebo/delete_model',DeleteModel)


    def create_model(self, name, xml_file_name, pose_dict ):
        rospy.loginfo("Creating model {}".format(name))

        xml_model = check_output([ self.path_xacro, self.sdf_file_path ]).strip('\n')

        request = SpawnModelRequest()
        request.model_name = name
        request.model_xml = xml_model
        request.robot_namespace = ''
        request.reference_frame = ''

        request.initial_pose.position.x = pose_dict['x']
        request.initial_pose.position.y = pose_dict['y']
        request.initial_pose.position.z = pose_dict['z']

        request.initial_pose.orientation.x = pose_dict['qx']
        request.initial_pose.orientation.y = pose_dict['qy']
        request.initial_pose.orientation.z = pose_dict['qz']
        request.initial_pose.orientation.w = pose_dict['qw']

        response =  self.add_model(request)
        rospy.loginfo(response)

    def data_callback(self, msg):
        """ This is the callback that gets called by data from all robots. We just update a datastructure
            with the latest data, and create an entry if it doesn't exist."""
        data = json.loads(msg.data)

        # Ignore data from current robot since it's already available
        if data['name'] == self.robot_name:
            pass
        elif self.all_data.get( data['name'], None) is None:
            self.create_model(data['name'], data['robot_sdf_file'], data['gazebo_pose'] )
            if self.use_custom_move_plugin:
                self.all_data[data['name']] = [ data, rospy.Time.now(), self.create_pub(data['name']) ]
            else:
                self.all_data[data['name']] = [ data, rospy.Time.now() ]
        else:
            self.all_data[data['name']][0] = data
            self.all_data[data['name']][1] = rospy.Time.now()

    def main(self):
        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            all_active_data = {}
            for name, data_w_timestamp in self.all_data.iteritems():
                data = data_w_timestamp[0]
                gazebo_pose = data['gazebo_pose']

                robot_pose =  Pose()
                robot_pose.position.x = gazebo_pose['x']
                robot_pose.position.y = gazebo_pose['y']
                robot_pose.position.z = gazebo_pose['z']
                robot_pose.orientation.x = gazebo_pose['qx']
                robot_pose.orientation.y = gazebo_pose['qy']
                robot_pose.orientation.z = gazebo_pose['qz']
                robot_pose.orientation.w = gazebo_pose['qw']

                if self.use_custom_move_plugin:
                    publisher = data_w_timestamp[2]
                    publisher.publish(robot_pose)
                else:
                    model_state = ModelState()
                    model_state.model_name = name
                    model_state.pose = robot_pose
                    self.gazebo_set_state_pub.publish(model_state)

                all_active_data[data['name']] = data
                
            self.all_active_data_pub.publish(json.dumps(all_active_data))
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('gazebo_model_mover')
    master = MasterGazeboMover()
    master.main()
