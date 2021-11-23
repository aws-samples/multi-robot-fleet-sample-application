#!/usr/bin/env python3
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

import rospy
import json
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class VelocityManager:
    def __init__(self):
        self.init_pubs()
        self.init_subs()
        self.all_data = {}
        self.all_data['move_base'] = { "x_vel": 0.0, "y_vel": 0.0, "z_ang": 0.0, "last_data_timestamp": rospy.Time.now(), "source": "MOVE_BASE" } 
        self.all_data['mqtt'] = { "x_vel": 0.0, "y_vel": 0.0, "z_ang": 0.0, "last_data_timestamp": rospy.Time.now(), "source": "MQTT" } 

    def init_pubs(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def init_subs(self):
        self.move_base_cmd_vel_sub = rospy.Subscriber("move_base_cmd_vel", Twist, self.move_base_cmd_vel_callback, queue_size=1)
        self.mqtt_cmd_vel_sub = rospy.Subscriber("mqtt_cmd_vel_string", String, self.mqtt_cmd_vel_callback, queue_size=1)

    def move_base_cmd_vel_callback(self, data):
        #rospy.loginfo("{}".format(data))
        self.all_data["move_base"]["x_vel"] = data.linear.x
        self.all_data["move_base"]["y_vel"] = data.linear.y
        self.all_data["move_base"]["z_ang"] = data.angular.z
        self.all_data["move_base"]["last_data_timestamp"] = rospy.Time.now()
        self.all_data["move_base"]["source"] = "MOVE_BASE"

    def mqtt_cmd_vel_callback(self, data):
        mqtt_data = json.loads(data.data)
        rospy.logwarn("Mqtt data loaded is {}".format(mqtt_data))
        self.all_data["mqtt"]["x_vel"] = mqtt_data["x_vel"]
        self.all_data["mqtt"]["y_vel"] = mqtt_data["y_vel"]
        self.all_data["mqtt"]["z_ang"] = mqtt_data["z_ang"]
        self.all_data["mqtt"]["last_data_timestamp"] = rospy.Time.now()
        self.all_data["mqtt"]["source"] = "MQTT"

    def main(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if rospy.Time.now() - self.all_data["mqtt"]["last_data_timestamp"] < rospy.Duration(3):
                # If mqtt data is not older than 3 secs, use it
                vel_dict_to_consider = self.all_data["mqtt"]
            
            elif rospy.Time.now() - self.all_data["move_base"]["last_data_timestamp"] < rospy.Duration(2):
                # If move_base vel is less than 2 secs ago
                vel_dict_to_consider = self.all_data["move_base"]
 
            else:
                vel_dict_to_consider = { "x_vel" : 0, "y_vel" : 0, "z_ang" : 0 }

            cmd_vel_to_send = Twist()
            cmd_vel_to_send.linear.x = vel_dict_to_consider["x_vel"]
            cmd_vel_to_send.linear.y = vel_dict_to_consider["y_vel"]
            cmd_vel_to_send.angular.z = vel_dict_to_consider["z_ang"]

            self.cmd_vel_pub.publish(cmd_vel_to_send)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('velocity_manager')
    node = VelocityManager()
    node.main()
