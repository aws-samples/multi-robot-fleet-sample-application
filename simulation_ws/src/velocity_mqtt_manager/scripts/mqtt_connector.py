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

import json
import ast
import rospy
from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient
import rospkg
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class MqttManager:
    def __init__(self):
        self.rospack = rospkg.RosPack()
        path_for_certs = self.rospack.get_path("velocity_mqtt_manager") + "/certs/"
        path_for_config = self.rospack.get_path("velocity_mqtt_manager") + "/config/iot_config.json"


        with open(path_for_config) as f:
          cert_data = json.load(f)

        # Init AWSIoTMQTTClient
        self.myAWSIoTMQTTClient = None
        self.myAWSIoTMQTTClient = AWSIoTMQTTClient("basicPubSub")
        self.myAWSIoTMQTTClient.configureEndpoint(cert_data["endpoint"], 443)
        self.myAWSIoTMQTTClient.configureCredentials( path_for_certs+ cert_data["rootCAPath"], path_for_certs+cert_data["privateKeyPath"], path_for_certs+cert_data["certificatePath"])

        # AWSIoTMQTTClient connection configuration
        self.myAWSIoTMQTTClient.configureAutoReconnectBackoffTime(1, 32, 20)
        self.myAWSIoTMQTTClient.configureOfflinePublishQueueing(-1)  # Infinite offline Publish queueing
        self.myAWSIoTMQTTClient.configureDrainingFrequency(2)  # Draining: 2 Hz
        self.myAWSIoTMQTTClient.configureConnectDisconnectTimeout(10)  # 10 sec
        self.myAWSIoTMQTTClient.configureMQTTOperationTimeout(5)  # 5 sec

        # Connect and subscribe to AWS IoT
        self.myAWSIoTMQTTClient.connect()

        self.init_pubs()
        self.init_subs()

    def init_pubs(self):
        self.mqtt_cmd_vel_pub = rospy.Publisher("mqtt_cmd_vel_string", String, queue_size=1)

    def init_subs(self):
        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)

        self.myAWSIoTMQTTClient.subscribe("mqtt_cmd_vel_from_cloud", 1, self.mqtt_from_cloud_callback)

    def mqtt_from_cloud_callback(self, client, userdata, messagedata):
        mqtt_payload = messagedata.payload.decode("utf8")

        self.mqtt_cmd_vel_pub.publish(json.dumps(ast.literal_eval(mqtt_payload) ))
        rospy.logwarn("mqtt form cloud is {}  {} {}  {}".format(client, userdata, messagedata.topic, mqtt_payload))

    def cmd_vel_callback(self, data):
        test_mqtt_send = {}
        test_mqtt_send["x_vel"] = data.linear.x
        test_mqtt_send["y_vel"] = data.linear.y
        test_mqtt_send["z_ang"] = data.angular.z

        # Publish to Mqtt data here
        self.myAWSIoTMQTTClient.publish("cmd_vel_mqtt", json.dumps(test_mqtt_send), 1)

    def main(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('mqtt_connector')
    node = MqttManager()
    node.main()
