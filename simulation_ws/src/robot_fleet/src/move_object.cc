/*
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
*/
/*
This plugin subscribes to a topic call "topic_to_move" and sets the pose of the model that it is
attached to based on the data from the topic. It is intended to be attached to a static model.

When spawning a robot with gazebo/spawn_sdf_model rosservice call, the topic to subscribe gets
a namespace attached to it. So this plugin then looks for <model_name>/topic_to_move
*/


#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>


using namespace gazebo;

    class MoveObject : public ModelPlugin
    {
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            this->model = _parent;
            this->model_name = this->model->GetName();
#if GAZEBO_MAJOR_VERSION < 8
            this->current_pose = this->model->GetWorldPose();
#else
            this->current_pose = this->model->WorldPose();
#endif

            // Listen to the update event. This event is broadcast every simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MoveObject::OnUpdate, this));
            this->old_secs =ros::Time::now().toSec();

            // Initialize ros, if it has not already been initialized.
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                    << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }

            // Create our ROS node. This acts in a similar manner to the Gazebo node
            this->rosNode.reset(new ros::NodeHandle(this->model->GetName()));

            this->rosNode->getParam(std::string("/use_custom_move_object_gazebo_plugin"), this->use_custom_move_object_gazebo_plugin);
            ROS_INFO_STREAM("use_move is set to: " << this->use_custom_move_object_gazebo_plugin);


            if (this->use_custom_move_object_gazebo_plugin)
            {
                // Store the pointer to the model

                // Subscribe to topic here. Note that if you pass model name, it adds namespace to the topic
                ros::SubscribeOptions sub_options = ros::SubscribeOptions::create<geometry_msgs::Pose>("topic_to_move", 1,
                                              boost::bind(&MoveObject::OnPoseCallback, this, _1),
                                              ros::VoidPtr(), &this->rosQueue);
                this->rosSubscription = this->rosNode->subscribe(sub_options);

                // Spin up the queue helper thread.
                this->rosQueueThread = std::thread(std::bind(&MoveObject::QueueThread, this));
            }
        }

        // Destructor
        public: virtual ~MoveObject()
        {
            if (this->use_custom_move_object_gazebo_plugin)
            {
                this->rosQueue.disable();
                this->rosQueueThread.detach();
            }
            this->rosNode->shutdown();
        }

        // Called by the world update start event
        public: void OnUpdate()
        {
            if (this->use_custom_move_object_gazebo_plugin)
            {
                this->model->SetWorldPose(this->current_pose);
            }
        }

        public: void OnPoseCallback(const geometry_msgs::PoseConstPtr &_msg)
        {
            // Update location variables based on data from callback.
            this->current_pose = ignition::math::Pose3d( _msg->position.x, _msg->position.y, _msg->position.z,
                                    _msg->orientation.w, _msg->orientation.x, _msg->orientation.y,_msg->orientation.z);
        }

        /// \brief ROS helper function that processes messages
        private: void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

            
        // Pointer to the model
        private: physics::ModelPtr model;
#if GAZEBO_MAJOR_VERSION < 8
        private: math::Pose current_pose;
#else
        private: ignition::math::Pose3d current_pose;
#endif
        private: std::string rostopic_to_follow;
        private: std::string model_name;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
        private: bool use_custom_move_object_gazebo_plugin;

        // Time Memory
        double old_secs;

        // \brief A node use for ROS transport
        private: std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A ROS subscriber
        private: ros::Subscriber rosSubscription;
        /// \brief A ROS callbackqueue that helps process messages
        private: ros::CallbackQueue rosQueue;
        /// \brief A thread the keeps running the rosQueue
        private: std::thread rosQueueThread;
    };

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MoveObject);
