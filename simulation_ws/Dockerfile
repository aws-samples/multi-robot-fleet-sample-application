FROM ros:noetic

# install tools
RUN apt update && apt install -y git python3-pip

# clone the main application
RUN git clone -b noetic-husky-mqtt-dev https://github.com/aws-samples/multi-robot-fleet-sample-application.git

# install app dependencies
WORKDIR /multi-robot-fleet-sample-application/simulation_ws
RUN rosws update
RUN rosdep install --from-paths src -i -r -y

# build app
WORKDIR /multi-robot-fleet-sample-application/simulation_ws
RUN . /opt/ros/noetic/setup.sh && catkin_make

# install AWS IOT SDK
RUN python3 -m pip install AWSIoTPythonSDK

# install xterm for interactive use
RUN apt install -y xterm

# copy certs and iot config file
WORKDIR /multi-robot-fleet-sample-application/simulation_ws
COPY src/velocity_mqtt_manager/certs src/velocity_mqtt_manager/certs 
COPY src/velocity_mqtt_manager/config/* src/velocity_mqtt_manager/config

# setup entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]