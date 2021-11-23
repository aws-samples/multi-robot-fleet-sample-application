# Husky Noetic app with optional IoT mqtt connector

## Requirements
* [ROS Noetic](http://wiki.ros.org/noetic) - Other ROS versions have not been tested.

## Usage

### Local Execution
This section is to run a single instace of the above application. This will NOT kick off multiple robots, but is a good way to see whats under the hood in this application. Has been tested on Ubuntu 20.04 focal, with ROS Noetic

```
git clone -b noetic-husky-mqtt-dev https://github.com/aws-samples/multi-robot-fleet-sample-application.git
cd multi-robot-fleet-sample-application/simulation_ws
rosws update
rosdep install -y -r --from-path src 
catkin_make
source devel/setup.bash
pip3 install AWSIoTPythonSDK

```

Set the appropriate environement variables required for the application
```
export HUSKY_REALSENSE_ENABLED=true
export HUSKY_LMS1XX_ENABLED=true
```

To see the application running on your local machine, run the following command.
```
source simulation_ws/devel/setup.bash
roslaunch velocity_mqtt_manager velocity_mqtt_manager.launch
# rosrun velocity_mqtt_manager mqtt_connector.py  ## For mqtt connector
```


## Security

See [CONTRIBUTING](CONTRIBUTING.md#security-issue-notifications) for more information.

## License

This library is licensed under the MIT-0 License. See the LICENSE file.

