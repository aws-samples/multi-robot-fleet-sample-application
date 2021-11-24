# How to build and run docker image
# To BUILD this example app
docker build -t robot_fleet:latest ./

# To RUN this app
export DISPLAY=:0
xhost +local:root
docker run -it --rm --privileged --net=host -e DISPLAY --name robot_fleet robot_fleet:latest roslaunch velocity_mqtt_manager velocity_mqtt_manager.launch

# To Connect to the IOT topics AFTER setting up certs and iot_config.json
# in another terminal
docker exec -it robot_fleet /bin/bash
# within container run the following commands to start the mqtt connection app
source devel/setup.bash
rosrun velocity_mqtt_manager mqtt_connector.py 

# To run in RoboMaker
# get your account info
export ACCOUNT_ID=$(aws sts get-caller-identity --query Account | bc)

# log into Amazon ECR
aws ecr create-repository --repository-name robot_fleet
aws ecr get-login-password --region us-west-2 | docker login --username AWS --password-stdin $ACCOUNT_ID.dkr.ecr.us-west-2.amazonaws.com

# tag image 
docker tag robot_fleet:latest $ACCOUNT_ID.dkr.ecr.us-west-2.amazonaws.com/robot_fleet:latest

# push image
docker push $ACCOUNT_ID.dkr.ecr.us-west-2.amazonaws.com/robot_fleet:latest

# in a new terminal - update the AWS CLI
cd ~
curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip"
unzip awscliv2.zip
sudo ./aws/install

# create robomaker simulation application
aws robomaker create-simulation-application --name robot-fleet-sim \
--simulation-software-suite name=SimulationRuntime \
--robot-software-suite name=General \
--environment uri=$ACCOUNT_ID.dkr.ecr.us-west-2.amazonaws.com/robot_fleet:latest

# copy the "arn" value returned then
# paste into simulation_ws/robot-fleet-sim.json "application" field
# save robot-fleet-sim.json file

# run the robomaker sim job
aws robomaker create-simulation-job  \
--compute computeType=CPU --max-job-duration-in-seconds 3600 \
--iam-role arn:aws:iam::$ACCOUNT_ID:role/robomaker_sim  \
--simulation-application file://$HOME/environment/multi-robot-fleet-sample-application/simulation_ws/robot-fleet-sim.json

