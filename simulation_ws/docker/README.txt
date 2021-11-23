# How to build and run docker image
# To BUILD this example app
docker build -t robot_fleet:latest ./

# To RUN this app
export DISPLAY=:0
xhost +local:root
docker run -it --privileged --net=host -e DISPLAY robot_fleet:latest roslaunch velocity_mqtt_manager velocity_mqtt_manager.launch

# get your account info
export ACCOUNT_ID=$(aws sts get-caller-identity --query Account | bc)

# log into Amazon ECR
aws ecr create-repository --repository-name rob304-reinvent
aws ecr get-login-password --region us-west-2 | docker login --username AWS --password-stdin $ACCOUNT_ID.dkr.ecr.us-west-2.amazonaws.com

# tag image 
docker tag robot_fleet:latest $ACCOUNT_ID.dkr.ecr.us-west-2.amazonaws.com/rob304-reinvent:latest

# push image
docker push $ACCOUNT_ID.dkr.ecr.us-west-2.amazonaws.com/rob304-reinvent:latest
