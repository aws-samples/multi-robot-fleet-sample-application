# How to build and run docker image
## To BUILD this example app
cd multi-robot-fleet-sample-app/simulation_ws
docker build -t robot_fleet:latest ./

## To RUN this app
export DISPLAY=:0
xhost +local:root
docker run -it --rm --privileged --net=host -e DISPLAY --name robot_fleet robot_fleet:latest roslaunch velocity_mqtt_manager velocity_mqtt_manager.launch

# For all instructions below - update the AWS CLI
cd ~
curl "https://awscli.amazonaws.com/awscli-exe-linux-x86_64.zip" -o "awscliv2.zip"
unzip awscliv2.zip
sudo ./aws/install

# To run with IOT MQTT connection
## Export environment variables required for making aws API calls
export CERT_FOLDER_LOCATION=/home/ubuntu/environment/multi-robot-fleet-sample-application/simulation_ws/src/velocity_mqtt_manager/certs/
export THING_NAME=husky_robot
export IOT_CONFIG_FILE=/home/ubuntu/environment/multi-robot-fleet-sample-application/simulation_ws/src/velocity_mqtt_manager/config/iot_config.json
export IOT_POLICY_NAME=husky_robot_iot_policy

## Copy the policy file template to location
wget https://ee-assets-prod-us-east-1.s3.us-east-1.amazonaws.com/modules/049ef6d0db2440b4a01217fc22a555c4/v8/iot-policy.json  -P /home/ubuntu/environment/
export IOT_POLICY_FILE=/home/ubuntu/environment/iot-policy.json

## Make aws api call to setup IoT thing and get credial files
export ACCOUNT_ID=$(aws sts get-caller-identity --query Account --output text)
export ENDPOINT_ADDRESS=$(aws iot describe-endpoint --endpoint-type iot:Data-ATS --query endpointAddress  --output text)
export THING_ARN=$(aws iot create-thing --thing-name $THING_NAME --query thingArn --output text)
export CERT_ARN=$(aws iot create-keys-and-certificate --set-as-active \
--certificate-pem-outfile ${CERT_FOLDER_LOCATION}${THING_NAME}.cert.pem  \
--public-key-outfile ${CERT_FOLDER_LOCATION}${THING_NAME}.public.key \
--private-key-outfile ${CERT_FOLDER_LOCATION}${THING_NAME}.private.key \
--query certificateArn --output text)

## Get root certificate and attach the cert to the IoT thing generated
curl https://www.amazontrust.com/repository/AmazonRootCA1.pem > ${CERT_FOLDER_LOCATION}root-CA.crt
aws iot register-certificate --certificate-pem file://$THING_NAME.cert.pem --ca-certificate-pem file://root-CA.crt
export CERT_ID=${CERT_ARN#*cert/}   # Unfortunately to clean up, api takes cert id and not arn.
aws iot attach-thing-principal --principal $CERT_ARN --thing-name $THING_NAME

## This replaces the "ACCOUNT_ID" template in the file to the env var in $ACCOUNT_ID
sed -i -e "s/ACCOUNT_ID/$ACCOUNT_ID/g" $IOT_POLICY_FILE

## Replace variables in the iot config file with credential files
sed -i -e "s/ENDPOINT/$ENDPOINT_ADDRESS/g" $IOT_CONFIG_FILE
sed -i -e "s/ROOTCA/root-CA.crt/g" $IOT_CONFIG_FILE
sed -i -e "s/PRIVATEKEY/$THING_NAME.private.key/g" $IOT_CONFIG_FILE
sed -i -e "s/CERTPATH/$THING_NAME.cert.pem/g" $IOT_CONFIG_FILE

## Create and attach the policy to the certificate
aws iot create-policy --policy-name $IOT_POLICY_NAME --policy-document file://$IOT_POLICY_FILE
aws iot attach-policy --policy-name $IOT_POLICY_NAME --target $CERT_ARN

## To Connect to the IOT topics AFTER setting up certs and iot_config.json
## Set LAUNCH_MQTT=true and re-build then re-run
export LAUNCH_MQTT=true
cd multi-robot-fleet-sample-app/simulation_ws
docker build -t robot_fleet:latest ./
docker run -it --rm --privileged --net=host -e DISPLAY --name robot_fleet robot_fleet:latest roslaunch velocity_mqtt_manager velocity_mqtt_manager.launch

# To run in RoboMaker
## Get your account info
export ACCOUNT_ID=$(aws sts get-caller-identity --query Account | bc)

## Log into Amazon ECR
aws ecr create-repository --repository-name robot_fleet
aws ecr get-login-password --region us-west-2 | docker login --username AWS --password-stdin $ACCOUNT_ID.dkr.ecr.us-west-2.amazonaws.com

## Tag image for ECR
docker tag robot_fleet:latest $ACCOUNT_ID.dkr.ecr.us-west-2.amazonaws.com/robot_fleet:latest

## Push image to ECR
docker push $ACCOUNT_ID.dkr.ecr.us-west-2.amazonaws.com/robot_fleet:latest

## Create robomaker simulation application
aws robomaker create-simulation-application --name robot-fleet-sim \
--simulation-software-suite name=SimulationRuntime \
--robot-software-suite name=General \
--environment uri=$ACCOUNT_ID.dkr.ecr.us-west-2.amazonaws.com/robot_fleet:latest

## Copy the "arn" value returned then
## Paste into simulation_ws/robot-fleet-sim.json "application" field
## Save robot-fleet-sim.json file

## Get the VPC and subnet info to use in the sim job
export STACK_NAME=mod-049ef6d0db2440b4
export SIM_ROLE_ARN=$(aws cloudformation describe-stacks --stack-name $STACK_NAME --query "Stacks[0].Outputs[?OutputKey=='SimulationRole'].OutputValue" --output text)
export SECURITY_GROUP=$(aws cloudformation describe-stacks --stack-name $STACK_NAME --query "Stacks[0].Outputs[?OutputKey=='DefaultSecurityGroupID'].OutputValue" --output text)
export SUBNET_1=$(aws cloudformation describe-stacks --stack-name $STACK_NAME --query "Stacks[0].Outputs[?OutputKey=='PublicSubnet1'].OutputValue" --output text)
export SUBNET_2=$(aws cloudformation describe-stacks --stack-name $STACK_NAME --query "Stacks[0].Outputs[?OutputKey=='PublicSubnet2'].OutputValue" --output text)
export SUBNET_3=$(aws cloudformation describe-stacks --stack-name $STACK_NAME --query "Stacks[0].Outputs[?OutputKey=='PublicSubnet3'].OutputValue" --output text)
export VPC=$(aws cloudformation describe-stacks --stack-name $STACK_NAME --query "Stacks[0].Outputs[?OutputKey=='VPC'].OutputValue" --output text)

## Run the robomaker sim job
aws robomaker create-simulation-job  \
--compute computeType=CPU --max-job-duration-in-seconds 3600 \
--iam-role $SIM_ROLE_ARN  \
--vpc-config subnets=$SUBNET_1,$SUBNET_2,$SUBNET_3,securityGroups=$SECURITY_GROUP,assignPublicIp=false \
--simulation-application file://$HOME/environment/multi-robot-fleet-sample-application/simulation_ws/robot-fleet-sim.json
