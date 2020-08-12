#!/bin/bash

# This script will create the AWS infrastructure required to launch the demo, as well as future multi-robot apps.
BASE_DIR=`pwd`
ROS_APP_DIR=$BASE_DIR/simulation_ws
LAUNCHER_APP_DIR=$BASE_DIR/setup/fleetLauncherApp
STACK_NAME=multibotrosbridge
AWS_PROFILE=default
CURRENT_STACK=.current-aws-stack
S3_OUTPUT_KEY=multirobotdemo/bundle/output.tar

# Setup AWS resources for the application
if [ ! -f "$CURRENT_STACK" ]; then
  # Deploy base stack (NOTE: This will NOT deploy the SAM-based Lambda function. To do that, follow the instructions in the README.)
  aws cloudformation deploy --template-file $LAUNCHER_APP_DIR/base_template.yml --stack-name $STACK_NAME --capabilities CAPABILITY_NAMED_IAM --parameter-overrides SimulationApplicationS3Key=$S3_OUTPUT_KEY --profile $AWS_PROFILE
  aws cloudformation wait stack-create-complete --stack-name $STACK_NAME --profile $AWS_PROFILE && echo "stackname=$STACK_NAME" > .current-aws-stack
fi

# Upload the application bundle. 
s3Bucket=$(aws cloudformation describe-stacks --stack-name $STACK_NAME --query "Stacks[0].Outputs[?OutputKey=='RoboMakerS3Bucket'].OutputValue" --profile $AWS_PROFILE --output text)

if [ ! $s3Bucket==None ] || [ -f "$ROS_APP_DIR/bundle/output.tar" ]
then
  echo "Uploading files..."
  aws s3 cp $ROS_APP_DIR/bundle/output.tar s3://$s3Bucket/$S3_OUTPUT_KEY --profile $AWS_PROFILE
else
  echo "Bundle could not be uploaded. Please ensure \"$ROS_APP_DIR/bundle/output.tar\" and the S3 bucket \"$s3Bucket\" exist."
  exit
fi

read -t 15 -p "Press any key to launch the sample simulation, or Ctrl+c within 15 seconds to exit." some_key
python3 $LAUNCHER_APP_DIR/fleetLauncherLambda/app.py $STACK_NAME
