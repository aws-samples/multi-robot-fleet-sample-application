#!/bin/bash

echo "###############################################################################"
echo "Workshop environment setup starting.."
echo "###############################################################################"

cd $(dirname $0)

sudo pip3 install -U awscli
sudo pip3 install boto3

STACK_NAME=multibotrosbridge`echo $C9_USER|tr -d [\.\\-=_]` 

#Setup AWS resources for the application
aws cloudformation deploy --template-file ./bootstrap.cfn.yaml --stack-name $STACK_NAME --capabilities CAPABILITY_NAMED_IAM

#Run the setup script to build, bundle and create the application launcher script
python3 ./aws_setup.py $STACK_NAME
