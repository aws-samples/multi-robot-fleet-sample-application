import json
import botocore
import boto3
from botocore.exceptions import WaiterError
from botocore.waiter import WaiterModel
from botocore.waiter import create_waiter_with_client
import datetime
import os
import sys
from copy import deepcopy

DEFAULT_ROSBRIDGE_SERVER_PORT = 9090
DEFAULT_IS_PUBLIC = False
DEFAULT_MAX_DURATION = 3600
DEFAULT_STREAM_UI = True

SIM_SERVER_PORT_MAPPINGS = [
  {
    "applicationPort": DEFAULT_ROSBRIDGE_SERVER_PORT,
    "enableOnPublicIp": DEFAULT_IS_PUBLIC,
    "jobPort": DEFAULT_ROSBRIDGE_SERVER_PORT
  }
]

sim_job_params = {
      "maxJobDurationInSeconds": (DEFAULT_MAX_DURATION if "MAX_JOB_DURATION" not in os.environ else os.getenv("MAX_JOB_DURATION")),
      "iamRole": os.getenv("IAM_ROLE"),
      "failureBehavior": "Fail",
      "simulationApplications": [],
      "vpcConfig": {
        "securityGroups": [ os.getenv('SECURITY_GROUP') ] if "SECURITY_GROUP" in os.environ else [],
        "subnets": [ os.getenv('SUBNET_1'), os.getenv('SUBNET_1') ] if ("SUBNET_1" in os.environ and "SUBNET_2" in os.environ) else []
      },
      "outputLocation": {
        "s3Bucket": os.getenv("S3_BUCKET"),
        "s3Prefix": "logs" 
      },
      "loggingConfig": {
        "recordAllRosTopics": True
      }
}

app_arn = os.getenv('SIM_APPLICATION_ARN')

waiter_config = {
  'version': 2,
  'waiters': {
    'SimJobCreated': {
      'operation': 'DescribeSimulationJob',
      'delay': 5,
      'maxAttempts': 70,
      'acceptors': [
          { 'matcher': 'path', 'expected': 'Pending', 'argument': 'status', 'state': 'retry' },
          { 'matcher': 'path', 'expected': 'Running', 'argument': 'status',  'state': 'success' },
          { 'matcher': 'path', 'expected': 'Terminated', 'argument': 'status',  'state': 'failure' },
          { 'matcher': 'path', 'expected': 'Completed', 'argument': 'status',  'state': 'failure' },
          { 'matcher': 'path', 'expected': 'Failed', 'argument': 'status',  'state': 'failure' }
        ]
    }
  }
}

robomaker = boto3.client('robomaker')

def create_application_config(input_params, is_server, server_ip): 
  
  if (is_server):
      port_mappings = SIM_SERVER_PORT_MAPPINGS
  else:
      port_mappings = []

  to_set_params = {
    "application": app_arn,
    "applicationVersion": "$LATEST",
    "launchConfig": {
      "environmentVariables": input_params['environmentVariables'],
      "launchFile": input_params['launchFile'],
      "packageName": input_params['packageName'],
      "portForwardingConfig" : { 'portMappings': port_mappings },
      "streamUI": DEFAULT_STREAM_UI
    }
  }

  to_set_params['launchConfig']['environmentVariables']['ROBOT_NAME'] = input_params['name']

  if (is_server):
    to_set_params['launchConfig']['environmentVariables']['ROSBRIDGE_STATE'] = "SERVER"
    to_set_params['launchConfig']['environmentVariables']['ROSBRIDGE_IP'] = "localhost"
  elif (server_ip):
    to_set_params['launchConfig']['environmentVariables']['ROSBRIDGE_STATE'] =  "CLIENT"
    to_set_params['launchConfig']['environmentVariables']['ROSBRIDGE_IP'] = server_ip

  return to_set_params
  
def lambda_handler(event, context):
  
  # Set base parameters for each simulation job. Order from Lambda event, then environment variables.

  if 'simulationJobParams' in event:

    if 'vpcConfig' in event['simulationJobParams']:
      sim_job_params['vpcConfig'] = sim_job_params['vpcConfig']
    
    if 'iamRole' in event['simulationJobParams']:
      sim_job_params['iamRole'] = sim_job_params['iamRole']
    
    if 'outputLocation' in event['simulationJobParams']:
      sim_job_params['outputLocation'] = sim_job_params['outputLocation']
    
  if 'simulationApplicationArn' in event:
    app_arn = event['simulationApplicationArn']

  if 'serverIP' in event:
    private_ip = event['serverIP']
  else:
    # Launch Server.
    server_app_params = create_application_config(event['server'], True, 'localhost')
    server_job_params = deepcopy(sim_job_params)
    server_job_params['simulationApplications'].append(server_app_params)
    server_job_response = robomaker.create_simulation_job(
      iamRole=server_job_params["iamRole"],
      maxJobDurationInSeconds=server_job_params["maxJobDurationInSeconds"],
      simulationApplications=[server_app_params],
      vpcConfig=server_job_params["vpcConfig"],
      loggingConfig=server_job_params["loggingConfig"],
      outputLocation=server_job_params["outputLocation"]
    )

    # Wait for server to be available.
    waiter_name = 'SimJobCreated'
    waiter_model = WaiterModel(waiter_config)
    custom_waiter = create_waiter_with_client(waiter_name, waiter_model, robomaker)
    custom_waiter.wait(job=server_job_response['arn'])
    desc_result = robomaker.describe_simulation_job( job = server_job_response['arn'] )
    private_ip = desc_result['networkInterface']['privateIpAddress']

  # Launch multiple robot batches.
  batch_job_requests = []
  client_app_params = {}
  client_job_params = {}
  for robot in event['robots']:
    client_app_params[robot['name']] = create_application_config(robot, False, private_ip)
    client_job_params[robot['name']] = deepcopy(sim_job_params)
    client_job_params[robot['name']]['simulationApplications'].append(client_app_params[robot['name']])
    batch_job_requests.append(client_job_params[robot['name']])

  response = robomaker.start_simulation_job_batch(
        batchPolicy={
          'timeoutInSeconds': DEFAULT_MAX_DURATION,
          'maxConcurrency': len(event['robots'])
          }, 
        createSimulationJobRequests=batch_job_requests, 
        tags = {
          'launcher': 'multi_robot_fleet'
        })

  return {
    'statusCode': 200,
    'body': response['arn']
  } 

# To run locally on your machine based on CFN stack outputs.
if __name__ == "__main__":

  # Get Sample Event
  event_path = "%s/event.json" % sys.path[0]

  with open(event_path) as f:
    event = json.load(f)
  
  if (len(sys.argv)==2):
    cfn = boto3.client('cloudformation')
    response = cfn.describe_stacks(
        StackName=sys.argv[1]
    )
    if len(response['Stacks'])>0:
      if len(response['Stacks'][0]['Outputs'])>0:
        for output in response['Stacks'][0]['Outputs']:
          if output['OutputKey'] == 'PublicSubnet1' or output['OutputKey'] == 'PublicSubnet2':
            sim_job_params['vpcConfig']['subnets'].append(output['OutputValue'])
          elif output['OutputKey'] == 'DefaultSecurityGroupID':
            sim_job_params['vpcConfig']['securityGroups'].append(output['OutputValue'])
          elif output['OutputKey'] == 'SimulationRole':
            sim_job_params['iamRole'] = output['OutputValue']
          elif output['OutputKey'] == 'RoboMakerS3Bucket':
            sim_job_params['outputLocation']['s3Bucket'] = output['OutputValue']
          elif output['OutputKey'] == 'SimulationApplicationARN':
            app_arn = output['OutputValue']
      else:
        print("Cloudformation stack did not any outputs. Using event.json values or environment variables for AWS infrastructure configuration.")
    else:
      print("Cloudformation stack not found. Using event.json values or environment variables for AWS infrastructure configuration.")
  else:
    print("No cloudformation defined. Using event.json values or environment variables for AWS infrastructure configuration.")
    
  print("Starting handler")
  lambda_handler(event, {})
  print("Simulations launched. Check out the AWS console to connect to the fleet simulation.")
