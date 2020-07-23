#
# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

import boto3
import json
import sys
from time import gmtime, strftime, sleep
import random
import subprocess
import os
import yaml
import re
from botocore.exceptions import ClientError

SETTING_FILE="../ws_settings.yml"
SETTINGS = ["aws_region", 
            "ros_version",
            "gazebo_version",
            "bucket_name", 
            "simulation_app_name", 
            "project_dir",
            "vpc", 
            "security_groups", 
            "subnets", 
            "iam_role"]
            
POSTPROCESS = [ "simulation_app_arn", 
                "app_launcher_settings"]
            

APPNAME_BASE="multi_robt_sigbridge_"

CFStackName = "" # Specify stack name through command parameter

class Setup:
    def __init__(self):
        self.settings = {}

    def setup_project_dir(self):
        try:
            result = os.getcwd().split("/")[-1]
        except Exception as e:
            errlog("Exception : %s" % str(e))
            return None

        return result

    def setup_aws_region(self):
        try:
            session = boto3.session.Session()
            result = session.region_name
        except Exception as e:
            errlog("Exception : %s" % str(e))
            return None

        return result

    def setup_ros_version(self):
        try:
            ros_distro = os.environ['ROS_DISTRO']
            if ros_distro  == "kinetic":
                result = "Kinetic"
            else:
                result = "Melodic"
        except Exception as e:
            errlog("Exception : %s" % str(e))
            return None

        return result

    def setup_gazebo_version(self):
        try:
            ros_distro = os.environ['ROS_DISTRO']
            if ros_distro  == "kinetic":
                result = 7
            else:
                result = 9
        except Exception as e:
            errlog("Exception : %s" % str(e))
            return None

        return result        

    def setup_simulation_app_name(self):
        try:
            id = boto3.client('sts').get_caller_identity()
            arn = id["Arn"]
            i = arn.rfind('/')
            if i > 0:
                name = re.sub(r'[^a-zA-Z0-9_\-]', "-", arn[i+1:])
                result = "{}simulation_{}".format(APPNAME_BASE, name)
                if len(result) > 255:
                    result = result[:255]
            else:
                result = "{}sumulation".format(APPNAME_BASE)
        except Exception as e:
            errlog("Exception : %s" % str(e))
            return None

        return result

    def setup_robot_app_name(self):
        try:
            id = boto3.client('sts').get_caller_identity()
            arn = id["Arn"]
            i = arn.rfind('/')
            if i > 0:
                name = re.sub(r'[^a-zA-Z0-9_\-]', "-", arn[i+1:])
                result = "{}robot_{}".format(APPNAME_BASE, name)
                if len(result) > 255:
                    result = result[:255]
            else:
                result = "{}robot".format(APPNAME_BASE)
        except Exception as e:
            errlog("Exception : %s" % str(e))
            return None

        return result

    def setup_vpc(self):
        return self.getFromCloudFormation("VPC")

    def setup_security_groups(self):
        return [self.getFromCloudFormation("DefaultSecurityGroupID")]
        
    def setup_subnets(self):
        return [self.getFromCloudFormation("PublicSubnet1") ,self.getFromCloudFormation("PublicSubnet2")]
        
    def setup_bucket_name(self):
        return self.getFromCloudFormation("RoboMakerS3Bucket")

    def setup_iam_role(self):
        return self.getFromCloudFormation("SimulationRole")

    def setup_robomaker_settings(self):
        log("setup roboMakerSettings.json..")

        file_name_template = "./roboMakerSettings.temp" 
        file_name_output = "../roboMakerSettings.json" 
        self.update_setting_file(file_name_template  ,file_name_output )
        return True

    def update_setting_file(self, in_file, out_file):
        with open(in_file) as f:
            lines = f.read()

        for item in self.settings:
            value = str(self.settings[item])
            value = value.replace("'", "\"")
            lines = lines.replace("<{}>".format(item), value)

        with open(out_file, mode="w") as f:
            f.write(lines)        
        

    def post_simulation_app_arn(self):
        
        log("Post process : copy bundle file to S3..")
        s3_client = boto3.client('s3')
        object_name = "multirobotdemo/simulation_ws/bundle/output.tar"
        try:
            response = s3_client.upload_file("../workspace/bundle/output.tar", self.settings["bucket_name"], object_name)
        except Exception as e:
            errlog("Post process failed!")
            errlog("Reason: Couldn't upload bundle. Haven't you build ros application yet? Please make sure to execute ros_setup.bash in advance. This script expected to be called after ros_setup.bash.")
            errlog(e)
            return None

        log("Post process : register application..")
        robomaker_client = boto3.client('robomaker')
        isAlreadyExist = False
        try:
            response = robomaker_client.create_simulation_application(
                    name=self.settings["simulation_app_name"],
                    sources= [{
                        's3Bucket' : self.settings["bucket_name"],
                        's3Key': object_name,
                        'architecture':'X86_64'
                    }],
                    simulationSoftwareSuite={
                        'name':'Gazebo',
                        'version': str(self.settings["gazebo_version"])
                    },
                    robotSoftwareSuite={
                        'name':'ROS',
                        'version': str(self.settings["ros_version"])
                    },
                    renderingEngine={
                        'name':'OGRE',
                        'version':'1.x'
                    }
                )
        except ClientError as e:
            if e.response['Error']['Code'] == 'ResourceAlreadyExistsException':
                isAlreadyExist = True
            else:
                errlog("Post process failed!")
                errlog("Reason: Failed to register simulation application")
                errlog(e)
                return None                
        
        if isAlreadyExist:
            try:
                response = robomaker_client.list_simulation_applications(
                    filters=[{'name': 'name', 'values': [self.settings["simulation_app_name"]]}])
                    
                arn = None
                for app in response["simulationApplicationSummaries"]:
                    if app['name'] == self.settings["simulation_app_name"]:
                        arn = app['arn']
                        break
                
                response = robomaker_client.update_simulation_application(
                        application=arn,
                        sources= [{
                            's3Bucket' : self.settings["bucket_name"],
                            's3Key': object_name,
                            'architecture':'X86_64'
                        }],
                        simulationSoftwareSuite={
                            'name':'Gazebo',
                            'version': str(self.settings["gazebo_version"])
                        },
                        robotSoftwareSuite={
                            'name':'ROS',
                            'version': str(self.settings["ros_version"])
                        },
                        renderingEngine={
                            'name':'OGRE',
                            'version':'1.x'
                        }
                    )  
            except Exception as e:
                errlog("Post process failed!")
                errlog("Reason: Failed to update simulation application")
                errlog(e)
                return None   

        return response['arn']
    
    def post_app_launcher_settings(self):
        log("Post process : setup app_launcher.py..")
        self.update_setting_file("./app_launcher.py.temp"  ,"../app_launcher.py" )
        return True

    def getFromCloudFormation(self, outputname):
        result = None
        try:
            cf = boto3.client('cloudformation')
            stacks = cf.describe_stacks(StackName=CFStackName)
            
            for stack in stacks["Stacks"]:
                outputs = stack["Outputs"]
                for output in outputs:
                    if output["OutputKey"] == outputname:
                        result = output["OutputValue"]

        except Exception as e:
            errlog("Exception : %s" % str(e))
            return None

        if result == None:
            errlog(outputname + " couldn't be found in the CloudFormation stack")
            return None

        return result

    def saveSettings(self, _settings, items, file_name):
        if not file_name:
            return

        for aSetting in items:
            if not aSetting in _settings:
                _settings[aSetting] = None
        
        f = open(file_name, "w+")
        f.write(yaml.dump(_settings, default_flow_style=False))
    
    def entry(self, items, prefix, file_name = None):
        if file_name and os.path.exists(file_name):
            try:
                f = open(file_name, "r+")
                _settings = yaml.load(f, Loader=yaml.FullLoader)
                if _settings:
                    self.settings.update(_settings)
                else:
                    _settings = {}
            except Exception as e:
                errlog("\nSetup failed! \n => Reason: Setting file %s exists but failed to load\n" % file_name)
                errlog(" => Error Message: %s\n\n" % str(e))
                sys.exit(1)
        else:
            _settings = {}

        for aSetting in items:
            print("Check %s" % aSetting)
            try:            
                if (not aSetting in _settings) or (not _settings[aSetting]) :
                    func_name = "%s_%s" % (prefix, aSetting)
                    result = getattr(self, func_name)()
                    if not result:
                        errlog("Failed to setup %s\nFinishing...\n" % aSetting)
                        self.saveSettings(_settings, items, file_name)
                        sys.exit(1)
                    _settings[aSetting] = result
                    self.settings.update({aSetting : result})
            except Exception as e:
                errlog("Exception : %s" % str(e))
                errlog("Failed to setup %s\nFinishing...\n" % aSetting)
                self.saveSettings(_settings, items, file_name)
                sys.exit(1)
                
            print("   => Ok")
            print("   Using %s for %s" % (str(_settings[aSetting]),aSetting))
        self.saveSettings(_settings, items, file_name)

def log(message):
    print("  \033[34m{}\033[0m".format(message))

def errlog(message):
    print("\033[91m{}\033[0m".format(message))

if __name__ == '__main__':
    CFStackName = sys.argv[1]
    setup = Setup()
    print("Start setup process..")
    setup.entry(SETTINGS, "setup")
    print("Setup finished successfully!")    
    print("Execute post process..")
    setup.entry(POSTPROCESS, "post")
    print("Post process finished successfully!")   
    print("")
    print("Post process finished successfully!")    
    print("\nDone!\nYou can try 4 robot multi robot simulation by:")
    print("\npython3 ./app_launcher.py\n")

