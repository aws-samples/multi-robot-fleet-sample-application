#!/bin/bash
set -e

# setup ros environment
source /opt/ros/noetic/setup.bash
source /multi-robot-fleet-sample-application/simulation_ws/devel/setup.bash
export HUSKY_REALSENSE_ENABLED=true
export HUSKY_LMS1XX_ENABLED=true
exec "$@"
