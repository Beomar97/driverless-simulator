#!/bin/bash
################################################################################
# Script starting ros 2 bridge
# Authors      : Ali Kamberi & Guilherme Vicentini Briner
# Company      : Zurich UAS Racing
# Copyright by : Zurich UAS Racing
#-------------------------------------------------------------------------------
# This script will start the ros2 bridge in a separate terminal. Args provided:
# $1:    dir to ros2 bridge location
# $2:    optional parameters for ros2 bridge
#-------------------------------------------------------------------------------
################################################################################

ROSFOLDER=$1
OPTIONAL_PARAMS=$2
ROS2_CMD="ros2 launch fsds_ros2_bridge  fsds_ros2_bridge.launch.py"

if [ ! -d "$ROSFOLDER" ];
then
  ROSFOLDER="/home/sim/Formula-Student-Driverless-Simulator/ros2"
fi

echo "folder containing ros2 bridge to build: $ROSFOLDER"
echo "OPTIONAL PARAMS SET:                    $OPTIONAL_PARAMS"

# if optional params are set, update the command for starting ros2 bridge
if [ ! -z ${OPTIONAL_PARAMS+x} ];
then
  ROS2_CMD="$ROS2_CMD $OPTIONAL_PARAMS"
fi

cd $ROSFOLDER

source /opt/ros/galactic/setup.bash
. $ROSFOLDER/install/local_setup.bash

# launch ros2 bridge
$ROS2_CMD