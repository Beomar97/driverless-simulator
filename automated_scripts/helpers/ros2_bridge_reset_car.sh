#!/bin/bash
################################################################################
# Script for resetting car in simulation tool via ros2 bridge
# Authors      : Ali Kamberi & Guilherme Vicentini Briner
# Company      : Zurich UAS Racing
# Copyright by : Zurich UAS Racing
#-------------------------------------------------------------------------------
# This script will reset the cars position to the starting line via the ros2
# bridge
# $1:    dir to ros2 bridge location
#-------------------------------------------------------------------------------
################################################################################

ROSFOLDER=$1
PLOTSACTIVE=$2

if [ ! -d "$ROSFOLDER" ];
then
  ROSFOLDER="/home/sim/Formula-Student-Driverless-Simulator/ros2"
fi
echo "folder containing ros2 bridge to build: $ROSFOLDER"

cd $ROSFOLDER

source /opt/ros/galactic/setup.bash
. $ROSFOLDER/install/local_setup.bash

ros2 service call /reset fs_msgs/srv/Reset


echo "CMD: $PLOTSACTIVE"
if [ "$PLOTSACTIVE" = "plots=True" ];
then
  ros2 service call /plot_reset fs_msgs/srv/Reset
fi
