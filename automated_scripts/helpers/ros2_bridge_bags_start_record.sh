#!/bin/bash
################################################################################
# Script for recording ros bags to topics
# Authors      : Ali Kamberi & Guilherme Vicentini Briner
# Company      : Zurich UAS Racing
# Copyright by : Zurich UAS Racing
#-------------------------------------------------------------------------------
# This script will record to all the ros2 topics or the topics provided via 
# Args.
# Args provided:
# $1:    location where to store the rosbags
# $2:    optional parameters with ros2 topics (if not set, all topics are
#        recorded)
#-------------------------------------------------------------------------------
################################################################################

ROSBAGSFOLDER=$1

if [ ! -d "$ROSBAGSFOLDER" ];
then
  ROSBAGSFOLDER="/home/sim/Formula-Student-Driverless-Simulator/saved_simulation_runs"
fi

ROSTOPICS=$2
ROS_BAGS_CMD="ros2 bag record "
# if topics are specified, add them to command
if [ ! -z "$ROSTOPICS" ]
then
  ROS_BAGS_CMD="$ROS_BAGS_CMD $ROSTOPICS"
fi
# for recording all topics
if [ -z "$ROSTOPICS" ]
then
  # record all topics
  ROS_BAGS_CMD="$ROS_BAGS_CMD -a"
fi

cd $ROSBAGSFOLDER
echo "self defined topics:   $ROSTOPICS"
echo "ros bags cmd:          $ROS_BAGS_CMD"
sleep 3

source /opt/ros/galactic/setup.bash
$ROS_BAGS_CMD