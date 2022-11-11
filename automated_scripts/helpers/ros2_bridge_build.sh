#!/bin/bash
################################################################################
# Script for building ros 2 bridge
# Authors      : Ali Kamberi & Guilherme Vicentini Briner
# Company      : Zurich UAS Racing
# Copyright by : Zurich UAS Racing
#-------------------------------------------------------------------------------
# This script will build the ros2 bridge Args provided:
# $1:    dir to ros2 bridge location
#-------------------------------------------------------------------------------
################################################################################

ROSFOLDER=$1

if [ ! -d "$ROSFOLDER" ];
then
  ROSFOLDER="/home/sim/Formula-Student-Driverless-Simulator/ros2"
fi
echo "folder containing ros2 bridge to build: $ROSFOLDER"

cd $ROSFOLDER

source /opt/ros/galactic/setup.bash

colcon build