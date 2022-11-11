#!/bin/bash
################################################################################
# Script for starting simulation tool & preparing ros2 bridge
# Authors      : Ali Kamberi
# Company      : Zurich UAS Racing
# Copyright by : Zurich UAS Racing
#-------------------------------------------------------------------------------
# This script will start the simulation tool in a different terminal and
# prepare the ros2 bridge by building it and sourcing the needed files
#-------------------------------------------------------------------------------
################################################################################

echo -e "starting simulation tool in separate terminal\n"

# start new shell with simulation tool
gnome-terminal --tab -- bash -c "cd ~/fsds-v2.1.0; /bin/bash ./FSDS.sh"

cd ~/Formula-Student-Driverless-Simulator/ros2

echo -e "cleanup ros2 bridge"
# cleanup ros2 bridge
if [-d "build"]; then
  rm -rf build/
fi
if [-d "install"]; then
  rm -rf install/
fi
if [-d "log"]; then
  rm -rf log/
fi

echo -e "source galactic, build packages & source overlay"
# source ros2 galactic &  build ros2
source /opt/ros/galactic/setup.bash
colcon build
. install/local_setup.bash

# launch ros2 bridge
read -p "please select the scenario for the simluation tool and start the simulation. Confirm this step by entering (y)es or (n)o:" yn

case $yn in 
	yes ) echo ok, we will proceed;;
	no ) echo exiting...;
		exit;;
	* ) echo invalid response;
		exit 1;;
esac

ros2 launch fsds_ros2_bridge  fsds_ros2_bridge.launch.py 
