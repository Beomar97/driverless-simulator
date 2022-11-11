#!/bin/bash
################################################################################
# Script for installing ros2, simulation tool v2.1 and nvidia tools
# Authors      : Ali Kamberi & Guilherme Vicentini Briner
# Company      : Zurich UAS Racing
# Copyright by : Zurich UAS Racing
#-------------------------------------------------------------------------------
# This script will install ros2 foxy & galactic with all the required extensions and also
# install the simulation tool from Formula Student Driverless v2.1.0
#-------------------------------------------------------------------------------
################################################################################

echo -e "please enter username, for which the simulation tool and ros2 foxy should be installed\n"
read -p "username:" USERNAME

# check if user exists
if id -u "$USERNAME" >/dev/null 2>&1; then
    echo "found user"
else
    echo "User does not exist -- Invalid Username"
    exit 0
fi
HOMEDIR="/home/$USERNAME"

apt update
apt upgrade -y

# install ros tools and development tools
apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget \
  libyaml-cpp-dev
# install some pip packages needed for testing
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest
# install Fast-RTPS dependencies
apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
apt install --no-install-recommends -y \
  libcunit1-dev

# install desktop
echo -e "installing ubuntu desktop"
apt install tasksel -y
tasksel install ubuntu-desktop

# install git & locale, set locale
apt install git locales -y
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ros2 add apt repository
echo -e "instaling ros2 foxy and ros2 galactic"
apt install curl gnupg2 lsb-release -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
# ros2 add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo -e "instaling ros2 foxy"
apt update
apt install ros-foxy-desktop -y
echo -e "instaling ros2 galctic"
apt install ros-galactic-desktop -y
# download ros2 foxy Patch Release 7
#cd $HOMEDIR
#ROS2_FOXY=ros2-foxy-20220208-linux-focal-amd64.tar.bz2
#wget https://github.com/ros2/ros2/releases/download/release-foxy-20220208/$ROS2_FOXY
# extract ros2 bin files & delete .tar
# mkdir -p $HOMEDIR/ros2_foxy
# cd $HOMEDIR/ros2_foxy
# tar xf $HOMEDIR/$ROS2_FOXY
#rm -rf $HOMEDIR/$ROS2_FOXY

# install colcon
echo -e "installing colcon for building ros2"
apt install python3-colcon-common-extensions -y

# install and init rosdep
echo -e "installing rosdep"
apt install -y python3-rosdep
sudo rosdep init
rosdep update
# install missing dependencies
mkdir -p ~/ros2_foxy/src
cd ~/ros2_foxy
wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
vcs import src < ros2.repos
rosdep install --from-paths ~/ros2_foxy/ros2-linux/share --ignore-src -y --skip-keys "cyclonedds fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"

# simulation tool get binary files
SIMULATION_ZIP=fsds-v2.1.0-linux.zip
SIMULATION_FOLDER=fsds-v2.1.0
cd $HOMEDIR/
wget https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.1.0/$SIMULATION_ZIP
unzip -o $SIMULATION_ZIP -d $SIMULATION_FOLDER 
rm -rf $SIMULATION_ZIP
# build formula student diverless simulator
#cd $SIMULATION_FOLDER
#chmod +x FSDS.sh
#/bin/bash FSDS.sh

# simulation tool source files
# TODO: change path to github repo
# SIMULATION_SOURCE_TAR=v2.1.0.tar.gz
SIMULATION_AirSim_FOLDER=Formula-Student-Driverless-Simulator/AirSim
# wget https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/archive/refs/tags/$SIMULATION_SOURCE_TAR
git clone https://github.zhaw.ch/FSZHAW/Simulation_Tool.git Formula-Student-Driverless-Simulator
# tar xf $SIMULATION_SOURCE_TAR
# rm -rf $SIMULATION_SOURCE_TAR
# build AirSim
cd $SIMULATION_AirSim_FOLDER
/bin/bash setup.sh
/bin/bash build.sh
# change rights to folders
cd $HOMEDIR
chown -R $USERNAME:$USERNAME $SIMULATION_AirSim_FOLDER $SIMULATION_FOLDER

# install vulkan tools
echo -e "installing vulkan-tools"
apt install vulkan-tools -y
