#!/bin/bash
# The MIT License (MIT)
# Copyright (c) 2014 OROCA and ROS Korea Users Group

# Script to install ROS Indigo on Ubuntu 14.04 ARM from namniart build farm

sudo update-locale LANG=C LANGUAGE=C LC_ALL=C LC_MESSAGES=POSIX

version=`lsb_release -sc`

echo "[Checking the ubuntu version]"
case $version in
  "trusty")
  ;;
  *)
    echo "[For Ubuntu  14.04 (Trusty) ARM]"
    exit 0
esac

echo "[Update & upgrade the package]"
sudo apt-get update -qq
sudo apt-get upgrade -qq

echo "[Installing chrony and setting the ntpdate]"
sudo apt-get install -y chrony
sudo ntpdate ntp.ubuntu.com

echo "[Add the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c 'echo "deb http://packages.namniart.com/repos/ros trusty main" > /etc/apt/sources.list.d/ros-latest.list'
fi

echo "[Download the ROS keys]"
roskey=`apt-key list | grep "ROS builder"`
if [ -z "$roskey" ]; then
  wget http://packages.namniart.com/repos/namniart.key -O - | sudo apt-key add -
fi

echo "[Update & upgrade the package]"
sudo apt-get update -qq
sudo apt-get upgrade -qq

echo "[Installing ROS]"
sudo apt-get install -y build-essential ros-indigo-ros-base

echo "[rosdep init and python-rosinstall]"
sudo sh -c "rosdep init"
rosdep update
. /opt/ros/indigo/setup.sh
sudo apt-get install -y python-rosinstall

echo "[Making the catkin workspace and testing the catkin_make]"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make

echo "[Setting the ROS evironment]"
sh -c "echo \"source /opt/ros/indigo/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source ~/catkin_ws/devel/setup.bash\" >> ~/.bashrc"
sh -c "echo \"export ROS_MASTER_URI=http://localhost:11311\" >> ~/.bashrc"
sh -c "echo \"export ROS_HOSTNAME=localhost\" >> ~/.bashrc"
sh -c "echo \"export ROSLAUNCH_SSH_UNKNOWN=1\" >> ~/.bashrc"

echo "[Complete!!!]"

exec bash

exit 0
