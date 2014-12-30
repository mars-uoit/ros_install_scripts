#!/bin/bash

# Setup ROS Repositories
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu wheezy main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

roskey=`apt-key list | grep "ROS builder"`
if [ -z "$roskey" ]; then
  wget --quiet https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
fi

sudo apt-get -y update
sudo apt-get -y upgrade

sudo apt-get -y install ntpdate
sudo service ntp stop
sudo ntpdate ntp.ubuntu.com
sudo service ntp start

# Install bootstrap dependencies
sudo apt-get -y install python-setuptools
sudo easy_install pip
sudo pip install -U rosdep rosinstall_generator wstool rosinstall

# Initializing rosdep
sudo rosdep init
rosdep update

# Create a catkin Workspace
mkdir -p ~/ros_catkin_ws
cd ~/ros_catkin_ws

# Install ROS Robot
rosinstall_generator robot --rosdistro indigo --deps --wet-only --exclude roslisp collada_parser collada_urdf --tar > indigo-robot-wet.rosinstall
wstool init -j8 src indigo-robot-wet.rosinstall

# Resolve Dependencies
mkdir -p ~/ros_catkin_ws/external_src
sudo apt-get -y install checkinstall cmake

cd ~/ros_catkin_ws/external_src
sudo apt-get install -y libboost-system-dev libboost-thread-dev
git clone https://github.com/ros/console_bridge.git
cd console_bridge
cmake .
sudo checkinstall make install

cd ~/ros_catkin_ws/external_src
wget http://archive.raspbian.org/raspbian/pool/main/l/lz4/liblz4-1_0.0~r122-2_armhf.deb
wget http://archive.raspbian.org/raspbian/pool/main/l/lz4/liblz4-dev_0.0~r122-2_armhf.deb
sudo dpkg -i liblz4-1_0.0~r122-2_armhf.deb liblz4-dev_0.0~r122-2_armhf.deb

# Resolving Dependencies with rosdep
cd ~/ros_catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:wheezy

# Building the catkin Workspace
echo "[THIS TAKES A WHILE (HOURS!!!)]"
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo


mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make

sh -c "echo \"source /opt/ros/indigo/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source ~/catkin_ws/devel/setup.bash\" >> ~/.bashrc"
sh -c "echo \"export ROS_MASTER_URI=http://localhost:11311\" >> ~/.bashrc"
sh -c "echo \"export ROS_HOSTNAME=localhost\" >> ~/.bashrc"

echo "[Complete!!!]"

exec bash

exit 0

