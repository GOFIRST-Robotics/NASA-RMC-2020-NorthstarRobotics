#! /bin/bash -e
set -x

# VERSION 1.0 Last Changed 2019-01-11

# Fix Gazebo models

sudo apt-get install -y ros-kinetic-can-msgs 2>&1 >/dev/null ||:
sudo apt-get install -y ros-kinetic-controller-manager 2>&1 >/dev/null ||:
sudo apt-get install -y ros-kinetic-cv-bridge 2>&1 >/dev/null ||:
sudo apt-get install -y ros-kinetic-effort-controllers 2>&1 >/dev/null ||:
sudo apt-get install -y ros-kinetic-gazebo-msgs 2>&1 >/dev/null ||:
sudo apt-get install -y ros-kinetic-gazebo-plugins 2>&1 >/dev/null ||:
sudo apt-get install -y ros-kinetic-gazebo-ros 2>&1 >/dev/null ||:
sudo apt-get install -y ros-kinetic-gazebo-ros-control 2>&1 >/dev/null ||:
sudo apt-get install -y ros-kinetic-joint-state-controller 2>&1 >/dev/null ||:
sudo apt-get install -y ros-kinetic-joy 2>&1 >/dev/null ||:
sudo apt-get install -y ros-kinetic-move-base 2>&1 >/dev/null ||:
sudo apt-get install -y ros-kinetic-nav-core 2>&1 >/dev/null ||:
sudo apt-get install -y ros-kinetic-robot-localization 2>&1 >/dev/null ||:
sudo apt-get install -y ros-kinetic-rviz-imu-plugin 2>&1 >/dev/null ||:
sudo apt-get install -y ros-kinetic-socketcan-bridge 2>&1 >/dev/null ||:
sudo apt-get install -y ros-kinetic-socketcan-interface 2>&1 >/dev/null ||:
sudo apt-get install -y ros-kinetic-tf2 2>&1 >/dev/null ||:
sudo apt-get install -y ros-kinetic-tf2-ros 2>&1 >/dev/null ||:
sudo apt-get install -y ros-kinetic-velocity-controllers 2>&1 >/dev/null ||:
