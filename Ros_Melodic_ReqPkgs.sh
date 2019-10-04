#! /bin/bash -e
set -x

# VERSION 1.0 Last Changed 2019-01-11

# Fix Gazebo models

apt-get install -y ros-melodic-can-msgs 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-controller-manager 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-cv-bridge 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-effort-controllers 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-gazebo-msgs 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-gazebo-plugins 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-gazebo-ros 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-gazebo-ros-control 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-joint-state-controller 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-joy 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-move-base 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-nav-core 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-robot-localization 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-robot-state-publisher 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-rviz-imu-plugin 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-socketcan-bridge 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-socketcan-interface 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-tf2 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-tf2-ros 2>&1 >/dev/null ||:
apt-get install -y ros-melodic-velocity-controllers 2>&1 >/dev/null ||:
