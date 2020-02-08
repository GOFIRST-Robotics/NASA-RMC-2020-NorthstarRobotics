# Documentation
new member slides here:
https://docs.google.com/presentation/d/1eXhP5dbxtgWJBE492b78TFrnQFL9yoPsJFO4HszJn60/edit?usp=sharing

 - see doc folder for specific documentation
 - see old_code for previous year's code possibly with notes
 - see examples for an example node with the best practices style

# Starter Instructions

## Getting Everything Running From a Fresh Ubuntu Install
Configure ROS environment: see  [here:](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
add to bashrc for permanence, else run in terminal each time:
```
export ROS_HOSTNAME=ubuntu.local
export ROS_MASTER_URI=http://ubuntu.local:11311

source /opt/ros/<distro>/setup.bash
source <your path here>devel/setup.bash
```

If installing ROS on Linux Mint: 
Start following http://wiki.ros.org/kinetic/Installation/Ubuntu 
Once "apt-get update" doesn't work go here:
http://insane-on-linux.blogspot.com/2014/10/installing-ros-indigo-on-mint-17.html
Replace "trusty" with "xenial" if using kinetic (second comment)

### Updating Git Submodules
This command will pull down the submodules and install them as they are on the origin. after init a <submod name> can be added to only update a specific submodule.
'''
git submodule update --init
'''

### Install Required ROS Packages
run the script found here:
https://github.com/GOFIRST-Robotics/NASA-RMC-2020-NorthstarRobotics/blob/master/Ros_Melodic_ReqPkgs.sh

### Installing RealSense
Get RealSense installed and running:
https://github.com/intel-ros/realsense
If you run into an issue at catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release,
run "pip install empy" and NOT "pip install em"

### Installing our repo: 
(replace kinetic with melodic depending on your version of ROS)

```
git clone git@github.com:GOFIRST-Robotics/NASA-RMC-2020-NorthstarRobotics.git
cd NASA-RMC-2020-NorthstarRobotics/
git submodule update --init
```

(if you have errors at this point, you might not have permissions set up correctly - contact me)

```
 source /opt/ros/kinetic/setup.bash
sudo apt-get install ros-kinetic-socketcan-bridge
./Ros_Melodic_ReqPkgs.sh
```

follow directions here https://github.com/GOFIRST-Robotics/aruco_localization/tree/f2d2b1bf10f03d95e89e13dae89f3918f2d7ff88
to install aruco (includes download from other site)

or see https://github.com/juliaschatz/rmc-docker/blob/master/Dockerfile for an outdated list of packages and ArUco setup commands

### Sourcing 
If your ~/.bashrc isn't set up, do this each time you open a terminal
```
source /opt/ros/kinetic/setup.bash
source devel/setup.bash

```
## Git help

### Setting up SSH keys
Might fix permission issues if that's your problem
See ssh_setup file for instructions

### Pushing with Git
'''
git status //What files in what status
git add . //What files to add, . means all in directory
git status
git commit -m "[package name] [message]"
git pull //Adds the other code added since last pull
git push
'''

