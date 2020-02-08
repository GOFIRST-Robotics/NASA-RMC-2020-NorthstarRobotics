# NASA-RMC-2020-NorthstarRobotics
The NASA RMC 2020 repository for Northstar Robotics, a segment of University of Minnesota Robotics student group

## About

> The goal of the competition is to collect icy regolith simulant from a simulated martian surface and deposit the collected material in a hopper. The top layer of the arena is a fine, dusty material, and the  bottom layer is gravel, which simulates icy regolith on mars. This is accomplished over two runs of 15 minutes. Massive bonus points are available for runs that are fully autonomous, but teleop control is available.
[Presentation](https://docs.google.com/presentation/d/1eXhP5dbxtgWJBE492b78TFrnQFL9yoPsJFO4HszJn60/edit#slide=id.p)

### Competition Rules

 - [NASA Current
   Rules](https://www.nasa.gov/offices/education/centers/kennedy/technology/nasarmc.html)
- [Rulebook 2020](https://www.nasa.gov/sites/default/files/atoms/files/rmc_lunabotics_2020_registration_rules_rubrics_all.pdf)
- [Our Notes
   (old)](https://docs.google.com/document/d/1pDDC_1_RaHGiCGMDKFiZ845Ba5-fpn_cems4Xnu7P24/edit)

## Documentation 
See documentation README for instructions

## Timeline

See project board for [current status](https://github.com/GOFIRST-Robotics/NASA-RMC-2020-NorthstarRobotics/projects/1)

 - [x] -   Learning
 - [x] -   Robust Teleop
 - [ ] -  Trough Alignment
 
 Winter Break
 - [ ] - Path Planning [FEB]
	 - [ ]  - Mapping
	 - [ ] Obstacle Avoidance
 - [ ]  - Partial Autonomy [MAR]
	 - [ ] Dumping
	 - [ ] Driving
 - [ ] - Full Autonomy [APRIL]
 - [ ] - Testing [APRIL]
 - [ ] - Competition [MAY]
Variable Schedule
 - [ ] - Simple Driver Assistance
	 - [ ]  - Auto mining
	 - [ ] - Lift Leveling
	 - [ ] - Camera Streaming?

[Milestones](https://docs.google.com/document/d/17NoA1MLxK1FFYn2GWv_bi-EYi50x5UTQvx3rQ464B9A/edit?usp=sharing)
## ROS Workspace


**Package:**		Description:

* **rovr_common**			robot specific system launch files (high level)
* **rovr_control**	    	robot control and telecom, wheel io (low level)
* **rovr_description**		a description of the robot, for simulation (geometry 							 parameters)
* **rovr_gazebo**			gazebo simulation environment
* **rovr_input**			testing input/movement
* **rovr_navigation**		navigation, EKF

Feature packages
* **decawave**				localization using decawaves
* **formatter_string**		string formatter
* **navx**					navx IMU data
* **socketcan_bridge**		socketcan bridge
* **socketcan_interface**	socketcan interface
* **telecom**				telecom

3rd party
* **aruco_localization**  	localization using aruco markers (visual targets)	

## Other Repos
### Companion Repos
 - [serial](https://github.com/GOFIRST-Robotics/serial)
 - [realsense-ros](https://github.com/GOFIRST-Robotics/realsense-ros)
- [aruco_localization](https://github.com/GOFIRST-Robotics/aruco_localization)
- [navXTimeSync](https://github.com/GOFIRST-Robotics/navXTimeSync)
### Previous Years

 - [NASA-RMC-2019-NorthstarRobotics](https://github.com/GOFIRST-Robotics/NASA-RMC-2019-NorthstarRobotics)
- [NASA-RMC-2018](https://github.com/GOFIRST-Robotics/NASA-RMC-2018)

## Resources
### Communication
All team communication is through Slack - please see an officer or team member for instructions on how to joing our Slack workspace.  Relevant channels are "rmc" and "rmc_programming".


## FAQs
### NASA FAQ
[FAQ](https://www.nasa.gov/sites/default/files/atoms/files/rmc_lunabotics_2020_faq_03.pdf)
### How do I get the Intel RealSense to work?
Follow the installation instructions at this [repo](https://github.com/IntelRealSense/realsense-ros).
### How do I get started with Linux?
Here are some recommendations from Jude:
[Single Page Command Line Essentials](https://drive.google.com/file/d/1d_TEG5M8cbDhSmptYmyXXeNGSyzCbaeC/view)  
[link](https://drive.google.com/file/d/1chCfI9dKEk5xn1EhsuBepCiudX6nKlng/view)  
[link](https://drive.google.com/file/d/1erAJl0C8ypFN3QjhTqR0VGCr4sjenJmp/view)
Good chapters: 2-6, 10, 12, 15, 17  
[link](https://drive.google.com/file/d/1VJa_LGtTaZmOy9H4unzVqFZFCB_CC14B/view)
### What is ROS?
[Wikipedia](https://en.wikipedia.org/wiki/Robot_Operating_System): 

> Robot Operating System (ROS or ros) is robotics middleware (i.e. collection of software frameworks for robot software development). Although ROS is not an operating system, it provides services designed for a heterogeneous computer cluster such as hardware abstraction, low-level device control, implementation of commonly used functionality, message-passing between processes, and package management. Running sets of ROS-based processes are represented in a graph architecture where processing takes place in nodes that may receive, post and multiplex sensor data, control, state, planning, actuator, and other messages.

With ROS, we can use existing packages, share our packages, and facilitate communication between the parts of our software

### Example ROS filesystem
### Example node communication

### Getting Started with ROS
-   ROS requires* Ubuntu OS (each version of ROS requires* a specific version of Ubuntu)
	-   Recommended:
	    -   Install Ubuntu 18.04  http://releases.ubuntu.com/18.04/
    
		-   Install ROS Melodic  http://wiki.ros.org/melodic
    -   Also acceptable:
	    -   If you already have installed a different version of Ubuntu, you may use its corresponding ROS version. The robot’s computer is running 18.04 with Melodic, however, so you may have compatibility issues.
	    - Running a virtual machine  https://www.vmware.com/
	
	See documentation folder README for more information
