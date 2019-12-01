#!/bin/bash

source devel/setup.bash
src/rovr_common/scripts/can_jetson_setup
src/rovr_common/scripts/wifi_power_save_off
roslaunch src/rovr_control/launch/main_rovr_teleop_direct.launch