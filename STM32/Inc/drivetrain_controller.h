//
// Created by hunter on 11/15/19.
//

#ifndef NASA_RMC_RT_DRIVETRAIN_CONTROLLER_H
#define NASA_RMC_RT_DRIVETRAIN_CONTROLLER_H

#include "can_manager.h"
// see
// https://husarion.com/tutorials/ros-tutorials/3-simple-kinematics-for-mobile-robot/
// for an explanation on how the mathematics for the twist were calculated

// ALL VALUES ARE IN MILLIMETERS
#define DT_WHEEL_RADIUS 180.0f
#define DT_WIDTH 1000.0f
#define DT_GEAR_RATIO 80
#define DT_MMS_TO_RPM (DT_GEAR_RATIO / (DT_WHEEL_RADIUS * 0.1047f))
typedef enum { DRIVE_MSG_TWIST = 35 } DRIVE_MSG_T;

#define DRIVE_LOOP_MS 100u  // milliseconds, unsigned
#define DRIVE_MOTOR_POLE_PAIRS 7u

#endif  // NASA_RMC_RT_DRIVETRAIN_CONTROLLER_H
