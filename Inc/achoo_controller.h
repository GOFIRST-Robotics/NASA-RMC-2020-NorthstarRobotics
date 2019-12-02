//
// Created by nick on 11/5/19.
//

#ifndef NASA_RMC_RT_ACHOO_CONTROLLER_H
#define NASA_RMC_RT_ACHOO_CONTROLLER_H

#include "can_manager.h"

typedef enum { STANDING = 0, MOVING_STAND, MOVING_KNEEL, KNEELING } KneelState;

#define ACHOO_MOTOR_POLE_PAIRS 7
#define ACHOO_MSG_SET_KNEEL 40u
#define ACHOO_MSG_STATUS 41u

#define ACHOO_LOOP_MS 100
#define ACHOO_KNEEL_SETPOINT 0     // mm
#define ACHOO_STAND_SETPOINT 20    // mm
#define ACHOO_DEG_MM_CONV 0.00278  // mm per degree
#define ACHOO_ERROR_THRESHOLD 1    // mm

void achooCANCallback(rmc_can_msg msg);

float getACHOOError();
KneelState getACHOOState();

#endif  // NASA_RMC_RT_ACHOO_CONTROLLER_H
