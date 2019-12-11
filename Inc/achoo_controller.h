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

#define ACHOO_LOOP_MS 20u

void achooCANCallback(rmc_can_msg msg);

KneelState getACHOOState();

#endif  // NASA_RMC_RT_ACHOO_CONTROLLER_H
