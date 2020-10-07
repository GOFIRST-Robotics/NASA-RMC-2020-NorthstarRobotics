#ifndef NASA_RMC_RT_GESUNDHEIT_CONTROLLER_H
#define NASA_RMC_RT_GESUNDHEIT_CONTROLLER_H

#include "can_manager.h"

typedef enum { STOWED = 0, MOVING_STOW, MOVING_EXTEND, EXTENDED } ExtendState;

#define GESUNDHEIT_MOTOR_POLE_PAIRS 7

#define GESUNDHEIT_MSG_SET_EXTEND 50u
#define GESUNDHEIT_MSG_SET_SPEED 51u
#define GESUNDHEIT_MSG_SET_DOOR 52u
#define GESUNDHEIT_MSG_STATUS 53u

#define GESUNDHEIT_LOOP_MS 20u

void gesundheitCANCallback(rmc_can_msg msg);

ExtendState getGesundheitState();

#endif  // NASA_RMC_RT_GESUNDHEIT_CONTROLLER_H
