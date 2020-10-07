#ifndef NASA_RMC_RT_SNEEZE_CONTROLLER_H
#define NASA_RMC_RT_SNEEZE_CONTROLLER_H

#include "can_manager.h"

#define SNEEZE_MOTOR_POLE_PAIRS 7
#define SNEEZE_MSG_STATUS 60u
#define SNEEZE_MSG_SET_DIG_SPEED 61u
#define SNEEZE_MSG_GO_HOME 62u
#define SNEEZE_MSG_SET_TRANS_SPEED 63u

#define SNEEZE_LOOP_MS 20u

typedef enum { IDLE = 0, DIGGING, HOMING } SneezeState;

void tissueCANCallback(rmc_can_msg msg);

bool isSNEEZEHome();

#endif  // NASA_RMC_RT_SNEEZE_CONTROLLER_H
