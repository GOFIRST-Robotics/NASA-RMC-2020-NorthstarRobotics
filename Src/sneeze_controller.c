#include "sneeze_controller.h"
#include <can_manager.h>
#include <main.h>
#include <rt_conf.h>
#include "VESC.h"

#include <FreeRTOS.h>
#include <stm32f3xx_hal_conf.h>
#include <task.h>

static VESC* digMotor;
static VESC* transMotor;
static SneezeState state;
static bool isHomed = false;
static F32 digSpeed = 0.0f;
static F32 transSpeed = 0.0f;

void sneezeCANCallback(rmc_can_msg msg) {
  // Because of the mask we only get messages that have our ID
  U32 msg_type = msg.id >> 8u;
  if (msg_type == SNEEZE_MSG_SET_DIG_SPEED && msg.length >= 4) {
    state = DIGGING;
    S32 idx = 0;
    digSpeed = (F32)buffer_pop_int32(msg.buf, &idx);
    if (digSpeed == 0) {
      state = IDLE;
    }
  } else if (msg_type == SNEEZE_MSG_GO_HOME) {
    if (!(state == IDLE && isHomed)) {
      state = HOMING;
    }
  } else if (msg_type == SNEEZE_MSG_SET_TRANS_SPEED && msg.length >= 4) {
    S32 idx = 0;
    transSpeed = (F32)buffer_pop_int32(msg.buf, &idx);
  }
}

void sneezeControllerFunc(void const* argument) {
  // System setup
  registerCANMsgHandler(SNEEZE_SYS_ID, &sneezeCANCallback);
  digMotor = create_vesc(SNEEZE_MOTOR_DIG, SNEEZE_MOTOR_POLE_PAIRS);
  transMotor = create_vesc(SNEEZE_MOTOR_TRANS, SNEEZE_MOTOR_POLE_PAIRS);

  TickType_t lastWakeTime;
  while (1) {
    vTaskDelayUntil(&lastWakeTime, SNEEZE_LOOP_MS * portTICK_RATE_MS);
    // Handle target state transitions
    isHomed = HAL_GPIO_ReadPin(SNEEZE_Home_GPIO_Port, SNEEZE_Home_Pin);

    if (state == HOMING && isHomed) {
      state = IDLE;
    }

    if (state == HOMING) {
      vesc_set_rpm(digMotor, 100);
    } else if (state == DIGGING) {
      vesc_set_rpm(digMotor, digSpeed);
    } else {
      vesc_set_rpm(digMotor, 0);
    }

    vesc_set_rpm(transMotor, transSpeed);

    // Send status message
    U8 data[6];
    data[0] = state;
    data[1] = isHomed;
    S32 idx = 2;
    buffer_put_int32(data, &idx, (S32)vesc_get_rpm(digMotor));
    do_send_can_message((SNEEZE_MSG_STATUS << 8u) | SNEEZE_SYS_ID, data, 6);
  }
}

bool isSNEEZEHome() { return isHomed; }