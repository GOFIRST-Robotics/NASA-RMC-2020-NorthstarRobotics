//
// Created by julia on 11/5/19.
//

#include "gesundheit_controller.h"
#include <can_manager.h>
#include <main.h>
#include <rt_conf.h>
#include "VESC.h"
#include "buffer.h"

#include <FreeRTOS.h>
#include <relay.h>
#include <stm32f3xx_hal_conf.h>
#include <task.h>

static ExtendState targetState = STOWED;
static ExtendState currentState = STOWED;
static bool doorOpen = false;
static F32 targetRPM = 0.0f;
static VESC* gesundheitMotor;
static Relay extension;
static Relay flipl;
static Relay flipr;
static Relay door;

void gesundheitCANCallback(rmc_can_msg msg) {
  // Because of the mask we only get messages that have our ID
  U32 msg_type = msg.id >> 8u;
  if (msg_type == GESUNDHEIT_MSG_SET_EXTEND && msg.length >= 1) {
    U8 param = msg.buf[0];
    switch (param) {
      case 1:
        targetState = EXTENDED;
        break;
      default:
      case 0:
        targetState = STOWED;
    }
  } else if (msg_type == GESUNDHEIT_MSG_SET_SPEED && msg.length >= 4) {
    S32 idx = 0;
    targetRPM = (F32)buffer_pop_int32(msg.buf, &idx);
  } else if (msg_type == GESUNDHEIT_MSG_SET_DOOR && msg.length >= 1) {
    U8 param = msg.buf[0];
    doorOpen = param > 0;
  }
}

void gesundheitControllerFunc(void const* argument) {
  // System setup
  registerCANMsgHandler(GESUNDHEIT_SYS_ID, &gesundheitCANCallback);
  gesundheitMotor =
      create_vesc(GESUNDHEIT_MOTOR_ID, GESUNDHEIT_MOTOR_POLE_PAIRS);
  extension = create_relay(R_GESUNDEXT_FW_GPIO_Port, R_GESUNDEXT_FW_Pin,
                           R_GESUNDEXT_RV_GPIO_Port, R_GESUNDEXT_RV_Pin);
  flipl = create_relay(R_GESUNDL_FW_GPIO_Port, R_GESUNDL_FW_Pin,
                       R_GESUNDL_RV_GPIO_Port, R_GESUNDL_RV_Pin);
  flipr = create_relay(R_GESUNDR_FW_GPIO_Port, R_GESUNDR_FW_Pin,
                       R_GESUNDR_RV_GPIO_Port, R_GESUNDR_RV_Pin);
  door = create_relay(R_DOOR_FW_GPIO_Port, R_DOOR_FW_Pin, R_DOOR_RV_GPIO_Port,
                      R_DOOR_RV_Pin);

  TickType_t lastWakeTime;
  while (1) {
    vTaskDelayUntil(&lastWakeTime, GESUNDHEIT_LOOP_MS * portTICK_RATE_MS);
    // Handle target state transitions
    if (targetState == STOWED &&
        (currentState == EXTENDED || currentState == MOVING_EXTEND)) {
      currentState = MOVING_STOW;
    } else if (targetState == EXTENDED &&
               (currentState == STOWED || currentState == MOVING_STOW)) {
      currentState = MOVING_EXTEND;
    }
    bool lowLimitR = HAL_GPIO_ReadPin(GESUNDHEIT_LimitRI_GPIO_Port,
                                      GESUNDHEIT_LimitRI_Pin) == 0;
    bool lowLimitL = HAL_GPIO_ReadPin(GESUNDHEIT_LimitLI_GPIO_Port,
                                      GESUNDHEIT_LimitLI_Pin) == 0;
    bool highLimitR = HAL_GPIO_ReadPin(GESUNDHEIT_LimitRO_GPIO_Port,
                                       GESUNDHEIT_LimitRO_Pin) == 0;
    bool highLimitL = HAL_GPIO_ReadPin(GESUNDHEIT_LimitLO_GPIO_Port,
                                       GESUNDHEIT_LimitLO_Pin) == 0;
    // Check if our movement has completed
    if (lowLimitR && lowLimitL && currentState == MOVING_STOW) {
      currentState = STOWED;
    }
    if (highLimitR && highLimitL && currentState == MOVING_EXTEND) {
      currentState = EXTENDED;
    }

    // Linear actuators have built in stops
    if (currentState == MOVING_EXTEND || currentState == EXTENDED) {
      set_relay(&extension, RELAY_FORWARD);
    } else {
      set_relay(&extension, RELAY_REVERSE);
    }

    // Set relays for motors
    if (currentState == MOVING_EXTEND && !highLimitR) {
      set_relay(&flipr, RELAY_FORWARD);
    } else if (currentState == MOVING_STOW && !lowLimitR) {
      set_relay(&flipr, RELAY_REVERSE);
    }
    if (currentState == MOVING_EXTEND && !highLimitL) {
      set_relay(&flipl, RELAY_FORWARD);
    } else if (currentState == MOVING_STOW && !lowLimitL) {
      set_relay(&flipl, RELAY_REVERSE);
    }

    // Set RPM
    vesc_set_rpm(gesundheitMotor, targetRPM);

    // Set door (linear actuator)
    if (doorOpen) {
      set_relay(&door, RELAY_FORWARD);
    } else {
      set_relay(&door, RELAY_REVERSE);
    }

    // Send status message
    U8 data[6];
    data[0] = currentState;
    data[1] = doorOpen;
    S32 rpm = (S32)vesc_get_rpm(gesundheitMotor);
    S32 idx = 2;
    buffer_put_int32(data, &idx, rpm);
    do_send_can_message((GESUNDHEIT_MSG_STATUS << 8u) | GESUNDHEIT_SYS_ID, data,
                        1);
  }
}

ExtendState getGESUNDHEITState() { return currentState; }
