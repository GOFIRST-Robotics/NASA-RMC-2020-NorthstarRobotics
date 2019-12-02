//
// Created by nick on 11/5/19.
//

#include "achoo_controller.h"
#include <FreeRTOS.h>
#include <can_manager.h>
#include <math.h>
#include <rt_conf.h>
#include <task.h>
#include "VESC.h"
#include "stdlib.h"

KneelState targetState = KNEELING;
KneelState currentState = KNEELING;
float pos_target = ACHOO_STAND_SETPOINT;
VESC* leftMotor;
VESC* rightMotor;

void achooCANCallback(rmc_can_msg msg) {
  // Because of the mask we only get messages that have our ID
  U32 msg_type = msg.id >> 8u;
  if (msg_type == ACHOO_MSG_SET_KNEEL && msg.length >= 1) {
    U8 param = msg.buf[0];
    switch (param) {
      case 1:
        targetState = KNEELING;
        break;
      default:
      case 0:
        targetState = STANDING;
    }
  }
}

void achooControllerFunc(void const* argument) {
  // System setup
  registerCANMsgHandler(ACHOO_SYS_ID, &achooCANCallback);
  leftMotor = create_vesc(ACHOO_MOTOR_L, ACHOO_MOTOR_POLE_PAIRS);
  rightMotor = create_vesc(ACHOO_MOTOR_R, ACHOO_MOTOR_POLE_PAIRS);

  TickType_t lastWakeTime;
  while (1) {
    vTaskDelayUntil(&lastWakeTime, ACHOO_LOOP_MS * portTICK_RATE_MS);

    // Handle target state transitions
    if (targetState == KNEELING &&
        (currentState == STANDING || currentState == MOVING_STAND)) {
      pos_target = ACHOO_KNEEL_SETPOINT;
      currentState = MOVING_KNEEL;
    } else if (targetState == STANDING &&
               (currentState == KNEELING || currentState == MOVING_KNEEL)) {
      pos_target = ACHOO_STAND_SETPOINT;
      currentState = MOVING_STAND;
    }
    // Check if our movement has completed
    bool inThreshold = fabsf(getACHOOError()) < ACHOO_ERROR_THRESHOLD;
    if (currentState == MOVING_KNEEL && inThreshold) {
      currentState = KNEELING;
    } else if (currentState == MOVING_STAND && inThreshold) {
      currentState = STANDING;
    }

    // Continuously set VESC position PID target
    vesc_set_position(leftMotor, pos_target / ACHOO_DEG_MM_CONV);
    vesc_set_position(rightMotor, pos_target / ACHOO_DEG_MM_CONV);

    // Send status message
    U8 data[1];
    data[0] = currentState;
    do_send_can_message((ACHOO_MSG_STATUS << 8u) | ACHOO_SYS_ID, data, 1);
  }
}

float getACHOOError() {
  float lerr = pos_target - vesc_get_position(leftMotor) * ACHOO_DEG_MM_CONV;
  float rerr = pos_target - vesc_get_position(rightMotor) * ACHOO_DEG_MM_CONV;
  return (lerr + rerr) / 2;
}

KneelState getACHOOState() { return currentState; }