//
// Created by hunter on 11/15/19.
//

#include "drivetrain_controller.h"
#include <FreeRTOS.h>
#include <can_manager.h>
#include <math.h>
#include <print.h>
#include <rt_conf.h>
#include <stdio.h>
#include <stm32f3xx_hal.h>
#include <string.h>
#include <task.h>
#include "VESC.h"
#include "stdlib.h"

// I am telling the motor controllers to implement a twist message until I get
// another twist message
F32 new_speed_left = 0;
F32 new_speed_right = 0;

void drivetrain_move(rmc_can_msg msg) {
  // Because of the mask we only get messages that have our ID
  S32 cmd_speed = 0;  // in mm/s
  S32 cmd_angV = 0;

  //    if(msg.id & 0xFF != DRIVETRAIN_SYS_ID)
  //    {
  //        return;
  //    }
  // This portion is already done by the CAN dispatcher. It is here purely for
  // Hunter's understanding.

  S32 idx = 0;
  switch (msg.id >> 8) {
    case DRIVE_MSG_TWIST:

      cmd_angV = buffer_pop_int32(msg.buf, &idx);
      cmd_speed = buffer_pop_int32(msg.buf, &idx);

      new_speed_right = (cmd_angV * DT_WIDTH) / 2 + cmd_speed;
      new_speed_left = cmd_speed * 2 - new_speed_right;
      new_speed_right = new_speed_right * DT_MMS_TO_RPM;
      new_speed_left = new_speed_left * DT_MMS_TO_RPM;
      break;

    default:
      // weewoo something went wrong
      break;
  }
}

void drivetrain_loop(void const* argument) {
  // SETUP
  registerCANMsgHandler(DRIVETRAIN_SYS_ID, &drivetrain_move);
  VESC* blm = create_vesc(DRIVE_MOTOR_BL, DRIVE_MOTOR_POLE_PAIRS);
  VESC* brm = create_vesc(DRIVE_MOTOR_BR, DRIVE_MOTOR_POLE_PAIRS);
  VESC* frm = create_vesc(DRIVE_MOTOR_FR, DRIVE_MOTOR_POLE_PAIRS);
  VESC* flm = create_vesc(DRIVE_MOTOR_FL, DRIVE_MOTOR_POLE_PAIRS);

  TickType_t lastWakeTime;
  // END SETUP

  while (1) {
    vTaskDelayUntil(
        &lastWakeTime,
        DRIVE_LOOP_MS * portTICK_RATE_MS);  // Not sure what this does yet,
    // min 1hz refresh rate

    vesc_set_rpm(blm, new_speed_left);
    vesc_set_rpm(flm, new_speed_left);
    vesc_set_rpm(brm, new_speed_right);
    vesc_set_rpm(frm, new_speed_right);

    // now publish some optometry data
  }
}