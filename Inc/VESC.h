//
// Created by nick on 10/21/19.
//

#ifndef NASA_RMC_RT_VESC_H
#define NASA_RMC_RT_VESC_H

#include "can_manager.h"
#include "stdint.h"
#include "types.h"

#define VESC_PACKET_SET_CURRENT 1
#define VESC_PACKET_SET_RPM 3
#define VESC_PACKET_SET_POS 4
#define VESC_PACKET_SET_DUTY 0
#define VESC_PACKET_STATUS 9
#define VESC_PACKET_STATUS_2 14
#define VESC_PACKET_STATUS_3 15
#define VESC_PACKET_STATUS_4 16
#define VESC_PACKET_STATUS_5 27
#define VESC_PACKET_PING 17
#define VESC_PACKET_PONG 18

typedef struct {
  U8 id;
  S32 pole_pairs;
  F32 erpm;
  F32 current;
  F32 duty;
  F32 amp_hours;
  F32 amp_hours_charged;
  F32 watt_hours;
  F32 watt_hours_charged;
  F32 temp_fet;
  F32 temp_motor;
  F32 current_in;
  F32 pid_pos_now;
  S32 tacho_value;
  F32 v_in;
} VESC;

VESC* create_vesc(U8 id, S32 pole_pairs);
void vesc_send_message(VESC const* vesc, U8 type, U8 const* message,
                       S32 length);

void vesc_system_init();
void handle_vesc_can_recv(rmc_can_msg msg);

/**
 * Pops a value off the buffer starting at index of the specified length and
 * increments index to match
 */
S32 buffer_pop_int32(U8 const* buffer, S32* index);
/**
 * Pops a value off the buffer starting at index of the specified length and
 * increments index to match
 */
S16 buffer_pop_int16(U8 const* buffer, S32* index);
/**
 * Puts a value into the buffer starting at index of the specified length and
 * increments index to match
 */
void buffer_put_int32(U8* buffer, S32* index, S32 value);
/**
 * Puts a value into the the buffer starting at index of the specified length
 * and increments index to match
 */
void buffer_put_int16(U8* buffer, S32* index, S16 value);

/**
 * Set duty cycle to specified VESC
 * @param vesc
 * @param duty_cycle Duty cycle in ?
 */
void vesc_set_duty_cycle(VESC const* vesc, F32 duty_cycle);
/**
 * Set target velocity to selected VESC
 * @param vesc
 * @param rpm Velocity in RPM
 */
void vesc_set_rpm(VESC const* vesc, F32 rpm);
/**
 * Set current setpoint to specified VESC
 * @param vesc
 * @param current Current in Amperes
 */
void vesc_set_current(VESC const* vesc, F32 current);
/**
 * Set target position
 * @param vesc
 * @param position Position in degrees
 */
void vesc_set_position(VESC const* vesc, F32 position);

/**
 * Get the actual RPM of the motor as returned by the VESC
 * @param vesc
 * @return The latest speed of the motor
 */
F32 vesc_get_rpm(VESC const* vesc);

/**
 * Get the position of the motor in degrees
 * @param vesc
 * @return The position of the motor
 */
F32 vesc_get_position(VESC const* vesc);

#endif  // NASA_RMC_RT_VESC_H
