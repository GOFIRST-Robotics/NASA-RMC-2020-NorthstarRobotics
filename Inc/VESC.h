//
// Created by nick on 10/21/19.
//

#ifndef NASA_RMC_RT_VESC_H
#define NASA_RMC_RT_VESC_H

#include "stdint.h"
#include "can_manager.hpp"

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

#define VESC_TACHO_CPR 42

typedef struct {
    uint8_t id;
    int pole_pairs;
    float erpm;
    float current;
    float duty;
    float amp_hours;
    float amp_hours_charged;
    float watt_hours;
    float watt_hours_charged;
    float temp_fet;
    float temp_motor;
    float current_in;
    float pid_pos_now;
    int32_t tacho_value;
    float v_in;
} VESC;

VESC* create_VESC(uint8_t id, int pole_pairs);
void vesc_send_message(VESC* vesc, uint8_t type, uint8_t* message, int length);

void vesc_system_init();
void handle_vesc_can_recv(rmc_can_msg msg);

/**
 * Pops a value off the buffer starting at index of the specified length and increments index to match
 */
int32_t buffer_pop_int32(uint8_t* buffer, int* index);
/**
 * Pops a value off the buffer starting at index of the specified length and increments index to match
 */
int16_t buffer_pop_int16(uint8_t* buffer, int* index);
/**
 * Puts a value into the buffer starting at index of the specified length and increments index to match
 */
void buffer_put_int32(uint8_t* buffer, int* index, int32_t value);
/**
 * Puts a value into the the buffer starting at index of the specified length and increments index to match
 */
void buffer_put_int16(uint8_t* buffer, int* index, int16_t value);

/**
 * Set duty cycle to specified VESC
 * @param vesc
 * @param duty_cycle Duty cycle in ?
 */
void vesc_set_duty_cycle(VESC* vesc, float duty_cycle);
/**
 * Set target velocity to selected VESC
 * @param vesc
 * @param rpm Velocity in RPM
 */
void vesc_set_rpm(VESC* vesc, float rpm);
/**
 * Set current setpoint to specified VESC
 * @param vesc
 * @param current Current in Amperes
 */
void vesc_set_current(VESC* vesc, float current);
/**
 * Set position
 * @param vesc
 * @param position Position in ?
 */
void vesc_set_position(VESC* vesc, float position);

/**
 * Get the actual RPM of the motor as returned by the VESC
 * @param vesc
 * @return The latest speed of the motor
 */
float vesc_get_rpm(VESC* vesc);

/**
 * Get the position of the motor in revolutions
 * @param vesc
 * @return The position of the motor
 */
float vesc_get_position(VESC* vesc);


#endif //NASA_RMC_RT_VESC_H
