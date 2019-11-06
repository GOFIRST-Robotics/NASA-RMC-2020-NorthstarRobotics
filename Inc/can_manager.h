//
// Created by nick on 10/22/19.
//
#ifndef NASA_RMC_RT_CAN_MANAGER_H
#define NASA_RMC_RT_CAN_MANAGER_H

#include "stdint.h"
#include "stdbool.h"

#define CAN_ALL_TYPES_MASK 0x7FFFFF00
#define CAN_HANDLERS_SIZE 10

typedef struct {unsigned int id; uint8_t buf[8]; int length;} rmc_can_msg;

void do_send_can_message(unsigned int id, uint8_t* buf, int length);
bool enque_can_message(unsigned int id, uint8_t* buf, int length);

typedef struct {unsigned int mask; void (*callback)(rmc_can_msg msg);} CANMsgHandlerPair;

void registerCANMsgHandler(unsigned int mask, void (*callback)(rmc_can_msg msg));

#endif //NASA_RMC_RT_CAN_MANAGER_H