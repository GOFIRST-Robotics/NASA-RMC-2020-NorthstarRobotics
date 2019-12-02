//
// Created by nick on 10/22/19.
//
#ifndef NASA_RMC_RT_CAN_MANAGER_H
#define NASA_RMC_RT_CAN_MANAGER_H

#include "stdbool.h"
#include "stdint.h"
#include "types.h"

#define CAN_ALL_TYPES_MASK 0x7FFFFF00u
#define CAN_HANDLERS_SIZE 10

typedef struct {
  U32 id;
  U8 buf[8];
  S32 length;
} rmc_can_msg;

void do_send_can_message(U32 id, U8 const* buf, S32 length);

typedef struct {
  unsigned int mask;
  void (*callback)(rmc_can_msg msg);
} CANMsgHandlerPair;

void registerCANMsgHandler(U32 mask, void (*callback)(rmc_can_msg msg));

#endif  // NASA_RMC_RT_CAN_MANAGER_H