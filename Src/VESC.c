//
// Created by nick on 10/21/19.
//

#include "VESC.h"
#include "can_manager.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "types.h"

#define VESC_MAP_SIZE 16
VESC vesc_map[VESC_MAP_SIZE] = {{0}};
static S32 vesc_count = 0;

void vesc_system_init() {
  registerCANMsgHandler(0xFFFFFFFFu, handle_vesc_can_recv);
}

void vesc_send_message(VESC const* vesc, U8 const type, U8 const* buffer,
                       S32 const length) {
  do_send_can_message(type << 8u | vesc->id, buffer, length);
}

VESC* create_vesc(U8 const id, S32 const pole_pairs) {
  VESC* vesc = &vesc_map[vesc_count];
  ++vesc_count;
  vesc->id = id;
  vesc->pole_pairs = pole_pairs;
  return vesc;
}

// https://github.com/vedderb/bldc/blob/a141e750bb667cd828e9fd5e5b185724f22fae0b/comm_can.c#L1114
void handle_vesc_can_recv(rmc_can_msg const msg) {
  U8 const vesc_id = msg.id & 0xFFu;
  VESC* ptr = NULL;
  for (S32 i = 0; i < vesc_count; ++i) {
    if (vesc_map[i].id == vesc_id) {
      ptr = &vesc_map[i];
      break;
    }
  }
  if (ptr != NULL) {
    U8 cmd_id = msg.id >> 8u;
    S32 index = 0;
    U8 const* buf = msg.buf;
    switch (cmd_id) {
      case VESC_PACKET_STATUS: {
        if (msg.length < 8) {
          break;
        }
        // To turn this into an actual RPM, divide by 6 for the turnigy motors
        ptr->erpm = (F32)buffer_pop_int32(buf, &index);
        ptr->current = (F32)buffer_pop_int16(buf, &index) / 10.0f;
        ptr->duty = (F32)buffer_pop_int16(buf, &index) / 1000.0f;
        break;
      }
      case VESC_PACKET_STATUS_2: {
        if (msg.length < 8) {
          break;
        }
        ptr->amp_hours = (F32)buffer_pop_int32(buf, &index) / 1e4f;
        ptr->amp_hours_charged = (F32)buffer_pop_int32(buf, &index) / 1e4f;
        break;
      }
      case VESC_PACKET_STATUS_3: {
        if (msg.length < 8) {
          break;
        }
        ptr->watt_hours = (F32)buffer_pop_int32(buf, &index) / 1e4f;
        ptr->watt_hours_charged = (F32)buffer_pop_int32(buf, &index) / 1e4f;
        break;
      }
      case VESC_PACKET_STATUS_4: {
        if (msg.length < 8) {
          break;
        }
        ptr->temp_fet = (F32)buffer_pop_int16(buf, &index) / 10.0f;
        ptr->temp_motor = (F32)buffer_pop_int16(buf, &index) / 10.0f;
        ptr->current_in = (F32)buffer_pop_int16(buf, &index) / 10.0f;
        ptr->pid_pos_now = (F32)buffer_pop_int16(buf, &index) / 50.0f;
        break;
      }
      case VESC_PACKET_STATUS_5: {
        if (msg.length < 6) {
          break;
        }
        ptr->tacho_value = buffer_pop_int32(buf, &index);
        ptr->v_in = (F32)buffer_pop_int16(buf, &index) / 1e1f;
        break;
      }
    }
  }
}

void vesc_set_duty_cycle(VESC const* vesc, F32 const duty_cycle) {
  U8 buffer[4];
  U32 val = (U32)(duty_cycle * 100000.0);
  S32 index = 0;
  buffer_put_int32(buffer, &index, val);
  vesc_send_message(vesc, VESC_PACKET_SET_DUTY, buffer, 4);
}

void vesc_set_rpm(VESC const* vesc, F32 const rpm) {
  // Normalize rpm to erpm
  U8 buffer[4];
  U32 val = (U32)(rpm * vesc->pole_pairs);
  S32 index = 0;
  buffer_put_int32(buffer, &index, val);
  vesc_send_message(vesc, VESC_PACKET_SET_RPM, buffer, 4);
}

void vesc_set_position(VESC const* vesc, F32 const pos) {
  // Multiply degrees to encoder counts
  U8 buffer[4];
  U32 val = (U32)(6.0f * vesc->pole_pairs * 1000000.0f);
  memcpy(buffer, &val, sizeof(U8) * 4);
  vesc_send_message(vesc, VESC_PACKET_SET_POS, buffer, 4);
}

void vesc_set_current(VESC const* vesc, F32 const current) {
  U8 buffer[4];
  U32 val = (U32)(current * 1000.0);
  memcpy(buffer, &val, sizeof(U8) * 4);
  vesc_send_message(vesc, VESC_PACKET_SET_CURRENT, buffer, 4);
}

F32 vesc_get_rpm(VESC const* vesc) { return vesc->erpm / vesc->pole_pairs; }

F32 vesc_get_position(VESC const* vesc) {
  return 360.0f * vesc->tacho_value / (6.0f * vesc->pole_pairs);
}

S32 buffer_pop_int32(U8 const* buffer, S32* index) {
  S32 buf;
  buf = buffer[(*index)++] << 24;
  buf |= buffer[(*index)++] << 16;
  buf |= buffer[(*index)++] << 8;
  buf |= buffer[(*index)++];
  return buf;
}

int16_t buffer_pop_int16(U8 const* buffer, S32* index) {
  int16_t buf;
  buf |= buffer[(*index)++] << 8;
  buf |= buffer[(*index)++];
  return buf;
}

void buffer_put_int32(U8* buffer, S32* index, S32 const value) {
  buffer[(*index)++] = value >> 24u;
  buffer[(*index)++] = value >> 16u;
  buffer[(*index)++] = value >> 8u;
  buffer[(*index)++] = value;
}

void buffer_put_int16(U8* buffer, S32* index, S16 const value) {
  buffer[(*index)++] = value >> 8u;
  buffer[(*index)++] = value;
}
