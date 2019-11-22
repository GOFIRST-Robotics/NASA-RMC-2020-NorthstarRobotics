//
// Created by nick on 10/21/19.
//

#include "VESC.h"
#include "can_manager.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"

#define VESC_MAP_SIZE 16
VESC vesc_map[VESC_MAP_SIZE] = {};
int vesc_count = 0;

void vesc_system_init() {
  registerCANMsgHandler(0xFFFFFFFF, handle_vesc_can_recv);
}

void vesc_send_message(VESC* vesc, uint8_t type, uint8_t* buffer, int length) {
  do_send_can_message(type << 8 | vesc->id, buffer, length);
}

VESC* create_vesc(uint8_t id, int pole_pairs) {
  VESC* vesc = &vesc_map[vesc_count++];
  memset(vesc, 0, sizeof(VESC));
  vesc->id = id;
  vesc->pole_pairs = pole_pairs;
  return vesc;
}

// https://github.com/vedderb/bldc/blob/a141e750bb667cd828e9fd5e5b185724f22fae0b/comm_can.c#L1114
void handle_vesc_can_recv(rmc_can_msg msg) {
  uint8_t vesc_id = msg.id & 0xFF;
  VESC* ptr = NULL;
  for (int i = 0; i < vesc_count; ++i) {
    if (vesc_map[i].id == vesc_id) {
      ptr = &vesc_map[i];
      break;
    }
  }
  if (ptr != NULL) {
    uint8_t cmd_id = msg.id >> 8;
    int index = 0;
    uint8_t* buf = msg.buf;
    switch (cmd_id) {
      case VESC_PACKET_STATUS: {
        if (msg.length < 8) {
          break;
        }
        // To turn this into an actual RPM, divide by 6 for the turnigy motors
        ptr->erpm = (float)buffer_pop_int32(buf, &index);
        ptr->current = (float)buffer_pop_int16(buf, &index) / 10.0;
        ptr->duty = (float)buffer_pop_int16(buf, &index) / 1000.0;
        break;
      }
      case VESC_PACKET_STATUS_2: {
        if (msg.length < 8) {
          break;
        }
        ptr->amp_hours = (float)buffer_pop_int32(buf, &index) / 1e4;
        ptr->amp_hours_charged = (float)buffer_pop_int32(buf, &index) / 1e4;
        break;
      }
      case VESC_PACKET_STATUS_3: {
        if (msg.length < 8) {
          break;
        }
        ptr->watt_hours = (float)buffer_pop_int32(buf, &index) / 1e4;
        ptr->watt_hours_charged = (float)buffer_pop_int32(buf, &index) / 1e4;
        break;
      }
      case VESC_PACKET_STATUS_4: {
        if (msg.length < 8) {
          break;
        }
        ptr->temp_fet = (float)buffer_pop_int16(buf, &index) / 10.0;
        ptr->temp_motor = (float)buffer_pop_int16(buf, &index) / 10.0;
        ptr->current_in = (float)buffer_pop_int16(buf, &index) / 10.0;
        ptr->pid_pos_now = (float)buffer_pop_int16(buf, &index) / 50.0;
        break;
      }
      case VESC_PACKET_STATUS_5: {
        if (msg.length < 6) {
          break;
        }
        ptr->tacho_value = buffer_pop_int32(buf, &index);
        ptr->v_in = (float)buffer_pop_int16(buf, &index) / 1e1;
        break;
      }
    }
  }
}

void vesc_set_duty_cycle(VESC* vesc, float duty_cycle) {
  uint8_t buffer[4];
  int32_t val = (int32_t)(duty_cycle * 100000.0);
  int index = 0;
  buffer_put_int32(buffer, &index, val);
  vesc_send_message(vesc, VESC_PACKET_SET_DUTY, buffer, 4);
}

void vesc_set_rpm(VESC* vesc, float rpm) {
  // Normalize rpm to erpm
  rpm *= vesc->pole_pairs;
  uint8_t buffer[4];
  int32_t val = (int32_t)(rpm);
  int index = 0;
  buffer_put_int32(buffer, &index, val);
  vesc_send_message(vesc, VESC_PACKET_SET_RPM, buffer, 4);
}

void vesc_set_position(VESC* vesc, float pos) {
  // Multiply degrees to encoder counts
  pos *= (6 * vesc->pole_pairs);
  uint8_t buffer[4];
  int32_t val = (int32_t)(pos * 1000000.0);
  memcpy(buffer, &val, sizeof(uint8_t) * 4);
  vesc_send_message(vesc, VESC_PACKET_SET_POS, buffer, 4);
}

void vesc_set_current(VESC* vesc, float current) {
  uint8_t buffer[4];
  int32_t val = (int32_t)(current * 1000.0);
  memcpy(buffer, &val, sizeof(uint8_t) * 4);
  vesc_send_message(vesc, VESC_PACKET_SET_CURRENT, buffer, 4);
}

float vesc_get_rpm(VESC* vesc) { return vesc->erpm / vesc->pole_pairs; }

float vesc_get_position(VESC* vesc) {
  return 360.0F * vesc->tacho_value / (6 * vesc->pole_pairs);
}

int32_t buffer_pop_int32(uint8_t* buffer, int* index) {
  int32_t buf;
  buf = buffer[(*index)++] << 24;
  buf |= buffer[(*index)++] << 16;
  buf |= buffer[(*index)++] << 8;
  buf |= buffer[(*index)++];
  return buf;
}

int16_t buffer_pop_int16(uint8_t* buffer, int* index) {
  int16_t buf;
  buf |= buffer[(*index)++] << 8;
  buf |= buffer[(*index)++];
  return buf;
}

void buffer_put_int32(uint8_t* buffer, int* index, int32_t number) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_put_int16(uint8_t* buffer, int* index, int16_t value) {
  buffer[(*index)++] = (value << 8) & 0xFF;
  buffer[(*index)++] = value & 0xFF;
}
