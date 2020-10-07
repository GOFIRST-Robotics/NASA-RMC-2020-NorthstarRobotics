#include "buffer.h"

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