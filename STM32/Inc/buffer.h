#ifndef NASA_RMC_RT_BUFFER_H
#define NASA_RMC_RT_BUFFER_H

#include "types.h"

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

#endif  // NASA_RMC_RT_BUFFER_H
