#ifndef NASA_RMC_RT_RELAY_H
#define NASA_RMC_RT_RELAY_H
#include <stm32f3xx_hal.h>
#include "types.h"

typedef enum {
  RELAY_OFF = 0,
  RELAY_FORWARD,
  RELAY_REVERSE,
  RELAY_ON
} RelayState;

typedef struct {
  GPIO_TypeDef* port_fw;
  U16 pin_fw;
  GPIO_TypeDef* port_rv;
  U16 pin_rv;
  RelayState state;
} Relay;

Relay create_relay(GPIO_TypeDef* port_fw, U16 pin_fw, GPIO_TypeDef* port_rv,
                   U16 pin_rv);
void set_relay(Relay* relay, RelayState state);

#endif  // NASA_RMC_RT_RELAY_H
