#include "relay.h"

Relay create_relay(GPIO_TypeDef* port_fw, U16 pin_fw, GPIO_TypeDef* port_rv,
                   U16 pin_rv) {
  Relay relay = {port_fw, pin_fw, port_rv, pin_rv, RELAY_OFF};
  return relay;
}

void set_relay(Relay* relay, RelayState state) {
  relay->state = state;
  if (state == RELAY_OFF) {
    HAL_GPIO_WritePin(relay->port_fw, relay->pin_fw, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(relay->port_rv, relay->pin_rv, GPIO_PIN_RESET);
  } else if (state == RELAY_FORWARD) {
    HAL_GPIO_WritePin(relay->port_fw, relay->pin_fw, GPIO_PIN_SET);
    HAL_GPIO_WritePin(relay->port_rv, relay->pin_rv, GPIO_PIN_RESET);
  } else if (state == RELAY_REVERSE) {
    HAL_GPIO_WritePin(relay->port_fw, relay->pin_fw, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(relay->port_rv, relay->pin_rv, GPIO_PIN_SET);
  } else if (state == RELAY_ON) {
    HAL_GPIO_WritePin(relay->port_fw, relay->pin_fw, GPIO_PIN_SET);
    HAL_GPIO_WritePin(relay->port_rv, relay->pin_rv, GPIO_PIN_SET);
  }
}