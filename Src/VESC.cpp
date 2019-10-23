//
// Created by nick on 10/21/19.
//

#include "VESC.hpp"
#include "can_manager.hpp"

int VESC::SendMessage(uint8_t type, uint8_t *buffer, int length) {
    enque_can_message(type << 8 | id, buffer, length);
}

VESC::VESC(uint8_t id) {
    this->id = id;
}
