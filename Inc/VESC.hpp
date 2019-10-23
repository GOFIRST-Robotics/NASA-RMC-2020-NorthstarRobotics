//
// Created by nick on 10/21/19.
//

#ifndef NASA_RMC_RT_VESC_HPP
#define NASA_RMC_RT_VESC_HPP

#include <cstdint>

#define VESC_PACKET_SET_CURRENT 1
#define VESC_PACKET_SET_RPM 3
#define VESC_PACKET_SET_POS 4 
#define VESC_PACKET_SET_DUTY 0
#define VESC_PACKET_STATUS 9 
#define VESC_PACKET_STATUS_2 14 
#define VESC_PACKET_STATUS_3 15 
#define VESC_PACKET_STATUS_4 16 
#define VESC_PACKET_PING 17
#define VESC_PACKET_PONG 18

class VESC {
public:
    explicit VESC(uint8_t id);

protected:
    int SendMessage(uint8_t type, uint8_t buffer[], int length);

    uint8_t id;
};


#endif //NASA_RMC_RT_VESC_HPP
