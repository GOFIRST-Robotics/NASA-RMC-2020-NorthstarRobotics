//
// Created by nick on 10/22/19.
//

extern "C" {
#include <stm32f3xx.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
}
#include <cstring>
#include <vector>
#include "can_manager.hpp"


extern "C" CAN_HandleTypeDef hcan;
extern "C" QueueHandle_t xCanTxQueue;
uint32_t TxMailbox;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];
std::vector<CANMsgHandlerPair> msgHandlers;

void do_send_can_message(unsigned int id, uint8_t* buf, int length) {
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.ExtId = id;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = length;
    TxHeader.TransmitGlobalTime = DISABLE;
    //todo mutex?
    int retval = HAL_CAN_AddTxMessage(&hcan, &TxHeader, buf, &TxMailbox);
}

bool enque_can_message(unsigned int id, uint8_t* buf, int length) {
    rmc_can_msg message;
    message.id = id;
    memcpy(&message.buf, buf, length * sizeof(uint8_t));
    message.length = length;
    return xQueueSend(xCanTxQueue, &message, 0) == pdTRUE;
}

extern "C" {
void canTxTaskFunc(void *params) {
    TickType_t lastWakeTime;
    while (1) {
        vTaskDelayUntil(&lastWakeTime, 1 * portTICK_RATE_MS);

        rmc_can_msg message;
        if (xQueueReceive(xCanTxQueue, &message, 0) == pdTRUE) {
            do_send_can_message(message.id, message.buf, message.length);
        }
    }
}
}

void registerCANMsgHandler(unsigned int mask, void (*callback)(rmc_can_msg msg)) {
    msgHandlers.push_back({.mask = mask, .callback = callback});
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_) {
    /* Get RX message */
    if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        /* Reception Error */
        Error_Handler();
    }
    rmc_can_msg msg;
    msg.id = RxHeader.ExtId;
    msg.length = RxHeader.DLC;
    memcpy(&msg.buf, RxData, msg.length);
    for (auto it = msgHandlers.begin(); it < msgHandlers.end(); ++it) {
        CANMsgHandlerPair handlerPair = *it;
        if (msg.id & handlerPair.mask) {
            handlerPair.callback(msg);
        }
    }
}