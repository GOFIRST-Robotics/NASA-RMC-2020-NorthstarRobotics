//
// Created by julia on 10/22/19.
//

#include <print.h>
#include <stm32f3xx.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "main.h"
#include "queue.h"
#include "task.h"

#include "can_manager.h"
#include "string.h"

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart2;
extern QueueHandle_t xCanRxQueue;
extern osMutexId canTxMutexHandle;
U32 CurTxMailbox;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
U8 RxData[8];
CANMsgHandlerPair msgHandlers[CAN_HANDLERS_SIZE];
int canHandlersCt = 0;

void do_send_can_message(U32 const id, U8 const* buf, S32 const length) {
  TxHeader.IDE = CAN_ID_EXT;
  TxHeader.ExtId = id;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = length;
  TxHeader.TransmitGlobalTime = DISABLE;
  xSemaphoreTake(canTxMutexHandle, portMAX_DELAY);
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) ==
         0) {  // Block until a mailbox is open
    vTaskDelay(1);
  }

  int retval = HAL_CAN_AddTxMessage(&hcan, &TxHeader, buf, &CurTxMailbox);
  xSemaphoreGive(canTxMutexHandle);
}

void registerCANMsgHandler(U32 const mask,
                           void (*const callback)(rmc_can_msg msg)) {
  msgHandlers[canHandlersCt++] =
      (CANMsgHandlerPair){.mask = mask, .callback = callback};
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan_) {
  /* Get RX message */
  if (HAL_CAN_GetRxMessage(hcan_, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
    /* Reception Error */
    Error_Handler();
  }
  rmc_can_msg msg;
  msg.id = RxHeader.ExtId;
  msg.length = RxHeader.DLC;
  memcpy(&msg.buf, RxData, msg.length);
  xQueueSendFromISR(xCanRxQueue, &msg, NULL);
  // todo worry about failure to post and/or higher priority task woken?
}

void canRxDispatchTask(void const* argument) {
  rmc_can_msg msg;
  while (1) {
    // Because we use the suspend feature (portMAX_DELAY), this should always
    // return true This should suspend until an item is available
    if (xQueueReceive(xCanRxQueue, &msg, portMAX_DELAY) == pdTRUE) {
      for (S32 i = 0; i < canHandlersCt; ++i) {
        CANMsgHandlerPair handlerPair = msgHandlers[i];
        if (msg.id & handlerPair.mask) {
          handlerPair.callback(msg);
        }
      }
    }
  }
}