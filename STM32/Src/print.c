#include "print.h"
#include <stm32f3xx_hal.h>

extern UART_HandleTypeDef huart2;

void print(char* string) {
  uint8_t len = 0;
  char* str_ = string;
  while (*(str_++) != 0) {
    len++;
  }
  HAL_UART_Transmit(&huart2, string, len, 10);
}

void println(char* string) {
  print(string);
  print("\r\n");
}