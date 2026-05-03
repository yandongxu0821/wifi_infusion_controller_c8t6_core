#ifndef __HANDSHAKE_H
#define __HANDSHAKE_H

#include "main.h"
#include "isr.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef UART_WaitForString(UART_HandleTypeDef *huart, const char *target, uint32_t timeout_ms);
void System_Handshake(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
