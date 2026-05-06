#ifndef __ISR_H
#define __ISR_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

// 声明在 freertos.c 中定义的任务句柄
extern osThreadId_t CommTaskHandle;

extern volatile uint32_t xLastDropTickISR;

extern volatile Flow_t flow;

// UART DMA 接收 buffer 大小（可根据需要调整）
#define UART_RX_BUF_SZ 127

// 全局 DMA 接收区与实际接收长度（在回调中设置）
extern uint8_t g_UartRxBuf[UART_RX_BUF_SZ];
extern volatile uint16_t g_UartRxLen;

// HAL 扩展的空闲接收回调（在 stm32 HAL 的 UART Ex 中被调用）
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

// void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif
