#include "isr.h"
#include "state.h"

volatile uint32_t xLastKeyTick     = 0;
volatile uint32_t xLastDropTickISR = 0;

#define KEY_DEBOUNCE_MS 50
#define PHOTO_DEBOUNCE_MS 10

/**
  * @brief UART 空闲接收回调（使用 HAL UART Ex 的 ReceiveToIdle DMA）
  * @param huart: UART 句柄
  * @param Size: 实际接收到的数据长度
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART1) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (Size > UART_RX_BUF_SZ) {
      Size = UART_RX_BUF_SZ;
    }

    // 保存接收长度，由任务读取并处理缓冲区内容
    g_UartRxLen = Size;

    // 通知 CommTask 有数据到达
    vTaskNotifyGiveFromISR(CommTaskHandle, &xHigherPriorityTaskWoken);

    // 请求调度切换（如果需要）
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    // 重新启动 DMA 接收，准备下一次接收
    HAL_UARTEx_ReceiveToIdle_DMA(huart, g_UartRxBuf, UART_RX_BUF_SZ);
  }
}

/**
  * @brief GPIO 外部中断回调函数
  * @param GPIO_Pin: 触发中断的 GPIO 引脚编号
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  HAL_GPIO_TogglePin(light_GPIO_Port, light_Pin);

  if (GPIO_Pin == PhotoelectricSensor_Pin) {
    uint32_t now = HAL_GetTick();
    if (now - xLastDropTickISR >= PHOTO_DEBOUNCE_MS) {
      xLastDropTickISR = now;

      flow.interval_ms = now - flow.last_tick;
      flow.last_tick = now;

      flow.pulse_count++;
    }
  }
  
  // if (GPIO_Pin == PowerKey_Pin) {
  //   uint32_t now = HAL_GetTick();

  //   // 消抖时间过滤
  //   if (now - xLastKeyTick < KEY_DEBOUNCE_MS)
  //     return;

  //   xLastKeyTick = now;

  //   // 检测是否真的按下
  //   if (HAL_GPIO_ReadPin(PowerKey_GPIO_Port, PowerKey_Pin) == GPIO_PIN_RESET) {

  //     xSystemState = (xSystemState == IDLE) ? WORKING : IDLE;
  //   }
  // }
}
