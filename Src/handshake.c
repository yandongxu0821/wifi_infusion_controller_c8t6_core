#include "handshake.h"
#include "string.h"

HAL_StatusTypeDef UART_WaitForString(UART_HandleTypeDef *huart, const char *target, uint32_t timeout_ms) {
  char rx_buf[32];
  uint8_t rx_char;
  uint8_t idx = 0;
  uint32_t tick_start = HAL_GetTick();
  size_t target_len = strlen(target);

  memset(rx_buf, 0, sizeof(rx_buf));

  while ((HAL_GetTick() - tick_start) < timeout_ms) {
    if (HAL_UART_Receive(huart, &rx_char, 1, 10) == HAL_OK) {
      // 存入新字节并保证始终有 \0 结尾
      rx_buf[idx++] = (char)rx_char;
      rx_buf[idx] = '\0';

      if (strstr(rx_buf, target) != NULL) {
        return HAL_OK;
      }

      if (idx >= sizeof(rx_buf) - 3) {
        // 移动最后几个字符到开头，防止目标字符串被截断
        memmove(rx_buf, &rx_buf[idx - target_len], target_len);
        idx = target_len;
        memset(&rx_buf[idx], 0, sizeof(rx_buf) - idx);
      }
    }
  }
  return HAL_TIMEOUT;
}
void System_Handshake(UART_HandleTypeDef *huart) {
  // Pre. 清空可能的寄存器残留
  __HAL_UART_FLUSH_DRREGISTER(huart);

  // 1. 等待收到 "SYN\n"
  while (UART_WaitForString(huart, "SYN\r\n", HAL_MAX_DELAY) != HAL_OK);

  // 2. 回复 "ACK,SYN\n"
  const char *resp = "ACK,SYN\n";
  HAL_UART_Transmit(huart, (uint8_t*)resp, strlen(resp), 100);

  // 3. 再次接收到 "ACK\n" 后退出
  while (UART_WaitForString(huart, "ACK\r\n", HAL_MAX_DELAY) != HAL_OK);

  // After. 握手成功，可以清空一下残留的寄存器
  __HAL_UART_FLUSH_DRREGISTER(huart);
}
