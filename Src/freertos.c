/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ssd1315.h"
#include "bh1750.h"
#include "state.h"
#include "handshake.h"
#include "stdio.h"
#include "usart.h"
#include "isr.h"
#include "string.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define REPORT_INTERVAL 5000
#define UART_TIMEOUT 8000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float   xLuxValue          = 0;             // 当前光照强度
float   xFlowRateLimits[2] = {1.00, 8.00};  // 滴速上下限，单位滴/秒
uint8_t xBH1750Present     = 0;             // 标志位，表示 BH1750 是否存在
uint8_t xShowStatus        = 0;             // 显示切换标志，0显示状态，1显示光照
uint8_t xShowCounts        = 0;             // 显示切换计数
static char xUartTxBuffer[64];              // UART 发送缓冲区
/* USER CODE END Variables */
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FlowDetectTask */
osThreadId_t FlowDetectTaskHandle;
const osThreadAttr_t FlowDetectTask_attributes = {
  .name = "FlowDetectTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CommTask */
osThreadId_t CommTaskHandle;
const osThreadAttr_t CommTask_attributes = {
  .name = "CommTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

// static int parse_uint(const char *s, int *value);
static int parse_float(const char *s, float *value);
void SendDataToESP(char *data);
void ParseCommand(char *command);
void SendHeartbeat(void);
void SendStatus(void);

/* USER CODE END FunctionPrototypes */

void StartDisplayTask(void *argument);
void StartFlowDetectTask(void *argument);
void StartControlTask(void *argument);
void StartCommTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* 初始化 I2C 互斥，保护总线访问 */
  I2C_MutexInit();

  if (I2C_Check_Device(&hi2c1, BH1750_ADDR) == HAL_OK) {
    BH1750_Init();
    BH1750_Start(0x10);
    xBH1750Present = 1;
  } else {
    xBH1750Present = 0;
  }
  SSD1315_Init();

  SSD1315_ShowString(0, 0, "System Init");
  SSD1315_ShowString(0, 2, "WiFi Connecting");
  SSD1315_ShowString(0, 4, "......");
  SSD1315_Update();
	
  System_Handshake(&huart1);
	
  SSD1315_Clear();
  SSD1315_ShowString(0, 0, "System Init");
  SSD1315_ShowString(0, 2, "WiFi Connected");
  SSD1315_Update();

	/* 启动 UART DMA + IDLE 接收（在握手完成后启动，避免与阻塞接收冲突） */
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_UartRxBuf, UART_RX_BUF_SZ) != HAL_OK) {
    Error_Handler();
  }

  HAL_Delay(1000);

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &DisplayTask_attributes);

  /* creation of FlowDetectTask */
  FlowDetectTaskHandle = osThreadNew(StartFlowDetectTask, NULL, &FlowDetectTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of CommTask */
  CommTaskHandle = osThreadNew(StartCommTask, NULL, &CommTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
  /* Infinite loop */
  for(;;) {
    SSD1315_Clear();
    char buffer[20];

    snprintf((char*)buffer, sizeof(buffer), "State: %s", Get_State_String(xSystemState));
    SSD1315_ShowString(0, 0, (char*)buffer);

    snprintf((char*)buffer, sizeof(buffer), "Alarm: %s", Get_Alarm_String(xAlarmState));
    SSD1315_ShowString(0, 2, (char*)buffer);

    snprintf((char*)buffer, sizeof(buffer), "Speed: %.2f", xCurrentSpeed);
    SSD1315_ShowString(0, 4, (char*)buffer);

    if (xBH1750Present) {
      uint16_t xLuxRawValue = BH1750_Read();
      xLuxValue = xLuxRawValue / 1.2;
      if (xShowStatus) {
        snprintf((char*)buffer, sizeof(buffer), "Light: %.2f", xLuxValue);
        SSD1315_ShowString(0, 6, (char*)buffer);
      } else {
        snprintf((char*)buffer, sizeof(buffer), "LO=%.2f HI=%.2f", xFlowRateLimits[0], xFlowRateLimits[1]);
        SSD1315_ShowString(0, 6, (char*)buffer);
      }
    } else {
      snprintf((char*)buffer, sizeof(buffer), "LO=%.2f HI=%.2f", xFlowRateLimits[0], xFlowRateLimits[1]);
      SSD1315_ShowString(0, 6, (char*)buffer);
    }

    SSD1315_Update();
    if (xShowCounts++ >= 10) { // 每 10 次刷新切换一次显示
      xShowCounts = 0;
      xShowStatus = !xShowStatus;
    }
    osDelay(500);
  }
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartFlowDetectTask */
/**
* @brief Function implementing the FlowDetectTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFlowDetectTask */
void StartFlowDetectTask(void *argument)
{
  /* USER CODE BEGIN StartFlowDetectTask */
  /* 上一次滴落时间（任务侧） */
  uint32_t last_drop_tick = HAL_GetTick();  // 使用 HAL 获取时间戳

  /* EMA 平滑后的滴速（以整型表示） */
  uint16_t speed_ema = 0;  // 滴速的EMA（使用整数表示，单位为滴/100ms）

  /* EMA 系数（简单线性加权） */
  const uint8_t alpha = 30;  // 这里的30是用于控制EMA平滑的比例，实际可以调节

  /* 最大滴速限制（防止异常脉冲，单位：滴/100ms） */
  const uint8_t MAX_SPEED = 80;  // 对应8滴/秒，即80滴/100ms

  /* 最小有效间隔（ms）防抖，单位：滴/100ms*/
  const uint8_t MIN_INTERVAL = 50;  // 对应20滴/秒，即50ms之间

  /* 基础超时（ms） */
  const uint32_t BASE_TIMEOUT = 3000;

  /* Infinite loop */
  for(;;)
  {
    osDelay(50);

    /*====================================================
      IDLE 状态处理
    ====================================================*/
    if (xSystemState == IDLE)
    {
      /* 清空滴数 */
      taskENTER_CRITICAL();
      xDropCount = 0;
      taskEXIT_CRITICAL();

      /* EMA 滴速清零 */
      speed_ema = 0;

      /* 当前滴速清零 */
      xCurrentSpeed = 0;

      /* 报警状态清空 */
      xAlarmState = ALARM_NONE;

      last_drop_tick = HAL_GetTick();

      continue;
    }

    /*====================================================
      检测是否有新液滴（通过 ISR 时间戳变化）
    ====================================================*/
    uint32_t current_isr_tick;

    taskENTER_CRITICAL();
    current_isr_tick = xLastDropTickISR;
    taskEXIT_CRITICAL();

    /* 有新滴（时间戳更新） */
    if (current_isr_tick != last_drop_tick)
    {
      uint32_t delta_tick = current_isr_tick - last_drop_tick;

      last_drop_tick = current_isr_tick;

      /* 转换为滴速（单位：滴/100ms） */
      if (delta_tick > MIN_INTERVAL)
      {
        uint16_t inst_speed = 1000 / delta_tick;  // 计算滴速（单位：滴/100ms）

        /* 限幅：防止异常值 */
        if (inst_speed <= MAX_SPEED)
        {
          /* EMA 滴速更新（整数加权平均） */
          speed_ema = (speed_ema * (100 - alpha) + inst_speed * alpha) / 100;

          /* 更新滴速 */
          xCurrentSpeed = speed_ema;
        }
      }
    }

    /*====================================================
      自适应超时判断
    ====================================================*/
    uint32_t now = HAL_GetTick();

    uint32_t dynamic_timeout;

    if (xCurrentSpeed > 0)
    {
      /* 基于当前滴速估算周期 */
      uint32_t period = 1000 / xCurrentSpeed;  // 单位：ms

      dynamic_timeout = period * 3;  // 转换为超时，单位：ms
    }
    else
    {
      dynamic_timeout = BASE_TIMEOUT;
    }

    if (dynamic_timeout < BASE_TIMEOUT)
      dynamic_timeout = BASE_TIMEOUT;

    /* 超时 → COMPLETE */
    if ((now - last_drop_tick) >= dynamic_timeout)
    {
      xAlarmState = ALARM_COMPLETE;
      xCurrentSpeed = 0;
    }
    else
    {
      /*====================================================
        报警判断
      ====================================================*/

      if (xCurrentSpeed > xFlowRateLimits[1])
      {
        xAlarmState = ALARM_HIGH;
      }
      else if (xCurrentSpeed < xFlowRateLimits[0] && xCurrentSpeed > 0)
      {
        xAlarmState = ALARM_LOW;
      }
      else
      {
        xAlarmState = ALARM_NONE;
      }
    }
  }
  /* USER CODE END StartFlowDetectTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  // const TickType_t delay_time = pdMS_TO_TICKS(100);

  /* Infinite loop */
  for(;;) {
    osDelay(100);
    if (xSystemState == WORKING) {
      switch(xAlarmState) {
        case ALARM_COMPLETE:
          HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
          break;

        case ALARM_HIGH:
          HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
          break;

        case ALARM_LOW:
          HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
          break;

        case ALARM_NONE:
        default:
          HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
          break;
      }
    } else {  // xSystemState == IDLE
      HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
    }
  }
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
  * @brief  Function implementing the CommTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)
{
  /* USER CODE BEGIN StartCommTask */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(REPORT_INTERVAL);
  xLastWakeTime = xTaskGetTickCount();

  TickType_t xLastCmdTime = 0;  // 用于检查超时
  const TickType_t timeout = pdMS_TO_TICKS(UART_TIMEOUT);  // 超时 UART_TIMEOUT 秒

  /* Infinite loop */
  for(;;)
  {
    char xUartReceivedData[UART_RX_BUF_SZ + 1];

    // 等待 UART 事件（最多 100ms），由 HAL_UARTEx_RxEventCallback 通知
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) > 0) {
      uint16_t len = g_UartRxLen;

      if (len > 0) {
        if (len >= UART_RX_BUF_SZ) len = UART_RX_BUF_SZ;
        memcpy(xUartReceivedData, g_UartRxBuf, len);
        xUartReceivedData[len] = '\0';

        // 清除长度，防止重复处理
        g_UartRxLen = 0;

        ParseCommand(xUartReceivedData);
        xLastCmdTime = xTaskGetTickCount();
      }
    }

    // 检查是否超时
    if (xTaskGetTickCount() - xLastCmdTime > timeout) {
      // 超过 UART_TIMEOUT 秒，拉低 EspRst 端口重启 ESP8266
      HAL_GPIO_WritePin(EspRst_GPIO_Port, EspRst_Pin, GPIO_PIN_RESET); // 拉低复位端口
      osDelay(100);  // 延迟足够时间让 ESP 重启
      HAL_GPIO_WritePin(EspRst_GPIO_Port, EspRst_Pin, GPIO_PIN_SET); // 释放复位端口
      xLastCmdTime = xTaskGetTickCount(); // 重置计时
    }

    // alarm变化立即发送一次
    if (xAlarmState != xLastAlarmState) {
      SendStatus();
      xLastAlarmState = xAlarmState;
      xLastWakeTime = xTaskGetTickCount();
    }

    if (xTaskGetTickCount() - xLastWakeTime >= xFrequency) {
      SendStatus();
      xLastWakeTime = xTaskGetTickCount();
    }

    // osDelay(10);
  }
  /* USER CODE END StartCommTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/** replace from atoi(), but more fast **
static int parse_uint(const char *s, int *value) {
  int num = 0;
  if (!s || *s < '0' || *s > '9') return 0;
  while (*s >= '0' && *s <= '9') {
    num = num * 10 + (*s++ - '0');
  }
  if (*s != '\0' && *s != '\r' && *s != '\n') return 0;
  *value = num;
  return 1;
}
*/

static int parse_float(const char *s, float *value) {
  if (!s || !value) return 0;

  int int_part = 0;
  int frac_part = 0;
  int frac_div = 1;
  int has_digit = 0;

  // --- 整数部分 ---
  while (*s >= '0' && *s <= '9') {
    has_digit = 1;
    int_part = int_part * 10 + (*s - '0');
    s++;
  }

  // --- 小数部分 ---
  if (*s == '.') {
    s++;
    if (*s < '0' || *s > '9') return 0;  // "." 后必须有数字

    while (*s >= '0' && *s <= '9') {
      frac_part = frac_part * 10 + (*s - '0');
      frac_div *= 10;
      s++;
    }
  }

  // --- 至少要有数字 ---
  if (!has_digit) return 0;

  // --- 结尾校验 ---
  if (*s != '\0' && *s != '\r' && *s != '\n') return 0;

  *value = (float)int_part + (float)frac_part / frac_div;
  return 1;
}

void SendDataToESP(char *data) {
  while (huart1.gState != HAL_UART_STATE_READY); // 确保准备好了
    
  strncpy(xUartTxBuffer, data, sizeof(xUartTxBuffer));
  HAL_UART_Transmit_DMA(&huart1, (uint8_t *)xUartTxBuffer, strlen(xUartTxBuffer));
}

void ParseCommand(char *command) {
  if (!command || !command[0]) return;

  char first = command[0];

  /* ---------------- 心跳/上限 ---------------- */
  if (first == 'H') {   
    // HB 心跳
    if (command[1] == 'B' && (command[2] == '\0' || command[2] == '\r' || command[2] == '\n')) {
      SendDataToESP("ACK\n");
      goto clear;
    }

    // HI,<value>  上限
    if (command[1] == 'I' && command[2] == ',') {
      float val;
      if (!parse_float(command + 3, &val)) {
        SendDataToESP("ERROR\n");
        goto clear;
      }

      if (val < 0 || val > 50) {
        SendDataToESP("ERROR\n");
        goto clear;
      }
      xFlowRateLimits[1] = val;

      SendDataToESP("ACK\n");
      goto clear;
    }
  }

  /* ---------------- 下限 ---------------- */
  else if (first == 'L') {  
    // LO,<value>
    if (command[1] == 'O' && command[2] == ',') {
      float val;
      if (!parse_float(command + 3, &val)) {
        SendDataToESP("ERROR\n");
        goto clear;
      }

      if (val < 0 || val > 50) {
        SendDataToESP("ERROR\n");
        goto clear;
      }
      xFlowRateLimits[0] = val;

      SendDataToESP("ACK\n");
      goto clear;
    }
  }

  /* ---------------- CMD控制 ---------------- */
  else if (first == 'C') {   
    if (strncmp(command, "CMD,", 4) != 0) {
      goto clear;
    }

    char *cmd = command + 4;

    if (strncmp(cmd, "START", 5) == 0) {
      char n = cmd[5];
      if (n == '\0' || n == '\r' || n == '\n') {
        xSystemState = WORKING;
        SendDataToESP("ACK\n");
        goto clear;
      }
    }
    else if (strncmp(cmd, "STOP", 4) == 0) {
      char n = cmd[4];
      if (n == '\0' || n == '\r' || n == '\n') {
        xSystemState = IDLE;
        SendDataToESP("ACK\n");
        goto clear;
      }
    }
    else {
      SendDataToESP("ERROR\n");
      goto clear;
    }
  }

  /* ---------------- 重新握手 ---------------- */
  else if (first == 'S') {
    // SYN 握手
    if (strcmp(command, "SYN") == 0) {
      SendDataToESP("ACK,SYN\n");
      goto clear;
    }
  }

clear:
  command[0] = '\0';
}

void SendStatus(void) {
  char status_message[20];

  if (xSystemState == IDLE) {   // Only report state when idle, to reduce unnecessary updates
    snprintf(status_message, sizeof(status_message), "STATE,%s\n", Get_State_String(xSystemState) );
    SendDataToESP(status_message);
    
    if (xBH1750Present) {
      snprintf(status_message, sizeof(status_message), "LIGHT,%.2f\n", xLuxValue );
      SendDataToESP(status_message);
    }
    return;
  }
  
  snprintf(status_message, sizeof(status_message), "STATE,%s\n", Get_State_String(xSystemState) );
  SendDataToESP(status_message);

  snprintf(status_message, sizeof(status_message), "SPEED,%.2f\n", xCurrentSpeed );
  SendDataToESP(status_message);

  snprintf(status_message, sizeof(status_message), "ALARM,%s\n", Get_Alarm_String(xAlarmState) );
  SendDataToESP(status_message);
  
  if (xBH1750Present) {
    snprintf(status_message, sizeof(status_message), "LIGHT,%.2f\n", xLuxValue );
    SendDataToESP(status_message);
  }
}

/* USER CODE END Application */

