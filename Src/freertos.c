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
float    xLuxValue          = 0;          // 当前光照强度
uint16_t xFlowRateLimits[2] = {60, 480};  // 滴速上下限，单位滴/分钟
uint8_t  xBH1750Present     = 0;          // 标志位，表示 BH1750 是否存在
uint8_t  xShowStatus        = 0;          // 显示切换标志，0显示状态，1显示光照
uint8_t  xShowCounts        = 0;          // 显示切换计数
static char xUartTxBuffer[64];            // UART 发送缓冲区
volatile Flow_t flow        = {0};        // 滴速计算结构体
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
static uint16_t Calc_Period_Speed(uint32_t dt_ms);
static uint16_t EMA_Filter(uint16_t prev, uint16_t now);
static void Update_Count_Speed(void);
static uint16_t Fuse_Speed(uint16_t sp_period, uint16_t sp_count, uint32_t dt);
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

	// 启动 UART DMA 空闲中断接收
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
  char buffer[20];
  uint16_t xLuxRawValue = 0;
  /* Infinite loop */
  for(;;) {
    SSD1315_Clear();

    // Line 1: State: xxxx
    snprintf((char*)buffer, sizeof(buffer), "State: %s", Get_State_String(xSystemState));
    SSD1315_ShowString(0, 0, (char*)buffer);

    // Line 2: Alarm: xxxx
    snprintf((char*)buffer, sizeof(buffer), "Alarm: %s", Get_Alarm_String(xAlarmState));
    SSD1315_ShowString(0, 2, (char*)buffer);

    // Line 3: Speed: xx.xx
    snprintf((char*)buffer, sizeof(buffer), "Speed: %d", flow.speed_cpm);
    SSD1315_ShowString(0, 4, (char*)buffer);

    if (xBH1750Present) {
      xLuxRawValue = BH1750_Read();
      xLuxValue = xLuxRawValue / 1.2;
      if (xShowStatus) {
        // Line 4: Light: xx.xx
        snprintf((char*)buffer, sizeof(buffer), "Light: %.2f", xLuxValue);
        SSD1315_ShowString(0, 6, (char*)buffer);
      } else {
        // Line 4: LO=xxx HI=xxx
        snprintf((char*)buffer, sizeof(buffer), "LO=%d HI=%d", xFlowRateLimits[0], xFlowRateLimits[1]);
        SSD1315_ShowString(0, 6, (char*)buffer);
      }
    } else {
      // Line 4: LO=xxx HI=xxx
      snprintf((char*)buffer, sizeof(buffer), "LO=%d HI=%d", xFlowRateLimits[0], xFlowRateLimits[1]);
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
  uint16_t period_raw = 0;

  /* 无滴超时时间 (ms)，超过则认为输液完成 */
  const uint32_t timeout = 5000;
  /* Infinite loop */
  for(;;)
  {
    // 1. 更新计数法
    Update_Count_Speed();

    // 2. 周期法计算
    if (flow.interval_ms > 0 && flow.interval_ms < 3000) {
      period_raw = Calc_Period_Speed(flow.interval_ms);

      flow.speed_period = EMA_Filter(flow.speed_period, period_raw);
    }

    // 3. 超时处理（你已有逻辑）
    if (HAL_GetTick() - flow.last_tick > 3000) {
      flow.speed_cpm = 0;
    } else {
      // 4. 融合
      flow.speed_cpm = Fuse_Speed(
        flow.speed_period,
        flow.speed_count,
        flow.interval_ms
      );
    }

    osDelay(100);

    /*====================================================
      超时判断

      如果超过 timeout 没有检测到新的液滴
      认为输液完成或阻塞 → ALARM_COMPLETE
    ====================================================*/
    if (xTaskGetTickCount() - flow.last_tick >= timeout) {
      xAlarmState = ALARM_COMPLETE;
    } else {
      /*------------------------------------------
        根据滴速判断报警状态
      ------------------------------------------*/

      /* 滴速过快 */
      if (flow.speed_cpm > xFlowRateLimits[1]) {
        xAlarmState = ALARM_HIGH;
      
      /* 滴速过慢 (但不是0) */
      } else if (flow.speed_cpm < xFlowRateLimits[0] && flow.speed_cpm > 0) {
        xAlarmState = ALARM_LOW;
      
      /* 正常 */
      } else {
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

  static GPIO_PinState xPowerLastState = GPIO_PIN_SET; // 上拉默认未按下

  /* Infinite loop */
  for(;;) {
    osDelay(50);

    GPIO_PinState xPowerCurrentState = HAL_GPIO_ReadPin(PowerKey_GPIO_Port, PowerKey_Pin);
    static uint32_t lastTick = 0;

    if (xPowerLastState == GPIO_PIN_SET && xPowerCurrentState == GPIO_PIN_RESET) {
      if ((xTaskGetTickCount() - lastTick) > pdMS_TO_TICKS(50)) {
        lastTick = xTaskGetTickCount();

        xSystemState = (xSystemState == IDLE) ? WORKING : IDLE;
        HAL_GPIO_TogglePin(light_GPIO_Port, light_Pin);
      }
    }

    xPowerLastState = xPowerCurrentState;

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

static uint16_t Calc_Period_Speed(uint32_t dt_ms) {
  if (dt_ms == 0) return 0;

  // 次/分钟 = 60000 / dt(ms)
  return (uint16_t)(60000UL / dt_ms);
}

static uint16_t EMA_Filter(uint16_t prev, uint16_t now) {
  // alpha ≈ 0.25
  return (prev * 3 + now) / 4;
}

static void Update_Count_Speed(void) {
  static uint32_t last_time = 0;
  uint32_t now = HAL_GetTick();

  if (now - last_time >= 1000) {
    // 次/分钟 = count * 60
    flow.speed_count = flow.pulse_count * 60;

    flow.pulse_count = 0;
    last_time = now;
  }
}

static uint16_t Fuse_Speed(uint16_t sp_period, uint16_t sp_count, uint32_t dt) {
  // 阈值：500ms（≈2Hz）
  if (dt >= 500) {
    // 低速：完全用周期法
    return sp_period;

  } else /*if (dt <= 100)*/ {
    // 高速：完全用计数法
    return sp_count;

  } /* else {
    // 中间区：线性融合
    uint32_t w = (dt - 100);   // 0~400
    // period权重：w，count权重：400-w

    return (sp_period * w + sp_count * (400 - w)) / 400;
  }*/
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

  if (xSystemState == IDLE) {
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

