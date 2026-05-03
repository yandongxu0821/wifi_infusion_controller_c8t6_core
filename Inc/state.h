#ifndef __STATE_H
#define __STATE_H

#include "main.h"

/* 系统状态枚举 */
typedef enum {
  IDLE = 0,
  WORKING,
  STATE_MAX
} SystemState_t;

/* 报警状态枚举 */
typedef enum {
  ALARM_NONE = 0,
  ALARM_COMPLETE,
  ALARM_LOW,
  ALARM_HIGH,
  ALARM_MAX
} AlarmState_t;

extern const char* SystemStateStrings[];
extern const char* AlarmStateStrings[];

extern volatile uint16_t      xDropCount;
extern volatile uint16_t      xLastCmdTime;
extern volatile float         xCurrentSpeed;
extern volatile SystemState_t xSystemState;
extern volatile AlarmState_t  xAlarmState;
extern volatile AlarmState_t  xLastAlarmState;

const char* Get_State_String(SystemState_t state);
const char* Get_Alarm_String(AlarmState_t state);

#endif
