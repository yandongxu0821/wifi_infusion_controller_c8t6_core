#include "state.h"

const char* SystemStateStrings[] = {
  "IDLE",
  "WORKING"
};

const char* AlarmStateStrings[] = {
  "NONE",
  "COMPLETE",
  "LOW",
  "HIGH"
};

volatile uint16_t      xLastCmdTime    = 0;            // 上次收信时间
volatile float         xCurrentSpeed;                  // 当前流速
volatile SystemState_t xSystemState    = IDLE;         // 系统状态
volatile AlarmState_t  xAlarmState     = ALARM_NONE;   // 报警状态
volatile AlarmState_t  xLastAlarmState = ALARM_NONE;   // 上次报警状态

const char* Get_State_String(SystemState_t xState) {
  if (xState < STATE_MAX) {
    return SystemStateStrings[xState];
  }
  return "UNKNOWN";
}

const char* Get_Alarm_String(AlarmState_t xState) {
  if (xState < ALARM_MAX) {
    return AlarmStateStrings[xState];
  }
  return "UNKNOWN";
}
