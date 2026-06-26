#ifndef CAN_APP_H
#define CAN_APP_H

#include "main.h"

void CAN_App_Init(void);
void CAN_App_TelemetryTask(uint32_t now_ms);
void CAN_App_SetDebugEnabled(uint8_t enabled);
uint8_t CAN_App_GetDebugEnabled(void);
void CAN_App_ToggleDebug(void);

#endif
