#ifndef CAN_APP_H
#define CAN_APP_H

#include "main.h"

void CAN_App_Init(void);
void CAN_App_TelemetryTask(uint32_t now_ms);

#endif
