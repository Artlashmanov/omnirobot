#ifndef POWER_APP_H
#define POWER_APP_H

#include "main.h"
#include <stdint.h>

#define POWER_APP_FLAG_INA228_PRESENT     0x01U
#define POWER_APP_FLAG_MEASUREMENT_VALID  0x02U
#define POWER_APP_FLAG_OVER_CURRENT       0x04U
#define POWER_APP_FLAG_OVER_VOLTAGE       0x08U
#define POWER_APP_FLAG_UNDER_VOLTAGE      0x10U
#define POWER_APP_FLAG_SENSOR_ERROR       0x20U
#define POWER_APP_FLAG_POWER_SATURATED    0x40U

typedef struct
{
  uint16_t bus_voltage_mv;
  int16_t current_ma;
  uint8_t power_w;
  uint8_t flags;
  uint32_t updated_ms;
  uint32_t error_count;
} PowerApp_State_t;

void PowerApp_Init(I2C_HandleTypeDef *hi2c);
void PowerApp_Poll(uint32_t now_ms);
const PowerApp_State_t *PowerApp_GetState(void);

#endif /* POWER_APP_H */