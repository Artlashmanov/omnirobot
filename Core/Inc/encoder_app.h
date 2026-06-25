#ifndef ENCODER_APP_H
#define ENCODER_APP_H

#include "main.h"
#include <stdint.h>

#define ENCODER_APP_COUNT 4U

#define ENCODER_APP_WHEEL_FLAG_PRESENT 0x01U
#define ENCODER_APP_WHEEL_FLAG_UPDATED 0x02U
#define ENCODER_APP_WHEEL_FLAG_MOVING  0x04U

typedef enum
{
  ENCODER_APP_FL = 0,
  ENCODER_APP_FR = 1,
  ENCODER_APP_RL = 2,
  ENCODER_APP_RR = 3
} EncoderApp_Wheel_t;

typedef struct
{
  uint8_t index;
  uint8_t flags;
  uint16_t raw_count;
  int16_t delta_ticks;
  int32_t speed_ticks_per_sec;
} EncoderApp_WheelState_t;

typedef struct
{
  uint32_t updated_ms;
  uint32_t sample_period_ms;
  EncoderApp_WheelState_t wheel[ENCODER_APP_COUNT];
} EncoderApp_State_t;

void EncoderApp_Init(TIM_HandleTypeDef *htim_fl,
                     TIM_HandleTypeDef *htim_fr,
                     TIM_HandleTypeDef *htim_rl,
                     TIM_HandleTypeDef *htim_rr);
HAL_StatusTypeDef EncoderApp_Start(void);
uint8_t EncoderApp_Poll(uint32_t now_ms);
const EncoderApp_State_t *EncoderApp_GetState(void);
const EncoderApp_WheelState_t *EncoderApp_GetWheelState(uint8_t index);
uint8_t EncoderApp_IsReady(void);
void EncoderApp_SetDebugEnabled(uint8_t enabled);
uint8_t EncoderApp_GetDebugEnabled(void);
void EncoderApp_ToggleDebug(void);
void EncoderApp_PrintDebug(void);

#endif /* ENCODER_APP_H */
