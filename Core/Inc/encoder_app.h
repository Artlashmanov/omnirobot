#ifndef ENCODER_APP_H
#define ENCODER_APP_H

#include "main.h"
#include <stdint.h>

#define ENCODER_APP_COUNT 4U

typedef enum
{
  ENCODER_APP_FL = 0,
  ENCODER_APP_FR = 1,
  ENCODER_APP_RL = 2,
  ENCODER_APP_RR = 3
} EncoderApp_Wheel_t;

typedef struct
{
  uint16_t raw_count;
  int16_t raw_delta;
  int32_t signed_delta;
} EncoderApp_WheelSample_t;

typedef struct
{
  EncoderApp_WheelSample_t wheel[ENCODER_APP_COUNT];
} EncoderApp_Sample_t;

void EncoderApp_Init(TIM_HandleTypeDef *htim_fl,
                     TIM_HandleTypeDef *htim_fr,
                     TIM_HandleTypeDef *htim_rl,
                     TIM_HandleTypeDef *htim_rr);
HAL_StatusTypeDef EncoderApp_Start(void);
uint8_t EncoderApp_Poll(uint32_t now_ms, EncoderApp_Sample_t *sample);
void EncoderApp_PrintDebug(const EncoderApp_Sample_t *sample);

#endif /* ENCODER_APP_H */
