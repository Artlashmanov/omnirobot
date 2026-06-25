#ifndef ENCODER_APP_H
#define ENCODER_APP_H

#include "main.h"
#include <stdint.h>

typedef struct
{
  uint16_t raw_count;
  int16_t raw_delta;
  int32_t signed_delta;
} EncoderApp_Sample_t;

void EncoderApp_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef EncoderApp_Start(void);
uint8_t EncoderApp_Poll(uint32_t now_ms, EncoderApp_Sample_t *sample);
void EncoderApp_PrintDebug(const EncoderApp_Sample_t *sample);

#endif /* ENCODER_APP_H */
