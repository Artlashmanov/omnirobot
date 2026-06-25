#include "encoder_app.h"

#include <stdio.h>

/*
 * Encoder 1, rear-right test motor: JGY-370 DC6V150RPM.
 *
 * Real connector order was verified on the encoder PCB, left to right:
 *   white  = M1 motor
 *   yellow = VCC encoder
 *   blue   = A encoder
 *   green  = B encoder
 *   black  = GND encoder
 *   red    = M2 motor
 *
 * Current safe STM32 test wiring:
 *   yellow -> 3.3V
 *   black  -> GND
 *   blue   -> PA6 / D12 / TIM3_CH1
 *   green  -> PA7 / D11 / TIM3_CH2
 *
 * Observed behavior:
 *   robot forward gives a negative raw TIM3 delta on this motor.
 *   Keep raw_delta untouched for debugging, but expose signed_delta so
 *   "forward" is positive for the rear-right wheel.
 */
#define ENCODER_APP_SAMPLE_PERIOD_MS   200U
#define ENCODER_APP_REAR_RIGHT_SIGN    (-1)

static TIM_HandleTypeDef *s_encoder_tim = NULL;
static uint16_t s_prev_count = 0;
static uint32_t s_last_sample_ms = 0;

void EncoderApp_Init(TIM_HandleTypeDef *htim)
{
  s_encoder_tim = htim;
  s_prev_count = 0;
  s_last_sample_ms = 0;
}

HAL_StatusTypeDef EncoderApp_Start(void)
{
  HAL_StatusTypeDef status;

  if (s_encoder_tim == NULL)
  {
    return HAL_ERROR;
  }

  __HAL_TIM_SET_COUNTER(s_encoder_tim, 0);

  status = HAL_TIM_Encoder_Start(s_encoder_tim, TIM_CHANNEL_ALL);
  if (status != HAL_OK)
  {
    return status;
  }

  s_prev_count = (uint16_t)__HAL_TIM_GET_COUNTER(s_encoder_tim);
  s_last_sample_ms = HAL_GetTick();

  return HAL_OK;
}

uint8_t EncoderApp_Poll(uint32_t now_ms, EncoderApp_Sample_t *sample)
{
  uint16_t now_count;
  int16_t raw_delta;

  if ((s_encoder_tim == NULL) || (sample == NULL))
  {
    return 0U;
  }

  if ((uint32_t)(now_ms - s_last_sample_ms) < ENCODER_APP_SAMPLE_PERIOD_MS)
  {
    return 0U;
  }

  now_count = (uint16_t)__HAL_TIM_GET_COUNTER(s_encoder_tim);
  raw_delta = (int16_t)(now_count - s_prev_count);

  s_prev_count = now_count;
  s_last_sample_ms = now_ms;

  sample->raw_count = now_count;
  sample->raw_delta = raw_delta;
  sample->signed_delta = ((int32_t)raw_delta * ENCODER_APP_REAR_RIGHT_SIGN);

  return 1U;
}

void EncoderApp_PrintDebug(const EncoderApp_Sample_t *sample)
{
  if (sample == NULL)
  {
    return;
  }

  printf("enc1 raw=%u delta=%d signed_delta=%ld\r\n",
         (unsigned int)sample->raw_count,
         (int)sample->raw_delta,
         (long)sample->signed_delta);
}
