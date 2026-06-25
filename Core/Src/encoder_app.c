#include "encoder_app.h"

#include <stdio.h>

/*
 * JGY-370 encoder wiring, verified on the encoder PCB:
 *   white  = M1 motor
 *   yellow = VCC encoder, use STM32 3.3V
 *   blue   = encoder A
 *   green  = encoder B
 *   black  = encoder GND
 *   red    = M2 motor
 *
 * Current STM32 encoder mapping:
 *   FL: blue PA0 / TIM5_CH1, green PA1 / TIM5_CH2
 *   FR: blue PC6 / TIM8_CH1, green PC7 / TIM8_CH2
 *   RL: blue PB6 / TIM4_CH1, green PB7 / TIM4_CH2
 *   RR: blue PA6 / TIM3_CH1, green PA7 / TIM3_CH2
 *
 * Direction signs are calibration values. RR was measured: robot forward
 * gives a negative raw delta, so its signed_delta uses -1.
 * Other wheels stay +1 until we measure them on the real robot.
 */
#define ENCODER_APP_SAMPLE_PERIOD_MS 200U

#define ENCODER_APP_FL_SIGN  (1)
#define ENCODER_APP_FR_SIGN  (-1)
#define ENCODER_APP_RL_SIGN  (1)
#define ENCODER_APP_RR_SIGN  (-1)

typedef struct
{
  TIM_HandleTypeDef *htim;
  uint16_t prev_count;
  int8_t sign;
  const char *label;
} EncoderApp_Channel_t;

static EncoderApp_Channel_t s_channels[ENCODER_APP_COUNT] =
{
  [ENCODER_APP_FL] = { NULL, 0, ENCODER_APP_FL_SIGN, "fl" },
  [ENCODER_APP_FR] = { NULL, 0, ENCODER_APP_FR_SIGN, "fr" },
  [ENCODER_APP_RL] = { NULL, 0, ENCODER_APP_RL_SIGN, "rl" },
  [ENCODER_APP_RR] = { NULL, 0, ENCODER_APP_RR_SIGN, "rr" },
};

static uint32_t s_last_sample_ms = 0;

void EncoderApp_Init(TIM_HandleTypeDef *htim_fl,
                     TIM_HandleTypeDef *htim_fr,
                     TIM_HandleTypeDef *htim_rl,
                     TIM_HandleTypeDef *htim_rr)
{
  s_channels[ENCODER_APP_FL].htim = htim_fl;
  s_channels[ENCODER_APP_FR].htim = htim_fr;
  s_channels[ENCODER_APP_RL].htim = htim_rl;
  s_channels[ENCODER_APP_RR].htim = htim_rr;

  for (uint32_t i = 0; i < ENCODER_APP_COUNT; i++)
  {
    s_channels[i].prev_count = 0;
  }

  s_last_sample_ms = 0;
}

HAL_StatusTypeDef EncoderApp_Start(void)
{
  HAL_StatusTypeDef status;

  for (uint32_t i = 0; i < ENCODER_APP_COUNT; i++)
  {
    if (s_channels[i].htim == NULL)
    {
      return HAL_ERROR;
    }

    __HAL_TIM_SET_COUNTER(s_channels[i].htim, 0);

    status = HAL_TIM_Encoder_Start(s_channels[i].htim, TIM_CHANNEL_ALL);
    if (status != HAL_OK)
    {
      return status;
    }

    s_channels[i].prev_count = (uint16_t)__HAL_TIM_GET_COUNTER(s_channels[i].htim);
  }

  s_last_sample_ms = HAL_GetTick();

  return HAL_OK;
}

uint8_t EncoderApp_Poll(uint32_t now_ms, EncoderApp_Sample_t *sample)
{
  if (sample == NULL)
  {
    return 0U;
  }

  if ((uint32_t)(now_ms - s_last_sample_ms) < ENCODER_APP_SAMPLE_PERIOD_MS)
  {
    return 0U;
  }

  for (uint32_t i = 0; i < ENCODER_APP_COUNT; i++)
  {
    uint16_t now_count;
    int16_t raw_delta;

    if (s_channels[i].htim == NULL)
    {
      return 0U;
    }

    now_count = (uint16_t)__HAL_TIM_GET_COUNTER(s_channels[i].htim);
    raw_delta = (int16_t)(now_count - s_channels[i].prev_count);

    s_channels[i].prev_count = now_count;

    sample->wheel[i].raw_count = now_count;
    sample->wheel[i].raw_delta = raw_delta;
    sample->wheel[i].signed_delta = (int32_t)raw_delta * (int32_t)s_channels[i].sign;
  }

  s_last_sample_ms = now_ms;

  return 1U;
}

void EncoderApp_PrintDebug(const EncoderApp_Sample_t *sample)
{
  if (sample == NULL)
  {
    return;
  }

  printf("enc "
         "fl raw=%u d=%d sd=%ld | "
         "fr raw=%u d=%d sd=%ld | "
         "rl raw=%u d=%d sd=%ld | "
         "rr raw=%u d=%d sd=%ld\r\n",
         (unsigned int)sample->wheel[ENCODER_APP_FL].raw_count,
         (int)sample->wheel[ENCODER_APP_FL].raw_delta,
         (long)sample->wheel[ENCODER_APP_FL].signed_delta,
         (unsigned int)sample->wheel[ENCODER_APP_FR].raw_count,
         (int)sample->wheel[ENCODER_APP_FR].raw_delta,
         (long)sample->wheel[ENCODER_APP_FR].signed_delta,
         (unsigned int)sample->wheel[ENCODER_APP_RL].raw_count,
         (int)sample->wheel[ENCODER_APP_RL].raw_delta,
         (long)sample->wheel[ENCODER_APP_RL].signed_delta,
         (unsigned int)sample->wheel[ENCODER_APP_RR].raw_count,
         (int)sample->wheel[ENCODER_APP_RR].raw_delta,
         (long)sample->wheel[ENCODER_APP_RR].signed_delta);
}
