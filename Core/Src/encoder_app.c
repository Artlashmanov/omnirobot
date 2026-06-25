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
 *   wheel 0 / FL: blue PA0 / TIM5_CH1, green PA1 / TIM5_CH2
 *   wheel 1 / FR: blue PC6 / TIM8_CH1, green PC7 / TIM8_CH2
 *   wheel 2 / RL: blue PB6 / TIM4_CH1, green PB7 / TIM4_CH2
 *   wheel 3 / RR: blue PA6 / TIM3_CH1, green PA7 / TIM3_CH2
 *
 * Direction signs are calibrated so robot forward is positive for every wheel.
 */
#define ENCODER_APP_SAMPLE_PERIOD_MS 100U

#define ENCODER_APP_FL_SIGN  (1)
#define ENCODER_APP_FR_SIGN  (-1)
#define ENCODER_APP_RL_SIGN  (1)
#define ENCODER_APP_RR_SIGN  (-1)

typedef struct
{
  TIM_HandleTypeDef *htim;
  uint16_t prev_count;
  int8_t sign;
} EncoderApp_Channel_t;

static EncoderApp_Channel_t s_channels[ENCODER_APP_COUNT] =
{
  [ENCODER_APP_FL] = { NULL, 0, ENCODER_APP_FL_SIGN },
  [ENCODER_APP_FR] = { NULL, 0, ENCODER_APP_FR_SIGN },
  [ENCODER_APP_RL] = { NULL, 0, ENCODER_APP_RL_SIGN },
  [ENCODER_APP_RR] = { NULL, 0, ENCODER_APP_RR_SIGN },
};

static EncoderApp_State_t s_state = {0};
static uint32_t s_last_sample_ms = 0;
static uint8_t s_ready = 0U;
static uint8_t s_debug_enabled = 0U;

static int32_t scale_delta_to_ticks_per_sec(int32_t delta_ticks, uint32_t dt_ms)
{
  if (dt_ms == 0U)
  {
    return 0;
  }

  return (int32_t)((delta_ticks * 1000L) / (int32_t)dt_ms);
}

void EncoderApp_Init(TIM_HandleTypeDef *htim_fl,
                     TIM_HandleTypeDef *htim_fr,
                     TIM_HandleTypeDef *htim_rl,
                     TIM_HandleTypeDef *htim_rr)
{
  s_channels[ENCODER_APP_FL].htim = htim_fl;
  s_channels[ENCODER_APP_FR].htim = htim_fr;
  s_channels[ENCODER_APP_RL].htim = htim_rl;
  s_channels[ENCODER_APP_RR].htim = htim_rr;

  s_state.updated_ms = 0;
  s_state.sample_period_ms = ENCODER_APP_SAMPLE_PERIOD_MS;

  for (uint32_t i = 0; i < ENCODER_APP_COUNT; i++)
  {
    s_channels[i].prev_count = 0;
    s_state.wheel[i].index = (uint8_t)i;
    s_state.wheel[i].flags = 0U;
    s_state.wheel[i].raw_count = 0U;
    s_state.wheel[i].delta_ticks = 0;
    s_state.wheel[i].speed_ticks_per_sec = 0;
  }

  s_last_sample_ms = 0;
  s_ready = 0U;
  s_debug_enabled = 0U;
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
    s_state.wheel[i].raw_count = s_channels[i].prev_count;
    s_state.wheel[i].flags = ENCODER_APP_WHEEL_FLAG_PRESENT;
  }

  s_last_sample_ms = HAL_GetTick();
  s_state.updated_ms = s_last_sample_ms;
  s_ready = 1U;

  return HAL_OK;
}

uint8_t EncoderApp_Poll(uint32_t now_ms)
{
  uint32_t dt_ms;

  if (s_ready == 0U)
  {
    return 0U;
  }

  dt_ms = now_ms - s_last_sample_ms;
  if (dt_ms < ENCODER_APP_SAMPLE_PERIOD_MS)
  {
    return 0U;
  }

  for (uint32_t i = 0; i < ENCODER_APP_COUNT; i++)
  {
    uint16_t now_count;
    int16_t raw_delta;
    int32_t signed_delta;
    uint8_t flags;

    if (s_channels[i].htim == NULL)
    {
      return 0U;
    }

    now_count = (uint16_t)__HAL_TIM_GET_COUNTER(s_channels[i].htim);
    raw_delta = (int16_t)(now_count - s_channels[i].prev_count);
    signed_delta = (int32_t)raw_delta * (int32_t)s_channels[i].sign;

    s_channels[i].prev_count = now_count;

    flags = ENCODER_APP_WHEEL_FLAG_PRESENT | ENCODER_APP_WHEEL_FLAG_UPDATED;
    if (signed_delta != 0)
    {
      flags |= ENCODER_APP_WHEEL_FLAG_MOVING;
    }

    s_state.wheel[i].index = (uint8_t)i;
    s_state.wheel[i].flags = flags;
    s_state.wheel[i].raw_count = now_count;
    s_state.wheel[i].delta_ticks = (int16_t)signed_delta;
    s_state.wheel[i].speed_ticks_per_sec = scale_delta_to_ticks_per_sec(signed_delta, dt_ms);
  }

  s_last_sample_ms = now_ms;
  s_state.updated_ms = now_ms;
  s_state.sample_period_ms = dt_ms;

  return 1U;
}

const EncoderApp_State_t *EncoderApp_GetState(void)
{
  return &s_state;
}

const EncoderApp_WheelState_t *EncoderApp_GetWheelState(uint8_t index)
{
  if (index >= ENCODER_APP_COUNT)
  {
    return NULL;
  }

  return &s_state.wheel[index];
}

uint8_t EncoderApp_IsReady(void)
{
  return s_ready;
}

void EncoderApp_SetDebugEnabled(uint8_t enabled)
{
  s_debug_enabled = (enabled != 0U) ? 1U : 0U;
}

uint8_t EncoderApp_GetDebugEnabled(void)
{
  return s_debug_enabled;
}

void EncoderApp_ToggleDebug(void)
{
  s_debug_enabled = (s_debug_enabled == 0U) ? 1U : 0U;
  printf("encoder debug %s\r\n", (s_debug_enabled != 0U) ? "on" : "off");
}

void EncoderApp_PrintDebug(void)
{
  if (s_debug_enabled == 0U)
  {
    return;
  }

  printf("enc "
         "fl raw=%u d=%d v=%ld | "
         "fr raw=%u d=%d v=%ld | "
         "rl raw=%u d=%d v=%ld | "
         "rr raw=%u d=%d v=%ld\r\n",
         (unsigned int)s_state.wheel[ENCODER_APP_FL].raw_count,
         (int)s_state.wheel[ENCODER_APP_FL].delta_ticks,
         (long)s_state.wheel[ENCODER_APP_FL].speed_ticks_per_sec,
         (unsigned int)s_state.wheel[ENCODER_APP_FR].raw_count,
         (int)s_state.wheel[ENCODER_APP_FR].delta_ticks,
         (long)s_state.wheel[ENCODER_APP_FR].speed_ticks_per_sec,
         (unsigned int)s_state.wheel[ENCODER_APP_RL].raw_count,
         (int)s_state.wheel[ENCODER_APP_RL].delta_ticks,
         (long)s_state.wheel[ENCODER_APP_RL].speed_ticks_per_sec,
         (unsigned int)s_state.wheel[ENCODER_APP_RR].raw_count,
         (int)s_state.wheel[ENCODER_APP_RR].delta_ticks,
         (long)s_state.wheel[ENCODER_APP_RR].speed_ticks_per_sec);
}

