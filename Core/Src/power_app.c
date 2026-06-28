#include "power_app.h"

#define INA228_ADDR_7BIT           0x40U
#define INA228_ADDR                (INA228_ADDR_7BIT << 1)

#define INA228_REG_VSHUNT          0x04U
#define INA228_REG_VBUS            0x05U

#define INA228_SHUNT_OHMS_UOHM     15000LL

#define POWER_APP_POLL_PERIOD_MS   200U
#define POWER_APP_I2C_TIMEOUT_MS   5U

#define POWER_APP_OVER_CURRENT_MA  5000L
#define POWER_APP_OVER_VOLTAGE_MV  26000U
#define POWER_APP_UNDER_VOLTAGE_MV 6000U

static I2C_HandleTypeDef *s_hi2c = NULL;
static PowerApp_State_t s_state = {0};
static uint32_t s_last_poll_ms = 0U;

static HAL_StatusTypeDef ina228_read_u24(uint8_t reg, uint32_t *value)
{
  uint8_t buf[3] = {0};
  HAL_StatusTypeDef status;

  if ((s_hi2c == NULL) || (value == NULL))
  {
    return HAL_ERROR;
  }

  status = HAL_I2C_Mem_Read(s_hi2c,
                            INA228_ADDR,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            buf,
                            sizeof(buf),
                            POWER_APP_I2C_TIMEOUT_MS);

  if (status != HAL_OK)
  {
    return status;
  }

  *value = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
  return HAL_OK;
}

static int32_t ina228_u24_to_s20(uint32_t raw24)
{
  int32_t value = (int32_t)(raw24 >> 4);

  if ((value & (1L << 19)) != 0)
  {
    value |= ~((1L << 20) - 1L);
  }

  return value;
}

static int16_t clamp_i32_to_i16(int32_t value)
{
  if (value > 32767L)
  {
    return 32767;
  }

  if (value < -32768L)
  {
    return -32768;
  }

  return (int16_t)value;
}

static uint16_t clamp_i32_to_u16(int32_t value)
{
  if (value <= 0L)
  {
    return 0U;
  }

  if (value > 65535L)
  {
    return 65535U;
  }

  return (uint16_t)value;
}

static uint8_t calc_power_w(uint16_t bus_voltage_mv, int16_t current_ma, uint8_t *flags)
{
  int32_t current_abs_ma = current_ma;
  int64_t power_mw;
  int64_t power_w;

  if (current_abs_ma < 0L)
  {
    current_abs_ma = -current_abs_ma;
  }

  power_mw = ((int64_t)bus_voltage_mv * (int64_t)current_abs_ma) / 1000LL;
  power_w = (power_mw + 500LL) / 1000LL;

  if (power_w > 255LL)
  {
    if (flags != NULL)
    {
      *flags |= POWER_APP_FLAG_POWER_SATURATED;
    }
    return 255U;
  }

  return (uint8_t)power_w;
}

void PowerApp_Init(I2C_HandleTypeDef *hi2c)
{
  s_hi2c = hi2c;
  s_last_poll_ms = 0U;

  s_state.bus_voltage_mv = 0U;
  s_state.current_ma = 0;
  s_state.power_w = 0U;
  s_state.flags = 0U;
  s_state.updated_ms = 0U;
  s_state.error_count = 0U;
}

void PowerApp_Poll(uint32_t now_ms)
{
  uint32_t vbus_raw24 = 0U;
  uint32_t vshunt_raw24 = 0U;
  int32_t vbus_raw20;
  int32_t vshunt_raw20;
  int64_t vbus_nv;
  int64_t vshunt_nv;
  int64_t current_ua;
  int32_t bus_voltage_mv;
  int32_t current_ma;
  uint8_t flags = 0U;

  if ((uint32_t)(now_ms - s_last_poll_ms) < POWER_APP_POLL_PERIOD_MS)
  {
    return;
  }

  s_last_poll_ms = now_ms;

  if ((s_hi2c == NULL) ||
      (HAL_I2C_IsDeviceReady(s_hi2c, INA228_ADDR, 1, POWER_APP_I2C_TIMEOUT_MS) != HAL_OK) ||
      (ina228_read_u24(INA228_REG_VBUS, &vbus_raw24) != HAL_OK) ||
      (ina228_read_u24(INA228_REG_VSHUNT, &vshunt_raw24) != HAL_OK))
  {
    s_state.bus_voltage_mv = 0U;
    s_state.current_ma = 0;
    s_state.power_w = 0U;
    s_state.flags = POWER_APP_FLAG_SENSOR_ERROR;
    s_state.updated_ms = now_ms;
    s_state.error_count++;
    return;
  }

  flags |= POWER_APP_FLAG_INA228_PRESENT;

  vbus_raw20 = (int32_t)(vbus_raw24 >> 4);
  vshunt_raw20 = ina228_u24_to_s20(vshunt_raw24);

  vbus_nv = ((int64_t)vbus_raw20 * 1953125LL) / 10LL;
  vshunt_nv = ((int64_t)vshunt_raw20 * 3125LL) / 10LL;
  current_ua = (vshunt_nv * 1000LL) / INA228_SHUNT_OHMS_UOHM;

  bus_voltage_mv = (int32_t)(vbus_nv / 1000000LL);
  current_ma = (int32_t)(current_ua / 1000LL);

  s_state.bus_voltage_mv = clamp_i32_to_u16(bus_voltage_mv);
  s_state.current_ma = clamp_i32_to_i16(current_ma);
  s_state.power_w = calc_power_w(s_state.bus_voltage_mv, s_state.current_ma, &flags);

  flags |= POWER_APP_FLAG_MEASUREMENT_VALID;

  if ((s_state.current_ma >= POWER_APP_OVER_CURRENT_MA) ||
      (s_state.current_ma <= -POWER_APP_OVER_CURRENT_MA))
  {
    flags |= POWER_APP_FLAG_OVER_CURRENT;
  }

  if (s_state.bus_voltage_mv > POWER_APP_OVER_VOLTAGE_MV)
  {
    flags |= POWER_APP_FLAG_OVER_VOLTAGE;
  }

  if ((s_state.bus_voltage_mv > 0U) && (s_state.bus_voltage_mv < POWER_APP_UNDER_VOLTAGE_MV))
  {
    flags |= POWER_APP_FLAG_UNDER_VOLTAGE;
  }

  s_state.flags = flags;
  s_state.updated_ms = now_ms;
}

const PowerApp_State_t *PowerApp_GetState(void)
{
  return &s_state;
}