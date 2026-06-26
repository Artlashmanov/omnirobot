#include "robot_app.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

/* ---------- External handles ---------- */
static I2C_HandleTypeDef *g_hi2c = NULL;
static TIM_HandleTypeDef *g_htim1 = NULL;
static TIM_HandleTypeDef *g_htim2 = NULL;

/* ---------- Speed model ----------
 * ВНЕШНИЙ интерфейс: speed_pct = 0..100
 * ВНУТРЕННИЙ исполнительный уровень: PWM = 0..999
 *
 * PWM_START — минимальный PWM, с которого моторы уже должны трогаться.
 * Если 30% всё ещё мало, поднимем только этот порог.
 */
#define PWM_MIN_VALUE   0U
#define PWM_MAX_VALUE   999U
#define PWM_START_VALUE 350U

/* ---------- State ---------- */
static uint16_t g_speed_pct = 40;   /* наружу в status / CAN */
static uint16_t g_speed_pwm = 0;    /* внутренняя исполнительная величина */
static MotionMode_t g_motion = MOTION_STOP;

/* ---------- INA228 ---------- */
#define INA228_ADDR_7BIT           0x40
#define INA228_ADDR                (INA228_ADDR_7BIT << 1)

#define INA228_REG_VSHUNT          0x04
#define INA228_REG_VBUS            0x05
#define INA228_REG_MANUFACTURER_ID 0x3E
#define INA228_REG_DEVICE_ID       0x3F

#define INA228_SHUNT_OHMS_UOHM     15000LL

/* ---------- UART via BSP COM1 ---------- */
static void uart_send_str(const char *s)
{
  if (s == NULL) return;
  printf("%s", s);
}

static void uart_send_fmt(const char *fmt, ...)
{
  char buf[160];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  uart_send_str(buf);
}

/* ---------- Percent -> PWM ---------- */
static uint16_t speed_percent_to_pwm(uint16_t speed_pct)
{
  uint32_t span;
  uint32_t pwm;

  if (speed_pct == 0U)
  {
    return 0U;
  }

  if (speed_pct > 100U)
  {
    speed_pct = 100U;
  }

  span = (uint32_t)(PWM_MAX_VALUE - PWM_START_VALUE);
  pwm = (uint32_t)PWM_START_VALUE + ((uint32_t)speed_pct * span) / 100U;

  if (pwm > PWM_MAX_VALUE)
  {
    pwm = PWM_MAX_VALUE;
  }

  return (uint16_t)pwm;
}

/* ---------- Motors ---------- */

static void motor_front_left_forward(uint16_t pwm)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(g_htim1, TIM_CHANNEL_1, pwm);
}

static void motor_front_left_backward(uint16_t pwm)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(g_htim1, TIM_CHANNEL_1, pwm);
}

static void motor_front_left_stop(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(g_htim1, TIM_CHANNEL_1, 0);
}

static void motor_front_right_forward(uint16_t pwm)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(g_htim1, TIM_CHANNEL_2, pwm);
}

static void motor_front_right_backward(uint16_t pwm)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(g_htim1, TIM_CHANNEL_2, pwm);
}

static void motor_front_right_stop(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(g_htim1, TIM_CHANNEL_2, 0);
}

static void motor_rear_left_forward(uint16_t pwm)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(g_htim1, TIM_CHANNEL_3, pwm);
}

static void motor_rear_left_backward(uint16_t pwm)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(g_htim1, TIM_CHANNEL_3, pwm);
}

static void motor_rear_left_stop(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(g_htim1, TIM_CHANNEL_3, 0);
}

/* 4-й мотор: PWM на TIM2_CH3 / PB10, DIR перенесены на PB14/PB15.
 * PB6/PB7 освобождены под TIM4 encoder.
 */
static void motor_rear_right_forward(uint16_t pwm)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(g_htim2, TIM_CHANNEL_3, pwm);
}

static void motor_rear_right_backward(uint16_t pwm)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(g_htim2, TIM_CHANNEL_3, pwm);
}

static void motor_rear_right_stop(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(g_htim2, TIM_CHANNEL_3, 0);
}

/* ---------- Motion ---------- */

static void robot_stop_internal(void)
{
  motor_front_left_stop();
  motor_front_right_stop();
  motor_rear_left_stop();
  motor_rear_right_stop();
}

static void robot_forward(uint16_t pwm)
{
  motor_front_left_forward(pwm);
  motor_front_right_forward(pwm);
  motor_rear_left_forward(pwm);
  motor_rear_right_forward(pwm);
}

static void robot_backward(uint16_t pwm)
{
  motor_front_left_backward(pwm);
  motor_front_right_backward(pwm);
  motor_rear_left_backward(pwm);
  motor_rear_right_backward(pwm);
}

static void robot_left(uint16_t pwm)
{
  motor_front_left_backward(pwm);
  motor_front_right_forward(pwm);
  motor_rear_left_forward(pwm);
  motor_rear_right_backward(pwm);
}

static void robot_right(uint16_t pwm)
{
  motor_front_left_forward(pwm);
  motor_front_right_backward(pwm);
  motor_rear_left_backward(pwm);
  motor_rear_right_forward(pwm);
}

static void robot_rotate_ccw(uint16_t pwm)
{
  motor_front_left_backward(pwm);
  motor_front_right_forward(pwm);
  motor_rear_left_backward(pwm);
  motor_rear_right_forward(pwm);
}

static void robot_rotate_cw(uint16_t pwm)
{
  motor_front_left_forward(pwm);
  motor_front_right_backward(pwm);
  motor_rear_left_forward(pwm);
  motor_rear_right_backward(pwm);
}

static const char* motion_mode_to_str(MotionMode_t mode)
{
  switch (mode)
  {
    case MOTION_STOP:       return "stop";
    case MOTION_FORWARD:    return "forward";
    case MOTION_BACKWARD:   return "backward";
    case MOTION_LEFT:       return "left";
    case MOTION_RIGHT:      return "right";
    case MOTION_ROTATE_CCW: return "rotate_ccw";
    case MOTION_ROTATE_CW:  return "rotate_cw";
    default:                return "unknown";
  }
}

static void apply_motion_state(void)
{
  switch (g_motion)
  {
    case MOTION_STOP:
      robot_stop_internal();
      break;

    case MOTION_FORWARD:
      robot_forward(g_speed_pwm);
      break;

    case MOTION_BACKWARD:
      robot_backward(g_speed_pwm);
      break;

    case MOTION_LEFT:
      robot_left(g_speed_pwm);
      break;

    case MOTION_RIGHT:
      robot_right(g_speed_pwm);
      break;

    case MOTION_ROTATE_CCW:
      robot_rotate_ccw(g_speed_pwm);
      break;

    case MOTION_ROTATE_CW:
      robot_rotate_cw(g_speed_pwm);
      break;

    default:
      robot_stop_internal();
      g_motion = MOTION_STOP;
      break;
  }
}

static void uart_send_status(void)
{
  uart_send_fmt("mode=%s speed_pct=%u pwm=%u\r\n",
                motion_mode_to_str(g_motion),
                g_speed_pct,
                g_speed_pwm);
}

/* ---------- I2C / INA228 ---------- */

static void i2c_scan_bus(void)
{
  uint8_t found = 0;

  uart_send_str("\r\nI2C scan start...\r\n");

  for (uint8_t addr = 1; addr < 127; addr++)
  {
    if (HAL_I2C_IsDeviceReady(g_hi2c, (uint16_t)(addr << 1), 2, 20) == HAL_OK)
    {
      uart_send_fmt("Found: 0x%02X\r\n", addr);
      found = 1;
    }
  }

  if (!found)
  {
    uart_send_str("No I2C devices found\r\n");
  }

  uart_send_str("I2C scan done\r\n");
}

static HAL_StatusTypeDef ina228_read_u16(uint8_t reg, uint16_t *value)
{
  uint8_t buf[2];
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(g_hi2c, INA228_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
  if (st != HAL_OK) return st;

  *value = ((uint16_t)buf[0] << 8) | buf[1];
  return HAL_OK;
}

static HAL_StatusTypeDef ina228_read_u24(uint8_t reg, uint32_t *value)
{
  uint8_t buf[3];
  HAL_StatusTypeDef st = HAL_I2C_Mem_Read(g_hi2c, INA228_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 3, 100);
  if (st != HAL_OK) return st;

  *value = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
  return HAL_OK;
}

static int32_t ina228_u24_to_s20(uint32_t raw24)
{
  int32_t v = (int32_t)(raw24 >> 4);

  if (v & (1 << 19))
  {
    v |= ~((1 << 20) - 1);
  }

  return v;
}

static void ina228_print_info(void)
{
  uint16_t man_id = 0;
  uint16_t dev_id = 0;

  uint32_t vbus_raw24 = 0;
  uint32_t vshunt_raw24 = 0;

  int32_t vbus_raw20 = 0;
  int32_t vshunt_raw20 = 0;

  int64_t vbus_nV_64 = 0;
  int64_t vshunt_nV_64 = 0;
  int64_t current_uA_64 = 0;

  int32_t vbus_mV = 0;
  int32_t vshunt_uV = 0;
  int32_t current_mA = 0;

  if (HAL_I2C_IsDeviceReady(g_hi2c, INA228_ADDR, 2, 50) != HAL_OK)
  {
    uart_send_str("INA228 not found at 0x40\r\n");
    return;
  }

  uart_send_str("INA228 found at 0x40\r\n");

  if (ina228_read_u16(INA228_REG_MANUFACTURER_ID, &man_id) == HAL_OK)
    uart_send_fmt("MANUFACTURER_ID = 0x%04X\r\n", man_id);
  else
    uart_send_str("Read MANUFACTURER_ID failed\r\n");

  if (ina228_read_u16(INA228_REG_DEVICE_ID, &dev_id) == HAL_OK)
    uart_send_fmt("DEVICE_ID = 0x%04X\r\n", dev_id);
  else
    uart_send_str("Read DEVICE_ID failed\r\n");

  if (ina228_read_u24(INA228_REG_VBUS, &vbus_raw24) != HAL_OK)
  {
    uart_send_str("Read VBUS raw failed\r\n");
    return;
  }

  if (ina228_read_u24(INA228_REG_VSHUNT, &vshunt_raw24) != HAL_OK)
  {
    uart_send_str("Read VSHUNT raw failed\r\n");
    return;
  }

  vbus_raw20 = (int32_t)(vbus_raw24 >> 4);
  vshunt_raw20 = ina228_u24_to_s20(vshunt_raw24);

  vbus_nV_64   = ((int64_t)vbus_raw20   * 1953125LL) / 10LL;
  vshunt_nV_64 = ((int64_t)vshunt_raw20 * 3125LL) / 10LL;
  current_uA_64 = (vshunt_nV_64 * 1000LL) / INA228_SHUNT_OHMS_UOHM;

  vbus_mV   = (int32_t)(vbus_nV_64 / 1000000LL);
  vshunt_uV = (int32_t)(vshunt_nV_64 / 1000LL);
  current_mA = (int32_t)(current_uA_64 / 1000LL);

  uart_send_fmt("VBUS_RAW24   = 0x%06lX\r\n", vbus_raw24);
  uart_send_fmt("VBUS_RAW20   = %ld\r\n", (long)vbus_raw20);
  uart_send_fmt("VBUS         = %ld.%03ld V\r\n",
                (long)(vbus_mV / 1000),
                (long)(vbus_mV % 1000));

  uart_send_fmt("VSHUNT_RAW24 = 0x%06lX\r\n", vshunt_raw24);
  uart_send_fmt("VSHUNT_RAW20 = %ld\r\n", (long)vshunt_raw20);
  uart_send_fmt("VSHUNT       = %ld.%06ld V\r\n",
                (long)(vshunt_uV / 1000000),
                (long)(vshunt_uV % 1000000));

  uart_send_fmt("CURRENT_EST  = %ld.%03ld A\r\n",
                (long)(current_mA / 1000),
                (long)(current_mA % 1000));
}

/* ---------- Public ---------- */

void RobotApp_Init(I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2)
{
  g_hi2c = hi2c;
  g_htim1 = htim1;
  g_htim2 = htim2;

  HAL_TIM_PWM_Start(g_htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(g_htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(g_htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(g_htim2, TIM_CHANNEL_3);

  g_motion = MOTION_STOP;
  g_speed_pct = 40;
  g_speed_pwm = speed_percent_to_pwm(g_speed_pct);

  robot_stop_internal();
}

void RobotApp_PrintHelp(void)
{
  uart_send_str("\r\n=== OmniRobot UART ready ===\r\n");
  uart_send_str("f - forward\r\n");
  uart_send_str("b - backward\r\n");
  uart_send_str("l - left\r\n");
  uart_send_str("r - right\r\n");
  uart_send_str("q - rotate CCW\r\n");
  uart_send_str("e - rotate CW\r\n");
  uart_send_str("s - stop\r\n");
  uart_send_str("+ - speed up (5%)\r\n");
  uart_send_str("- - speed down (5%)\r\n");
  uart_send_str("x - encoder debug on/off\r\n");
  uart_send_str("c - CAN debug on/off\r\n");
  uart_send_str("i - I2C scan\r\n");
  uart_send_str("v - INA228 check\r\n");
  uart_send_str("h - help\r\n");
  uart_send_status();
}

void RobotApp_HandleChar(uint8_t ch)
{
  switch (ch)
  {
    case 'f':
      g_motion = MOTION_FORWARD;
      apply_motion_state();
      uart_send_status();
      break;

    case 'b':
      g_motion = MOTION_BACKWARD;
      apply_motion_state();
      uart_send_status();
      break;

    case 'l':
      g_motion = MOTION_LEFT;
      apply_motion_state();
      uart_send_status();
      break;

    case 'r':
      g_motion = MOTION_RIGHT;
      apply_motion_state();
      uart_send_status();
      break;

    case 'q':
      g_motion = MOTION_ROTATE_CCW;
      apply_motion_state();
      uart_send_status();
      break;

    case 'e':
      g_motion = MOTION_ROTATE_CW;
      apply_motion_state();
      uart_send_status();
      break;

    case 's':
      g_motion = MOTION_STOP;
      apply_motion_state();
      uart_send_status();
      break;

    case '+':
      if (g_speed_pct <= 95U) g_speed_pct += 5U;
      else g_speed_pct = 100U;
      g_speed_pwm = speed_percent_to_pwm(g_speed_pct);
      apply_motion_state();
      uart_send_status();
      break;

    case '-':
      if (g_speed_pct >= 5U) g_speed_pct -= 5U;
      else g_speed_pct = 0U;
      g_speed_pwm = speed_percent_to_pwm(g_speed_pct);
      apply_motion_state();
      uart_send_status();
      break;

    case 'i':
      i2c_scan_bus();
      break;

    case 'v':
      ina228_print_info();
      break;

    case 'h':
      RobotApp_PrintHelp();
      break;

    case '\r':
    case '\n':
      break;

    default:
      break;
  }
}

void RobotApp_SetMotion(MotionMode_t mode)
{
  g_motion = mode;
  apply_motion_state();
}

void RobotApp_SetSpeed(uint16_t speed_pct)
{
  if (speed_pct > 100U)
  {
    speed_pct = 100U;
  }

  g_speed_pct = speed_pct;
  g_speed_pwm = speed_percent_to_pwm(g_speed_pct);
  apply_motion_state();
}

void RobotApp_Stop(void)
{
  g_motion = MOTION_STOP;
  apply_motion_state();
}

MotionMode_t RobotApp_GetMotion(void)
{
  return g_motion;
}

uint16_t RobotApp_GetSpeed(void)
{
  return g_speed_pct;
}
