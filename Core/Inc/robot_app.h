#ifndef ROBOT_APP_H
#define ROBOT_APP_H

#include "main.h"
#include <stdint.h>

typedef enum
{
  MOTION_STOP = 0,
  MOTION_FORWARD,
  MOTION_BACKWARD,
  MOTION_LEFT,
  MOTION_RIGHT,
  MOTION_ROTATE_CCW,
  MOTION_ROTATE_CW
} MotionMode_t;

void RobotApp_Init(I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2);
void RobotApp_PrintHelp(void);
void RobotApp_HandleChar(uint8_t ch);

void RobotApp_SetMotion(MotionMode_t mode);
void RobotApp_SetSpeed(uint16_t speed_pct);
void RobotApp_Stop(void);
MotionMode_t RobotApp_GetMotion(void);
uint16_t RobotApp_GetSpeed(void);

#endif
