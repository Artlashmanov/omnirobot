#include "can_app.h"
#include "robot_app.h"
#include "encoder_app.h"

extern FDCAN_HandleTypeDef hfdcan1;

#define PROTO_VERSION       1U

#define ID_CMD_MOTION       0x101U
#define ID_CMD_STOP         0x102U
#define ID_CMD_PING         0x103U
#define ID_CMD_STATUS_REQ   0x104U

#define ID_EVT_ACK          0x181U
#define ID_EVT_STATUS       0x182U
#define ID_EVT_PONG         0x183U
#define ID_EVT_BASE_STATUS  0x190U
#define ID_EVT_WHEEL_STATE  0x191U

#define CAN_APP_BASE_STATUS_PERIOD_MS  200U
#define CAN_APP_WHEEL_STATE_PERIOD_MS  100U

#define CAN_APP_STATUS_FLAG_ENCODERS_READY 0x01U
#define CAN_APP_STATUS_FLAG_ENCODER_DEBUG  0x02U

static uint8_t s_telem_seq = 0U;
static uint32_t s_last_base_status_ms = 0U;
static uint32_t s_last_wheel_state_ms = 0U;

static void put_i16_le(uint8_t *dst, int16_t value)
{
  uint16_t u = (uint16_t)value;

  dst[0] = (uint8_t)(u & 0xFFU);
  dst[1] = (uint8_t)((u >> 8) & 0xFFU);
}

static void put_i32_le(uint8_t *dst, int32_t value)
{
  uint32_t u = (uint32_t)value;

  dst[0] = (uint8_t)(u & 0xFFU);
  dst[1] = (uint8_t)((u >> 8) & 0xFFU);
  dst[2] = (uint8_t)((u >> 16) & 0xFFU);
  dst[3] = (uint8_t)((u >> 24) & 0xFFU);
}

static HAL_StatusTypeDef CAN_App_TrySendStd8(uint16_t std_id, const uint8_t data[8])
{
  FDCAN_TxHeaderTypeDef txHeader = {0};

  txHeader.Identifier = std_id;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, (uint8_t *)data);
}

static void CAN_App_SendStd8(uint16_t std_id, const uint8_t data[8])
{
  if (CAN_App_TrySendStd8(std_id, data) != HAL_OK)
  {
    Error_Handler();
  }
}

static void CAN_App_SendBaseStatus(void)
{
  uint8_t txData[8] = {0};
  uint8_t status_flags = 0U;

  if (EncoderApp_IsReady() != 0U)
  {
    status_flags |= CAN_APP_STATUS_FLAG_ENCODERS_READY;
  }

  if (EncoderApp_GetDebugEnabled() != 0U)
  {
    status_flags |= CAN_APP_STATUS_FLAG_ENCODER_DEBUG;
  }

  txData[0] = PROTO_VERSION;
  txData[1] = s_telem_seq++;
  txData[2] = (uint8_t)RobotApp_GetMotion();
  txData[3] = (uint8_t)(RobotApp_GetSpeed() > 255 ? 255 : RobotApp_GetSpeed());
  txData[4] = ENCODER_APP_COUNT;
  txData[5] = status_flags;
  txData[6] = 0U; /* error_flags, reserved for future faults */
  txData[7] = 0U;

  (void)CAN_App_TrySendStd8(ID_EVT_BASE_STATUS, txData);
}

static void CAN_App_SendWheelState(uint8_t wheel_index)
{
  const EncoderApp_WheelState_t *wheel = EncoderApp_GetWheelState(wheel_index);
  uint8_t txData[8] = {0};

  if (wheel == NULL)
  {
    return;
  }

  txData[0] = wheel->index;
  txData[1] = wheel->flags;
  put_i16_le(&txData[2], wheel->delta_ticks);
  put_i32_le(&txData[4], wheel->speed_ticks_per_sec);

  (void)CAN_App_TrySendStd8(ID_EVT_WHEEL_STATE, txData);
}

void CAN_App_Init(void)
{
  FDCAN_FilterTypeDef sFilterConfig = {0};

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x000;
  sFilterConfig.FilterID2 = 0x000;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT,
                                   FDCAN_FILTER_REMOTE,
                                   FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  s_telem_seq = 0U;
  s_last_base_status_ms = HAL_GetTick();
  s_last_wheel_state_ms = HAL_GetTick();
}

void CAN_App_TelemetryTask(uint32_t now_ms)
{
  if ((uint32_t)(now_ms - s_last_base_status_ms) >= CAN_APP_BASE_STATUS_PERIOD_MS)
  {
    s_last_base_status_ms = now_ms;
    CAN_App_SendBaseStatus();
  }

  if ((uint32_t)(now_ms - s_last_wheel_state_ms) >= CAN_APP_WHEEL_STATE_PERIOD_MS)
  {
    s_last_wheel_state_ms = now_ms;

    for (uint8_t i = 0U; i < ENCODER_APP_COUNT; i++)
    {
      CAN_App_SendWheelState(i);
    }
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rxHeader = {0};
  uint8_t rxData[8] = {0};
  uint8_t txData[8] = {0};
  uint8_t seq = 0;

  if (hfdcan->Instance != FDCAN1)
  {
    return;
  }

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
  {
    return;
  }

  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
  {
    Error_Handler();
  }

  if (rxHeader.IdType != FDCAN_STANDARD_ID)
  {
    return;
  }

  seq = rxData[1];

  switch (rxHeader.Identifier)
  {
    case ID_CMD_PING:
      txData[0] = PROTO_VERSION;
      txData[1] = seq;
      txData[2] = 0;
      txData[3] = 0;
      txData[4] = 0;
      txData[5] = 0;
      txData[6] = 0;
      txData[7] = 0;
      CAN_App_SendStd8(ID_EVT_PONG, txData);
      break;

    case ID_CMD_STATUS_REQ:
      txData[0] = PROTO_VERSION;
      txData[1] = seq;
      txData[2] = (uint8_t)RobotApp_GetMotion();
      txData[3] = (uint8_t)(RobotApp_GetSpeed() > 255 ? 255 : RobotApp_GetSpeed());
      txData[4] = 0;
      txData[5] = 0;
      txData[6] = 0;
      txData[7] = 0;
      CAN_App_SendStd8(ID_EVT_STATUS, txData);
      break;

    case ID_CMD_STOP:
      RobotApp_Stop();

      txData[0] = PROTO_VERSION;
      txData[1] = seq;
      txData[2] = (uint8_t)(ID_CMD_STOP & 0xFFU);
      txData[3] = 0;
      txData[4] = (uint8_t)RobotApp_GetMotion();
      txData[5] = (uint8_t)(RobotApp_GetSpeed() > 255 ? 255 : RobotApp_GetSpeed());
      txData[6] = 0;
      txData[7] = 0;
      CAN_App_SendStd8(ID_EVT_ACK, txData);
      break;

    case ID_CMD_MOTION:
      RobotApp_SetMotion((MotionMode_t)rxData[2]);
      RobotApp_SetSpeed(rxData[3]);

      txData[0] = PROTO_VERSION;
      txData[1] = seq;
      txData[2] = (uint8_t)(ID_CMD_MOTION & 0xFFU);
      txData[3] = 0;
      txData[4] = (uint8_t)RobotApp_GetMotion();
      txData[5] = (uint8_t)(RobotApp_GetSpeed() > 255 ? 255 : RobotApp_GetSpeed());
      txData[6] = 0;
      txData[7] = 0;
      CAN_App_SendStd8(ID_EVT_ACK, txData);
      break;

    default:
      break;
  }
}
