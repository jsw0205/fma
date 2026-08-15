#ifndef CAN_COMMS_H
#define CAN_COMMS_H

#include "Ifx_Types.h"

#define CAN_DRIVE_STATUS_ID  0x102U
#define CAN_STEERING_STATUS_ID  0x101U
#define CAN_COMMAND_ID  0x200U
#define CAN_CONTROL_META_ID  0x203U
#define CAN_DIAG_STATUS_ID  0x104U

/* fault_flags bits, DIAG_STATUS (0x104) byte1 */
#define CAN_FAULT_COMM_TIMEOUT    (1U << 0)
#define CAN_FAULT_POT_SENSOR      (1U << 1)  /* dropped 2026-08-15, always 0 (see App_Control.c) */
#define CAN_FAULT_ENCODER_SENSOR  (1U << 2)
#define CAN_FAULT_WATCHDOG_TRIP   (1U << 3)
#define CAN_FAULT_UNDERVOLTAGE    (1U << 4)

/* applied_stop_mode values, DIAG_STATUS (0x104) byte0 */
#define CAN_APPLIED_STOP_MODE_DISABLED  0U  /* motor not enabled (StopHoldEnable forced TRUE, drive off) */
#define CAN_APPLIED_STOP_MODE_FLAT      1U  /* enabled, StopHoldEnable == FALSE (coasting) */
#define CAN_APPLIED_STOP_MODE_HOLD      2U  /* enabled, StopHoldEnable == TRUE (active hold/brake) */

extern volatile uint32 g_canTxCount;
extern volatile uint32 g_canTxBusyCount;
extern volatile uint32 g_canRxCount;
extern volatile uint32 g_canInitStatus;
extern volatile uint32 g_canNodeInitStatus;
extern volatile uint32 g_canMsgObjInitStatus;
extern volatile uint32 g_canLastSendStatus;
extern volatile uint8 g_canLastPayload[8];
extern volatile sint16 g_canCmdTargetRpm;
extern volatile sint16 g_canCmdSteerAngle;
extern volatile boolean g_canCmdEnable;
extern volatile boolean g_canCmdActive;
extern volatile boolean g_canCmdSeen;
extern volatile boolean g_canCmdFlatStopMode;

void Can_Comms_Init(void);
void Can_Comms_Update_5ms(void);
void Can_Comms_SendDriveStatus(sint16 pwmDuty, sint16 targetRpm);
void Can_Comms_SendSteeringStatus(void);
void Can_Comms_SendDiagStatus(uint8 appliedStopMode,
                              uint8 faultFlags,
                              sint16 steerPwmDuty,
                              uint16 supplyVoltageMv);

#endif /* CAN_COMMS_H */
