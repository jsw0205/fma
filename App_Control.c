#include "App_Control.h"

#include "Can_Comms.h"
#include "Encoder_Motor.h"
#include "Lamp_Control.h"
#include "Ps2_Ctrl.h"
#include "Steering.h"

static uint8 computeFaultFlags(void)
{
    uint8 faultFlags = 0U;

    if ((g_canCmdSeen != FALSE) && (g_canCmdActive == FALSE))
    {
        faultFlags |= CAN_FAULT_COMM_TIMEOUT;
    }

    /* CAN_FAULT_POT_SENSOR: dropped 2026-08-15 per README — sharp turns
     * legitimately ride near the pot's mechanical rails, so a margin-based
     * check risks false positives. Always report 0 for this bit. */

    /* FAULT_ENCODER_SENSOR / FAULT_WATCHDOG_TRIP / FAULT_UNDERVOLTAGE:
     * no encoder-fault heuristic, no live watchdog (disabled in
     * Cpu0_Main.c), and no supply-voltage ADC channel exist in this
     * firmware yet, so these bits stay 0 until that hardware/logic is
     * added. */

    return faultFlags;
}

void appControlInit(void)
{
    Ps2_Ctrl_Init();
    Lamp_Control_Init();
    Can_Comms_Init();
    Encoder_Motor_Init();
    Steering_Init();
}

void appControlStep(void)
{
    boolean enabled;
    sint16 drivePwm;
    sint16 targetRpm;
    float32 commandRpm;
    float32 commandSteer;
    boolean commandReverse;
    boolean flatStopMode;
    uint8 appliedStopMode;
    sint16 steerPwmDuty10;

    Ps2_Ctrl_Update_5ms();
    Can_Comms_Update_5ms();
    Lamp_Control_Update_5ms();

    if (g_ps2CanControlMode != FALSE)
    {
        enabled = ((g_canCmdActive != FALSE) &&
                   (g_canCmdEnable != FALSE)) ? TRUE : FALSE;
        commandReverse = (g_canCmdTargetRpm < 0) ? TRUE : FALSE;
        commandRpm = (g_canCmdTargetRpm < 0) ?
                     (float32)(-g_canCmdTargetRpm) :
                     (float32)g_canCmdTargetRpm;
        commandSteer = (float32)g_canCmdSteerAngle;
        flatStopMode = g_canCmdFlatStopMode;
        targetRpm = g_canCmdTargetRpm;
    }
    else
    {
        enabled = Ps2_Ctrl_GetEnable();
        commandReverse = g_ps2ReverseMode;
        commandRpm = (float32)g_ps2TargetRpm;
        commandSteer = Ps2_Ctrl_GetTargetSteer();
        flatStopMode = g_ps2FlatStopMode;
        targetRpm = (g_ps2ReverseMode != FALSE) ?
                    -(sint16)g_ps2TargetRpm : (sint16)g_ps2TargetRpm;
    }

    if (enabled == FALSE)
    {
        Encoder_Motor_SetEnable(FALSE);
        Encoder_Motor_SetStopHoldEnable(TRUE);
        Steering_SetEnable(FALSE);

        Encoder_Motor_Update_5ms();
        Steering_Update_5ms();

        drivePwm = (g_driveReverseActive != FALSE) ?
                   -(sint16)g_drivePwmPercent :
                   (sint16)g_drivePwmPercent;
        steerPwmDuty10 = (sint16)(g_steeringPwmPercent * 10.0f);

        Can_Comms_SendDriveStatus(drivePwm, targetRpm);
        Can_Comms_SendSteeringStatus();
        Can_Comms_SendDiagStatus(CAN_APPLIED_STOP_MODE_DISABLED,
                                  computeFaultFlags(),
                                  steerPwmDuty10,
                                  0U);
        return;
    }

    Encoder_Motor_SetTargetRpm(commandRpm, commandReverse);
    Encoder_Motor_SetStopHoldEnable(
        (flatStopMode == FALSE) ? TRUE : FALSE);
    Encoder_Motor_SetEnable(TRUE);

    Steering_SetTargetAngle(commandSteer);
    Steering_SetEnable(TRUE);

    Encoder_Motor_Update_5ms();
    Steering_Update_5ms();

    drivePwm = (g_driveReverseActive != FALSE) ?
               -(sint16)g_drivePwmPercent :
               (sint16)g_drivePwmPercent;
    steerPwmDuty10 = (sint16)(g_steeringPwmPercent * 10.0f);

    appliedStopMode = (flatStopMode != FALSE) ?
                      CAN_APPLIED_STOP_MODE_FLAT :
                      CAN_APPLIED_STOP_MODE_HOLD;

    Can_Comms_SendDriveStatus(drivePwm, targetRpm);
    Can_Comms_SendSteeringStatus();
    Can_Comms_SendDiagStatus(appliedStopMode,
                              computeFaultFlags(),
                              steerPwmDuty10,
                              0U);
}
