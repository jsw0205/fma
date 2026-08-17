#include "Encoder_Motor.h"

#include "IfxCpu.h"
#include "IfxGtm.h"
#include "IfxGtm_Cmu.h"
#include "IfxGtm_Tom.h"
#include "IfxGtm_PinMap.h"
#include "IfxGtm_Tom_Pwm.h"
#include "IfxPort.h"
#include "IfxScuEru.h"
#include "IfxSrc.h"
#include "IfxStm.h"

#define ISR_PRIORITY_ENCODER_A  20
#define ISR_PRIORITY_ENCODER_B  21

#define ENCODER_A_PORT  MODULE_P15
#define ENCODER_A_PIN   4
#define ENCODER_B_PORT  MODULE_P33
#define ENCODER_B_PIN   7

#define DRIVE_PWM_OUT      IfxGtm_TOM0_11_TOUT13_P00_4_OUT
#define DRIVE_PWM_FREQ_HZ  20000.0f
#define DRIVE_DIR_PORT     MODULE_P00
#define DRIVE_DIR_PIN      8
#define DRIVE_BRK_PORT     MODULE_P00
#define DRIVE_BRK_PIN      9
#define DRIVE_REVERSE_RPM_THRESHOLD 2.0f
#define DRIVE_RPM_MAX      200.0f
#define DRIVE_PWM_FALL_LIMIT_PERCENT  4.0f
#define DRIVE_RUN_ADAPT_STEP_PERCENT   0.5f
#define DRIVE_RUN_COUNT_WINDOW_STEPS   4U
#define DRIVE_RUN_DELTA_DEADBAND_COUNT 0
#define DRIVE_HOLD_DELTA_DEADBAND_COUNT  2
#define DRIVE_HOLD_ADAPT_STEP_PERCENT    2.0f
#define DRIVE_HOLD_ADAPT_MAX_PERCENT     100.0f
#define DRIVE_STOP_BRAKE_PWM_PERCENT  15.0f
#define DRIVE_STOP_BRAKE_STEPS        1U
#define DRIVE_STOP_SETTLE_STEPS       6U
#define DRIVE_FLAT_STOP_BRAKE_PWM_PERCENT  10.0f
#define DRIVE_FLAT_STOP_BRAKE_STEPS        2U

#define ENCODER_TIME_BUFFER_SIZE  32U
#define ENCODER_MIN_BUFFER_COUNT  4U
#define ENCODER_STOP_TIMEOUT_SEC  0.05f
#define ENCODER_MIN_EDGE_TICKS    20000U
#define ENCODER_DIRECTION_CONFIRM_COUNT  4U
#define ENCODER_RPM_FILTER_ALPHA  0.45f
#define CONTROL_PERIOD_SEC        0.005f
#define COUNT_RPM_WINDOW_STEPS    40U

typedef struct
{
    uint32 dtBuffer[ENCODER_TIME_BUFFER_SIZE];
    uint32 dtSum;
    uint32 previousTick;
    uint32 lastEdgeTick;
    uint8 bufferIndex;
    uint8 bufferCount;
    sint8 direction;
} EncoderTimeFilter;

typedef enum
{
    DRIVE_STOP_STATE_NONE = 0,
    DRIVE_STOP_STATE_BRAKE,
    DRIVE_STOP_STATE_SETTLE,
    DRIVE_STOP_STATE_HOLD
} DriveStopState;

volatile sint32 g_encoderCount = 0;
volatile sint32 g_encoderDeltaCount = 0;
volatile uint8 g_encoderState = 0U;
volatile uint8 g_encoderRawA = 0U;
volatile uint8 g_encoderRawB = 0U;
volatile uint8 g_encoderPollA = 0U;
volatile uint8 g_encoderPollB = 0U;
volatile uint32 g_encoderIsrACount = 0U;
volatile uint32 g_encoderIsrBCount = 0U;
volatile uint32 g_encoderRejectedEdgeCount = 0U;
volatile uint32 g_encoderUpdateCount = 0U;
volatile uint32 g_encoderCountsPerRev = 300U;
volatile uint32 g_encoderAverageEdgeTicks = 0U;
volatile uint8 g_encoderActiveBufferSize = 16U;
volatile float32 g_encoderStmFrequency = 0.0f;
volatile float32 g_encoderRawTimeRpm = 0.0f;
volatile float32 g_encoderActualRpm = 0.0f;
volatile float32 g_encoderDeltaRpm = 0.0f;
volatile sint32 g_encoderWindowDeltaCount = 0;
volatile float32 g_driveTargetRpm = 0.0f;
volatile float32 g_drivePwmPercent = 0.0f;
volatile float32 g_driveTargetPwmPercent = 0.0f;
volatile boolean g_driveReverseActive = FALSE;
volatile boolean g_driveTestEnable = FALSE;
volatile float32 g_driveTestPwmPercent = 0.0f;
volatile float32 g_driveStopBrakePwmPercent = 0.0f;
volatile float32 g_driveHoldPwmPercent = 0.0f;
volatile sint32 g_driveHoldPosError = 0;
volatile sint8 g_driveHoldDirection = 0;

static float32 g_targetRpm = 0.0f;
static float32 g_previousTargetRpm = 0.0f;
static boolean g_previousTargetReverse = FALSE;
static boolean g_posHoldActive = FALSE;
static sint32 g_posHoldCount = 0;
static boolean g_targetReverse = FALSE;
static float32 g_openLoopPwmPercent = 0.0f;
static boolean g_openLoopReverse = FALSE;
static boolean g_enable = FALSE;
static boolean g_stopHoldEnable = TRUE;
static float32 g_driveAdaptivePwmCommand = 0.0f;
static float32 g_driveAdaptiveTargetCountRemainder = 0.0f;
static sint32 g_driveAdaptiveCountSum = 0;
static uint8 g_driveAdaptiveWindowStepCount = 0U;
static DriveStopState g_stopState = DRIVE_STOP_STATE_NONE;
static uint8 g_stopStepCount = 0U;
static boolean g_stopBrakeReverse = FALSE;
static boolean g_flatStopBrakeReverse = FALSE;
static uint8 g_flatStopBrakeStepCount = 0U;
static uint8 g_previousEncoderState = 0U;
static sint32 g_previousEncoderCount = 0;
static sint8 g_encoderStableDirection = 0;
static uint8 g_encoderReverseCandidateCount = 0U;
static sint32 g_encoderWindowCountSum = 0;
static uint8 g_encoderWindowStepCount = 0U;
static EncoderTimeFilter g_encoderTimeFilter = {
    {0U}, 0U, 0U, 0U, 0U, 0U, 1
};

static IfxGtm_Tom_Pwm_Driver g_drivePwm;
static uint32 g_drivePwmPeriodTicks = 0U;

static uint8 readEncoderState(void)
{
    uint8 stateA =
        (IfxPort_getPinState(&ENCODER_A_PORT, ENCODER_A_PIN) != FALSE) ? 1U : 0U;
    uint8 stateB =
        (IfxPort_getPinState(&ENCODER_B_PORT, ENCODER_B_PIN) != FALSE) ? 1U : 0U;

    g_encoderRawA = stateA;
    g_encoderRawB = stateB;

    return (uint8)((stateA << 1U) | stateB);
}

static void updateEncoderPollPins(void)
{
    g_encoderPollA =
        (IfxPort_getPinState(&ENCODER_A_PORT, ENCODER_A_PIN) != FALSE) ? 1U : 0U;
    g_encoderPollB =
        (IfxPort_getPinState(&ENCODER_B_PORT, ENCODER_B_PIN) != FALSE) ? 1U : 0U;
}

static sint8 decodeDirection(uint8 previousState, uint8 currentState)
{
    uint8 transition =
        (uint8)((previousState << 2U) | currentState);

    switch (transition)
    {
    case 0x1U:
    case 0x7U:
    case 0xEU:
    case 0x8U:
        return -1;

    case 0x2U:
    case 0xBU:
    case 0xDU:
    case 0x4U:
        return 1;

    default:
        return 0;
    }
}

static void updateTimeBuffer(uint32 dt)
{
    g_encoderTimeFilter.dtSum -=
        g_encoderTimeFilter.dtBuffer[g_encoderTimeFilter.bufferIndex];
    g_encoderTimeFilter.dtBuffer[g_encoderTimeFilter.bufferIndex] = dt;
    g_encoderTimeFilter.dtSum += dt;

    g_encoderTimeFilter.bufferIndex++;
    if (g_encoderTimeFilter.bufferIndex >= ENCODER_TIME_BUFFER_SIZE)
    {
        g_encoderTimeFilter.bufferIndex = 0U;
    }

    if (g_encoderTimeFilter.bufferCount < ENCODER_TIME_BUFFER_SIZE)
    {
        g_encoderTimeFilter.bufferCount++;
    }
}

static void resetTimeBuffer(uint32 currentTick, sint8 direction)
{
    uint8 i;

    for (i = 0U; i < ENCODER_TIME_BUFFER_SIZE; i++)
    {
        g_encoderTimeFilter.dtBuffer[i] = 0U;
    }

    g_encoderTimeFilter.dtSum = 0U;
    g_encoderTimeFilter.previousTick = currentTick;
    g_encoderTimeFilter.lastEdgeTick = currentTick;
    g_encoderTimeFilter.bufferIndex = 0U;
    g_encoderTimeFilter.bufferCount = 0U;
    g_encoderTimeFilter.direction = direction;
}

static void handleEncoderEdge(void)
{
    uint8 currentState = readEncoderState();
    sint8 direction =
        decodeDirection(g_previousEncoderState, currentState);

    if (direction != 0)
    {
        uint32 currentTick = IfxStm_getLower(&MODULE_STM0);
        uint32 dt = currentTick - g_encoderTimeFilter.previousTick;

        if ((g_encoderTimeFilter.previousTick != 0U) &&
            (dt < ENCODER_MIN_EDGE_TICKS))
        {
            g_encoderRejectedEdgeCount++;
            return;
        }

        if ((g_encoderStableDirection != 0) &&
            (direction != g_encoderStableDirection))
        {
            g_encoderReverseCandidateCount++;

            if (g_encoderReverseCandidateCount <
                ENCODER_DIRECTION_CONFIRM_COUNT)
            {
                g_encoderRejectedEdgeCount++;
                g_previousEncoderState = currentState;
                g_encoderState = currentState;
                return;
            }

            g_encoderStableDirection = direction;
            g_encoderReverseCandidateCount = 0U;
            g_encoderCount += direction;
            resetTimeBuffer(currentTick, direction);
            g_previousEncoderState = currentState;
            g_encoderState = currentState;
            return;
        }

        g_encoderStableDirection = direction;
        g_encoderReverseCandidateCount = 0U;

        if (g_encoderTimeFilter.previousTick != 0U)
        {
            updateTimeBuffer(dt);
        }

        g_encoderCount += direction;
        g_encoderTimeFilter.direction = direction;
        g_encoderTimeFilter.previousTick = currentTick;
        g_encoderTimeFilter.lastEdgeTick = currentTick;
    }

    g_previousEncoderState = currentState;
    g_encoderState = currentState;
}

IFX_INTERRUPT(isrEncoderA, 0, ISR_PRIORITY_ENCODER_A);

void isrEncoderA(void)
{
    IfxScuEru_clearEventFlag(IfxScuEru_InputChannel_0);
    g_encoderIsrACount++;
    handleEncoderEdge();
}

IFX_INTERRUPT(isrEncoderB, 0, ISR_PRIORITY_ENCODER_B);

void isrEncoderB(void)
{
    IfxScuEru_clearEventFlag(IfxScuEru_InputChannel_4);
    g_encoderIsrBCount++;
    handleEncoderEdge();
}

static void initEncoderInput(IfxScu_Req_In *requestPin,
                             IfxScuEru_InputChannel inputChannel,
                             IfxScuEru_OutputChannel outputChannel,
                             IfxScuEru_InputNodePointer nodePointer)
{
    IfxScuEru_initReqPin(requestPin, IfxPort_InputMode_pullUp);
    IfxScuEru_selectExternalInput(
        inputChannel, IfxScuEru_ExternalInputSelection_0);
    IfxScuEru_connectTrigger(inputChannel, nodePointer);
    IfxScuEru_enableRisingEdgeDetection(inputChannel);
    IfxScuEru_enableFallingEdgeDetection(inputChannel);
    IfxScuEru_enableTriggerPulse(inputChannel);
    IfxScuEru_clearEventFlag(inputChannel);
    IfxScuEru_setInterruptGatingPattern(
        outputChannel,
        IfxScuEru_InterruptGatingPattern_alwaysActive);
}

static void initEncoder(void)
{
    g_previousEncoderState = readEncoderState();
    g_encoderState = g_previousEncoderState;

    initEncoderInput(&IfxScu_REQ0_P15_4_IN,
                     IfxScuEru_InputChannel_0,
                     IfxScuEru_OutputChannel_0,
                     IfxScuEru_InputNodePointer_0);

    initEncoderInput(&IfxScu_REQ8_P33_7_IN,
                     IfxScuEru_InputChannel_4,
                     IfxScuEru_OutputChannel_1,
                     IfxScuEru_InputNodePointer_1);

    IfxSrc_init(&SRC_SCUERU0, IfxSrc_Tos_cpu0,
                ISR_PRIORITY_ENCODER_A);
    IfxSrc_enable(&SRC_SCUERU0);

    IfxSrc_init(&SRC_SCUERU1, IfxSrc_Tos_cpu0,
                ISR_PRIORITY_ENCODER_B);
    IfxSrc_enable(&SRC_SCUERU1);

}

static void setDrivePwmPercent(float32 percent)
{
    uint32 dutyTicks;

    if (percent > 100.0f)
    {
        percent = 100.0f;
    }
    else if (percent < 0.0f)
    {
        percent = 0.0f;
    }

    dutyTicks =
        (uint32)((float32)g_drivePwmPeriodTicks *
                 percent / 100.0f);

    IfxGtm_Tom_Ch_setCompareOneShadow(g_drivePwm.tom,
                                      g_drivePwm.tomChannel,
                                      dutyTicks);
    IfxGtm_Tom_Tgc_trigger(g_drivePwm.tgc[0]);

    g_drivePwmPercent = percent;
}

static float32 absoluteFloat(float32 value)
{
    return (value < 0.0f) ? -value : value;
}

static sint32 calculateTargetWindowCount(float32 targetRpm)
{
    float32 targetCount;
    float32 accumulatedCount;
    sint32 windowCount;

    targetCount =
        (targetRpm * (float32)g_encoderCountsPerRev *
         CONTROL_PERIOD_SEC * (float32)DRIVE_RUN_COUNT_WINDOW_STEPS) /
        60.0f;

    accumulatedCount = g_driveAdaptiveTargetCountRemainder + targetCount;

    if (accumulatedCount >= 0.0f)
    {
        windowCount = (sint32)accumulatedCount;
    }
    else
    {
        windowCount = -(sint32)(-accumulatedCount);
    }

    g_driveAdaptiveTargetCountRemainder =
        accumulatedCount - (float32)windowCount;

    return windowCount;
}

static void updateRpmMotor(void)
{
    float32 targetSignedRpm;
    float32 pwmCommand;
    float32 pwmPercent;
    float32 startHoldPwm;
    sint32 posErr;
    sint32 targetDeltaCount;
    sint32 deltaError;
    boolean desiredReverse;

    if (g_targetRpm > DRIVE_RPM_MAX)
    {
        g_targetRpm = DRIVE_RPM_MAX;
    }
    else if (g_targetRpm < 0.0f)
    {
        g_targetRpm = 0.0f;
    }

    g_driveTargetRpm = g_targetRpm;

    if ((g_targetRpm != g_previousTargetRpm) ||
        (g_targetReverse != g_previousTargetReverse))
    {
        if ((g_previousTargetRpm > 0.0f) && (g_targetRpm <= 0.0f))
        {
            g_posHoldActive = (g_stopHoldEnable != FALSE) ? TRUE : FALSE;
            g_stopState = (g_stopHoldEnable != FALSE) ?
                          DRIVE_STOP_STATE_HOLD : DRIVE_STOP_STATE_NONE;
            g_stopStepCount = 0U;
            g_posHoldCount = g_encoderCount;
            g_driveHoldPwmPercent = (g_stopHoldEnable != FALSE) ?
                                    g_driveAdaptivePwmCommand : 0.0f;
            g_driveHoldPosError = 0;
            g_driveHoldDirection = 0;
            g_driveAdaptivePwmCommand = 0.0f;
            g_driveAdaptiveTargetCountRemainder = 0.0f;
            g_driveAdaptiveCountSum = 0;
            g_driveAdaptiveWindowStepCount = 0U;
            g_stopBrakeReverse =
                (g_previousTargetReverse == FALSE) ? TRUE : FALSE;
            g_driveStopBrakePwmPercent = 0.0f;
            g_flatStopBrakeReverse =
                (g_previousTargetReverse == FALSE) ? TRUE : FALSE;
            g_flatStopBrakeStepCount =
                (g_stopHoldEnable == FALSE) ?
                DRIVE_FLAT_STOP_BRAKE_STEPS : 0U;
        }
        else if (g_targetRpm > 0.0f)
        {
            startHoldPwm = g_driveHoldPwmPercent;

            g_posHoldActive = FALSE;
            g_stopState = DRIVE_STOP_STATE_NONE;
            g_stopStepCount = 0U;
            g_driveHoldPwmPercent = 0.0f;
            g_driveHoldPosError = 0;
            g_driveHoldDirection = 0;
            g_driveAdaptiveTargetCountRemainder = 0.0f;
            g_driveAdaptiveCountSum = 0;
            g_driveAdaptiveWindowStepCount = 0U;
            g_flatStopBrakeStepCount = 0U;

            if (g_previousTargetRpm <= 0.0f)
            {
                if (((g_targetReverse == FALSE) && (startHoldPwm > 0.0f)) ||
                    ((g_targetReverse != FALSE) && (startHoldPwm < 0.0f)))
                {
                    g_driveAdaptivePwmCommand = startHoldPwm;
                }
                else
                {
                    g_driveAdaptivePwmCommand = 0.0f;
                }
            }
        }

        g_previousTargetRpm = g_targetRpm;
        g_previousTargetReverse = g_targetReverse;
    }

    if (g_enable == FALSE)
    {
        g_posHoldActive = FALSE;
        g_stopHoldEnable = TRUE;
        g_stopState = DRIVE_STOP_STATE_NONE;
        g_stopStepCount = 0U;
        g_driveHoldPwmPercent = 0.0f;
        g_driveHoldPosError = 0;
        g_driveHoldDirection = 0;
        g_driveAdaptivePwmCommand = 0.0f;
        g_driveAdaptiveTargetCountRemainder = 0.0f;
        g_driveAdaptiveCountSum = 0;
        g_driveAdaptiveWindowStepCount = 0U;
        g_flatStopBrakeStepCount = 0U;
        g_driveTargetPwmPercent = 0.0f;
        setDrivePwmPercent(0.0f);
        return;
    }

    targetSignedRpm = (g_targetReverse != FALSE) ?
                      -g_targetRpm : g_targetRpm;

    IfxPort_setPinLow(&DRIVE_BRK_PORT, DRIVE_BRK_PIN);

    pwmCommand = g_driveAdaptivePwmCommand;

    if (targetSignedRpm == 0.0f)
    {
        if (g_stopHoldEnable == FALSE)
        {
            g_posHoldActive = FALSE;
            g_stopState = DRIVE_STOP_STATE_NONE;
            g_stopStepCount = 0U;
            g_driveHoldPwmPercent = 0.0f;
            g_driveHoldPosError = 0;
            g_driveHoldDirection = 0;
            g_driveTargetPwmPercent = 0.0f;
            g_driveAdaptivePwmCommand = 0.0f;

            if (g_flatStopBrakeStepCount > 0U)
            {
                g_flatStopBrakeStepCount--;
                g_driveTargetPwmPercent =
                    DRIVE_FLAT_STOP_BRAKE_PWM_PERCENT;

                if (g_flatStopBrakeReverse != FALSE)
                {
                    IfxPort_setPinLow(&DRIVE_DIR_PORT, DRIVE_DIR_PIN);
                    g_driveReverseActive = TRUE;
                }
                else
                {
                    IfxPort_setPinHigh(&DRIVE_DIR_PORT, DRIVE_DIR_PIN);
                    g_driveReverseActive = FALSE;
                }

                setDrivePwmPercent(DRIVE_FLAT_STOP_BRAKE_PWM_PERCENT);
                return;
            }

            setDrivePwmPercent(0.0f);
            return;
        }

        if (g_stopState == DRIVE_STOP_STATE_BRAKE)
        {
            g_stopStepCount++;
            g_driveTargetPwmPercent = g_driveStopBrakePwmPercent;

            if (g_stopBrakeReverse != FALSE)
            {
                IfxPort_setPinLow(&DRIVE_DIR_PORT, DRIVE_DIR_PIN);
                g_driveReverseActive = TRUE;
            }
            else
            {
                IfxPort_setPinHigh(&DRIVE_DIR_PORT, DRIVE_DIR_PIN);
                g_driveReverseActive = FALSE;
            }

            setDrivePwmPercent(g_driveStopBrakePwmPercent);

            if (g_stopStepCount >= DRIVE_STOP_BRAKE_STEPS)
            {
                g_stopState = DRIVE_STOP_STATE_SETTLE;
                g_stopStepCount = 0U;
            }

            return;
        }

        if (g_stopState == DRIVE_STOP_STATE_SETTLE)
        {
            g_stopStepCount++;
            g_driveTargetPwmPercent = 0.0f;
            setDrivePwmPercent(0.0f);

            if (g_stopStepCount >= DRIVE_STOP_SETTLE_STEPS)
            {
                g_stopState = DRIVE_STOP_STATE_HOLD;
                g_stopStepCount = 0U;
                g_posHoldActive = TRUE;
                g_posHoldCount = g_encoderCount;
                g_driveHoldPwmPercent = 0.0f;
                g_driveHoldPosError = 0;
                g_driveHoldDirection = 0;
            }

            return;
        }

        if (g_posHoldActive == FALSE)
        {
            g_stopState = DRIVE_STOP_STATE_HOLD;
            g_posHoldActive = TRUE;
            g_posHoldCount = g_encoderCount;
            g_driveHoldPwmPercent = 0.0f;
            g_driveHoldPosError = 0;
            g_driveHoldDirection = 0;
        }

        posErr = g_posHoldCount - g_encoderCount;
        g_driveHoldPosError = posErr;

        if (g_encoderDeltaCount > DRIVE_HOLD_DELTA_DEADBAND_COUNT)
        {
            g_driveHoldPwmPercent -= DRIVE_HOLD_ADAPT_STEP_PERCENT;
        }
        else if (g_encoderDeltaCount < -DRIVE_HOLD_DELTA_DEADBAND_COUNT)
        {
            g_driveHoldPwmPercent += DRIVE_HOLD_ADAPT_STEP_PERCENT;
        }

        if (g_driveHoldPwmPercent > DRIVE_HOLD_ADAPT_MAX_PERCENT)
        {
            g_driveHoldPwmPercent = DRIVE_HOLD_ADAPT_MAX_PERCENT;
        }
        else if (g_driveHoldPwmPercent < -DRIVE_HOLD_ADAPT_MAX_PERCENT)
        {
            g_driveHoldPwmPercent = -DRIVE_HOLD_ADAPT_MAX_PERCENT;
        }

        if (g_driveHoldPwmPercent > 0.0f)
        {
            g_driveHoldDirection = 1;
        }
        else if (g_driveHoldPwmPercent < 0.0f)
        {
            g_driveHoldDirection = -1;
        }
        else
        {
            g_driveHoldDirection = 0;
        }

        pwmCommand = g_driveHoldPwmPercent;
    }
    else
    {
        g_driveAdaptiveCountSum += g_encoderDeltaCount;
        g_driveAdaptiveWindowStepCount++;

        if (g_driveAdaptiveWindowStepCount >= DRIVE_RUN_COUNT_WINDOW_STEPS)
        {
            targetDeltaCount = calculateTargetWindowCount(targetSignedRpm);
            deltaError = targetDeltaCount - g_driveAdaptiveCountSum;

            if (deltaError > DRIVE_RUN_DELTA_DEADBAND_COUNT)
            {
                g_driveAdaptivePwmCommand += DRIVE_RUN_ADAPT_STEP_PERCENT;
            }
            else if (deltaError < -DRIVE_RUN_DELTA_DEADBAND_COUNT)
            {
                g_driveAdaptivePwmCommand -= DRIVE_RUN_ADAPT_STEP_PERCENT;
            }

            if (g_driveAdaptivePwmCommand > 100.0f)
            {
                g_driveAdaptivePwmCommand = 100.0f;
            }
            else if (g_driveAdaptivePwmCommand < -100.0f)
            {
                g_driveAdaptivePwmCommand = -100.0f;
            }

            g_driveAdaptiveCountSum = 0;
            g_driveAdaptiveWindowStepCount = 0U;
        }

        pwmCommand = g_driveAdaptivePwmCommand;
    }

    desiredReverse = (pwmCommand < 0.0f) ? TRUE : FALSE;

    if (desiredReverse != g_driveReverseActive)
    {
        g_driveTargetPwmPercent = 0.0f;
    }

    if (desiredReverse != FALSE)
    {
        IfxPort_setPinLow(&DRIVE_DIR_PORT, DRIVE_DIR_PIN);
        g_driveReverseActive = TRUE;
        pwmPercent = -pwmCommand;
    }
    else
    {
        IfxPort_setPinHigh(&DRIVE_DIR_PORT, DRIVE_DIR_PIN);
        g_driveReverseActive = FALSE;
        pwmPercent = pwmCommand;
    }

    if (pwmPercent < (g_driveTargetPwmPercent - DRIVE_PWM_FALL_LIMIT_PERCENT))
    {
        pwmPercent = g_driveTargetPwmPercent - DRIVE_PWM_FALL_LIMIT_PERCENT;
    }

    g_driveTargetPwmPercent = pwmPercent;
    setDrivePwmPercent(pwmPercent);
}

static void updateOpenLoopMotor(void)
{
    float32 actualRpm;

    g_driveTargetRpm = g_openLoopPwmPercent;

    if ((g_enable == FALSE) || (g_openLoopPwmPercent <= 0.0f))
    {
        g_driveTargetPwmPercent = 0.0f;
        setDrivePwmPercent(0.0f);
        return;
    }

    actualRpm = absoluteFloat(g_encoderActualRpm);

    if (g_openLoopReverse != g_driveReverseActive)
    {
        setDrivePwmPercent(0.0f);

        if (actualRpm > DRIVE_REVERSE_RPM_THRESHOLD)
        {
            g_driveTargetPwmPercent = 0.0f;
            return;
        }

        g_driveReverseActive = g_openLoopReverse;
    }

    if (g_driveReverseActive != FALSE)
    {
        IfxPort_setPinLow(&DRIVE_DIR_PORT, DRIVE_DIR_PIN);
    }
    else
    {
        IfxPort_setPinHigh(&DRIVE_DIR_PORT, DRIVE_DIR_PIN);
    }

    IfxPort_setPinLow(&DRIVE_BRK_PORT, DRIVE_BRK_PIN);
    g_driveTargetPwmPercent = g_openLoopPwmPercent;
    setDrivePwmPercent(g_openLoopPwmPercent);
}

static void initDriveMotor(void)
{
    boolean interruptState;
    IfxGtm_Tom_Pwm_Config pwmConfig;

    interruptState = IfxCpu_disableInterrupts();

    IfxGtm_enable(&MODULE_GTM);
    IfxGtm_Cmu_setGclkFrequency(&MODULE_GTM, 100000000.0f);
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_FXCLK);

    IfxPort_setPinModeOutput(&DRIVE_DIR_PORT, DRIVE_DIR_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(&DRIVE_BRK_PORT, DRIVE_BRK_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinLow(&DRIVE_DIR_PORT, DRIVE_DIR_PIN);
    IfxPort_setPinLow(&DRIVE_BRK_PORT, DRIVE_BRK_PIN);

    g_drivePwmPeriodTicks = (uint32)(100000000.0f / DRIVE_PWM_FREQ_HZ);

    IfxGtm_Tom_Pwm_initConfig(&pwmConfig, &MODULE_GTM);
    pwmConfig.tom = DRIVE_PWM_OUT.tom;
    pwmConfig.tomChannel = DRIVE_PWM_OUT.channel;
    pwmConfig.clock = IfxGtm_Tom_Ch_ClkSrc_cmuFxclk0;
    pwmConfig.period = g_drivePwmPeriodTicks;
    pwmConfig.dutyCycle = 0U;
    pwmConfig.pin.outputPin = &DRIVE_PWM_OUT;
    pwmConfig.pin.outputMode = IfxPort_OutputMode_pushPull;
    pwmConfig.pin.padDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    pwmConfig.synchronousUpdateEnabled = TRUE;

    IfxGtm_Tom_Pwm_init(&g_drivePwm, &pwmConfig);
    IfxGtm_Tom_Pwm_start(&g_drivePwm, TRUE);
    setDrivePwmPercent(0.0f);

    IfxCpu_restoreInterrupts(interruptState);
}

void Encoder_Motor_Init(void)
{
    g_targetRpm = 0.0f;
    g_previousTargetRpm = 0.0f;
    g_targetReverse = FALSE;
    g_previousTargetReverse = FALSE;
    g_openLoopPwmPercent = 0.0f;
    g_openLoopReverse = FALSE;
    g_enable = FALSE;
    g_stopHoldEnable = TRUE;
    g_flatStopBrakeStepCount = 0U;
    initEncoder();
    initDriveMotor();
}

void Encoder_Motor_SetEncoderCountsPerRev(uint32 countsPerRev)
{
    g_encoderCountsPerRev = countsPerRev;
}

void Encoder_Motor_AdvanceRpmBufferSize(void)
{
    if (g_encoderActiveBufferSize < ENCODER_TIME_BUFFER_SIZE)
    {
        g_encoderActiveBufferSize += 4U;
    }
}

void Encoder_Motor_ReduceRpmBufferSize(void)
{
    if (g_encoderActiveBufferSize > 4U)
    {
        g_encoderActiveBufferSize -= 4U;
    }
}

void Encoder_Motor_SetTargetRpm(float32 targetRpm, boolean reverse)
{
    g_targetRpm = targetRpm;
    g_targetReverse = reverse;
}

void Encoder_Motor_SetStopHoldEnable(boolean enable)
{
    g_stopHoldEnable = enable;
}

void Encoder_Motor_SetOpenLoopCommand(float32 pwmPercent,
                                      boolean reverse)
{
    g_openLoopPwmPercent = pwmPercent;
    g_openLoopReverse = reverse;
}

void Encoder_Motor_SetEnable(boolean enable)
{
    if ((enable != FALSE) && (g_enable == FALSE))
    {
        uint32 currentTick = IfxStm_getLower(&MODULE_STM0);

        g_previousEncoderState = readEncoderState();
        g_encoderState = g_previousEncoderState;
        g_encoderStableDirection = 0;
        g_encoderReverseCandidateCount = 0U;
        g_encoderActualRpm = 0.0f;
        g_encoderRawTimeRpm = 0.0f;
        g_encoderAverageEdgeTicks = 0U;
        resetTimeBuffer(currentTick, 1);
    }

    g_enable = enable;
}

float32 Encoder_Motor_GetActualRpm(void)
{
    return g_encoderActualRpm;
}

static float32 calculateDeltaRpm(sint32 deltaCount)
{
    if (g_encoderCountsPerRev == 0U)
    {
        return 0.0f;
    }

    return ((float32)deltaCount * 60.0f) /
           ((float32)g_encoderCountsPerRev * CONTROL_PERIOD_SEC);
}

static float32 calculateCountWindowRpm(sint32 deltaCount)
{
    g_encoderWindowCountSum += deltaCount;
    g_encoderWindowStepCount++;

    if (g_encoderWindowStepCount < COUNT_RPM_WINDOW_STEPS)
    {
        return g_encoderActualRpm;
    }

    g_encoderWindowDeltaCount = g_encoderWindowCountSum;
    g_encoderWindowCountSum = 0;
    g_encoderWindowStepCount = 0U;

    if (g_encoderCountsPerRev == 0U)
    {
        return 0.0f;
    }

    return ((float32)g_encoderWindowDeltaCount * 60.0f) /
           ((float32)g_encoderCountsPerRev *
            CONTROL_PERIOD_SEC *
            (float32)COUNT_RPM_WINDOW_STEPS);
}

static float32 calculateEncoderRpm(void)
{
    boolean interruptState;
    uint8 i;
    uint8 sampleIndex;
    uint8 activeBufferSize;
    uint32 dtSum;
    uint32 lastEdgeTick;
    uint32 currentTick;
    uint8 bufferCount;
    sint8 direction;
    float32 stmFrequency;
    float32 rpm;

    interruptState = IfxCpu_disableInterrupts();
    activeBufferSize = g_encoderActiveBufferSize;
    bufferCount = g_encoderTimeFilter.bufferCount;

    if (activeBufferSize > bufferCount)
    {
        activeBufferSize = bufferCount;
    }

    dtSum = 0U;
    sampleIndex = g_encoderTimeFilter.bufferIndex;
    for (i = 0U; i < activeBufferSize; i++)
    {
        if (sampleIndex == 0U)
        {
            sampleIndex = ENCODER_TIME_BUFFER_SIZE - 1U;
        }
        else
        {
            sampleIndex--;
        }

        dtSum += g_encoderTimeFilter.dtBuffer[sampleIndex];
    }

    lastEdgeTick = g_encoderTimeFilter.lastEdgeTick;
    direction = g_encoderTimeFilter.direction;
    IfxCpu_restoreInterrupts(interruptState);

    stmFrequency = IfxStm_getFrequency(&MODULE_STM0);
    g_encoderStmFrequency = stmFrequency;
    currentTick = IfxStm_getLower(&MODULE_STM0);

    if (g_encoderCountsPerRev == 0U)
    {
        g_encoderAverageEdgeTicks = 0U;
        return 0.0f;
    }

    if ((lastEdgeTick == 0U) ||
        ((currentTick - lastEdgeTick) >
         (uint32)(stmFrequency * ENCODER_STOP_TIMEOUT_SEC)))
    {
        g_encoderAverageEdgeTicks = 0U;
        return 0.0f;
    }

    if ((activeBufferSize < ENCODER_MIN_BUFFER_COUNT) ||
        (dtSum == 0U))
    {
        return g_encoderActualRpm;
    }

    g_encoderAverageEdgeTicks = dtSum / (uint32)activeBufferSize;
    rpm = ((float32)activeBufferSize * 60.0f * stmFrequency) /
          ((float32)g_encoderCountsPerRev * (float32)dtSum);

    return rpm * (float32)direction;
}

void Encoder_Motor_Update_5ms(void)
{
    boolean interruptState;
    sint32 currentCount;
    float32 testPwmPercent;

    interruptState = IfxCpu_disableInterrupts();
    currentCount = g_encoderCount;
    IfxCpu_restoreInterrupts(interruptState);

    updateEncoderPollPins();
    g_encoderUpdateCount++;
    g_encoderDeltaCount = currentCount - g_previousEncoderCount;
    g_previousEncoderCount = currentCount;
    g_encoderDeltaRpm = calculateDeltaRpm(g_encoderDeltaCount);

    /* Actual RPM comes from the 200ms count window.
     * 300 counts/rev: 1 count / 200ms = 1 rpm. */
    g_encoderActualRpm = calculateCountWindowRpm(g_encoderDeltaCount);
    g_encoderRawTimeRpm = calculateEncoderRpm();

    if (g_driveTestEnable != FALSE)
    {
        g_driveTargetRpm = 0.0f;
        testPwmPercent = g_driveTestPwmPercent;
        g_driveTargetPwmPercent = testPwmPercent;

        IfxPort_setPinLow(&DRIVE_DIR_PORT, DRIVE_DIR_PIN);
        IfxPort_setPinLow(&DRIVE_BRK_PORT, DRIVE_BRK_PIN);
        setDrivePwmPercent(testPwmPercent);
    }
    else
    {
        updateRpmMotor();
    }
}
