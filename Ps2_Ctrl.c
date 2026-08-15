#include "Ps2_Ctrl.h"

#include "Encoder_Motor.h"
#include "PS2_Controller.h"
#include "Steering.h"

#define JOYSTICK_DEADZONE  0.1f
#define PS2_READ_DIVIDER   1U
#define PS2_RECONNECT_READS 100U
#define PS2_INVALID_READ_LIMIT 40U
#define PS2_OFF_PATTERN_LIMIT 80U
#define DRIVE_MAX_RPM  200U
#define DRIVE_RPM_STEP 10U

static float32 g_targetRpm = 0.0f;
static float32 g_targetSteer = 0.0f;
static boolean g_enable = FALSE;
static uint8 g_readDivider = PS2_READ_DIVIDER - 1U;
static uint8 g_reconnectCount = 0U;
static uint8 g_lastPadId = 0xFFU;
static boolean g_triangleWasPressed = FALSE;
static boolean g_circleWasPressed = FALSE;
static boolean g_crossWasPressed = FALSE;
static boolean g_squareWasPressed = FALSE;
static boolean g_l2WasPressed = FALSE;
static uint8 g_startStableCnt = 0U;
static uint8 g_selectStableCnt = 0U;

volatile uint16 g_ps2TargetRpm = 0U;
volatile uint8 g_ps2InvalidReadCount = 0U;
volatile boolean g_ps2ReverseMode = FALSE;
volatile boolean g_ps2Connected = FALSE;
volatile boolean g_ps2AnalogMode = FALSE;
volatile boolean g_ps2AnalogOffPatternDetected = FALSE;
volatile uint16 g_ps2OffPatternCount = 0U;
volatile boolean g_ps2FlatStopMode = FALSE;
volatile boolean g_ps2CanControlMode = FALSE;

static boolean isAnalogOffPattern(void)
{
    return (boolean)(((g_padId == 0x73U) || (g_padId == 0x79U)) &&
                     (g_ps2Rx[2] == 0x5AU) &&
                     (g_ps2Rx[5] == 0x80U) &&
                     (g_ps2Rx[6] == 0x80U) &&
                     (g_ps2Rx[7] == 0x80U) &&
                     (g_ps2Rx[8] == 0x80U));
}

static void setPs2Disconnected(void)
{
    g_enable = FALSE;
    g_targetRpm = 0.0f;
    g_targetSteer = 0.0f;
    g_ps2TargetRpm = 0U;
    g_driveTestEnable = FALSE;
    g_driveTestPwmPercent = 0.0f;
    g_ps2ReverseMode = FALSE;
    g_ps2FlatStopMode = FALSE;
    g_ps2Connected = FALSE;
    g_ps2AnalogMode = FALSE;
}

static void updateDriveCommand(void)
{
    boolean trianglePressed =
        ((g_btnHi & (1U << 4U)) == 0U);
    boolean circlePressed =
        ((g_btnHi & (1U << 5U)) == 0U);
    boolean crossPressed =
        ((g_btnHi & (1U << 6U)) == 0U);
    boolean squarePressed =
        ((g_btnHi & (1U << 7U)) == 0U);
    boolean l2Pressed =
        ((g_btnHi & (1U << 0U)) == 0U);
    boolean selectPressed =
        ((g_btnLo & (1U << 0U)) == 0U);
    boolean startPressed =
        ((g_btnLo & (1U << 3U)) == 0U);

    if (startPressed != FALSE)
    {
        if (g_startStableCnt < 3U)
        {
            g_startStableCnt++;
        }
    }
    else
    {
        g_startStableCnt = 0U;
    }

    if (selectPressed != FALSE)
    {
        if (g_selectStableCnt < 3U)
        {
            g_selectStableCnt++;
        }
    }
    else
    {
        g_selectStableCnt = 0U;
    }

    if (g_startStableCnt == 3U)
    {
        g_ps2CanControlMode = TRUE;
        g_ps2TargetRpm = 0U;
        g_targetRpm = 0.0f;
        g_ps2ReverseMode = FALSE;
        g_ps2FlatStopMode = TRUE;
        g_startStableCnt = 4U;
    }

    if (g_selectStableCnt == 3U)
    {
        g_ps2CanControlMode = FALSE;
        g_ps2TargetRpm = 0U;
        g_targetRpm = 0.0f;
        g_ps2ReverseMode = FALSE;
        g_ps2FlatStopMode = TRUE;
        g_selectStableCnt = 4U;
    }

    if ((trianglePressed != FALSE) && (g_triangleWasPressed == FALSE))
    {
        g_ps2ReverseMode =
            (g_ps2ReverseMode == FALSE) ? TRUE : FALSE;
    }

    if ((circlePressed != FALSE) && (g_circleWasPressed == FALSE))
    {
        if (g_ps2TargetRpm <=
            (DRIVE_MAX_RPM - DRIVE_RPM_STEP))
        {
            g_ps2TargetRpm += DRIVE_RPM_STEP;
            g_ps2FlatStopMode = FALSE;
        }
    }

    if ((crossPressed != FALSE) && (g_crossWasPressed == FALSE))
    {
        if (g_ps2TargetRpm >= DRIVE_RPM_STEP)
        {
            g_ps2TargetRpm -= DRIVE_RPM_STEP;
            if (g_ps2TargetRpm > 0U)
            {
                g_ps2FlatStopMode = FALSE;
            }
        }
    }

    if ((squarePressed != FALSE) && (g_squareWasPressed == FALSE))
    {
        g_ps2TargetRpm = 0U;
        g_ps2FlatStopMode = TRUE;
    }

    if ((l2Pressed != FALSE) && (g_l2WasPressed == FALSE))
    {
        g_ps2TargetRpm = 0U;
        g_ps2FlatStopMode = FALSE;
    }

    g_driveTestEnable = FALSE;
    g_driveTestPwmPercent = 0.0f;

    g_triangleWasPressed = trianglePressed;
    g_circleWasPressed = circlePressed;
    g_crossWasPressed = crossPressed;
    g_squareWasPressed = squarePressed;
    g_l2WasPressed = l2Pressed;

    g_targetRpm = (float32)g_ps2TargetRpm;
}

void Ps2_Ctrl_Init(void)
{
    ps2PinsInit();
}

void Ps2_Ctrl_Update_5ms(void)
{
    float32 normalizedX;
    boolean analogPad;
    boolean validPad;

    g_readDivider++;
    if (g_readDivider < PS2_READ_DIVIDER)
    {
        return;
    }

    g_readDivider = 0U;
    ps2ReadOnce();

    analogPad = ((g_padId == 0x73U) || (g_padId == 0x79U));
    validPad = ((analogPad != FALSE) || (g_padId == 0x41U));
    if (validPad == FALSE)
    {
        if (g_ps2InvalidReadCount < PS2_INVALID_READ_LIMIT)
        {
            g_ps2InvalidReadCount++;
        }

        g_triangleWasPressed = FALSE;
        g_circleWasPressed = FALSE;
        g_crossWasPressed = FALSE;
        g_squareWasPressed = FALSE;
        g_l2WasPressed = FALSE;
        g_startStableCnt = 0U;
        g_selectStableCnt = 0U;
        g_ps2Connected = FALSE;
        g_ps2AnalogMode = FALSE;
        g_ps2OffPatternCount = 0U;
        g_ps2AnalogOffPatternDetected = FALSE;

        if ((g_padId == 0x41U) && (g_lastPadId != 0x41U))
        {
            g_reconnectCount = 0U;
            ps2ConfigureAnalogMode();
        }
        else
        {
            g_reconnectCount++;
            if (g_reconnectCount >= PS2_RECONNECT_READS)
            {
                g_reconnectCount = 0U;
                ps2ConfigureAnalogMode();
            }
        }

        g_lastPadId = g_padId;

        if (g_ps2InvalidReadCount >= PS2_INVALID_READ_LIMIT)
        {
            setPs2Disconnected();
        }

        return;
    }

    if (analogPad == FALSE)
    {
        g_ps2Connected = FALSE;
        g_ps2AnalogMode = FALSE;
        g_ps2OffPatternCount = 0U;
        g_ps2AnalogOffPatternDetected = FALSE;
    }
    else if (isAnalogOffPattern() != FALSE)
    {
        if (g_ps2OffPatternCount < PS2_OFF_PATTERN_LIMIT)
        {
            g_ps2OffPatternCount++;
        }

        if (g_ps2OffPatternCount >= PS2_OFF_PATTERN_LIMIT)
        {
            g_ps2AnalogOffPatternDetected = TRUE;
            g_ps2InvalidReadCount = 0U;
            g_reconnectCount = 0U;
            g_lastPadId = g_padId;
            setPs2Disconnected();
            return;
        }
    }
    else
    {
        g_ps2OffPatternCount = 0U;
        g_ps2AnalogOffPatternDetected = FALSE;
    }

    g_enable = TRUE;
    g_ps2Connected = TRUE;
    g_ps2AnalogMode = analogPad;
    g_ps2InvalidReadCount = 0U;
    g_reconnectCount = 0U;
    g_lastPadId = g_padId;
    updateDriveCommand();

    if (analogPad == FALSE)
    {
        g_targetSteer = 0.0f;
        ps2ConfigureAnalogMode();
        return;
    }

    normalizedX = ((float32)g_ps2Rx[7] - 128.0f) / 128.0f;

    if ((normalizedX > -JOYSTICK_DEADZONE) &&
        (normalizedX < JOYSTICK_DEADZONE))
    {
        normalizedX = 0.0f;
    }

    g_targetSteer = normalizedX * STEER_MAX_ANGLE;
}

float32 Ps2_Ctrl_GetTargetRpm(void)
{
    return g_targetRpm;
}

float32 Ps2_Ctrl_GetTargetSteer(void)
{
    return g_targetSteer;
}

boolean Ps2_Ctrl_GetEnable(void)
{
    return g_enable;
}
