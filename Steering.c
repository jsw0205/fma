#include "Steering.h"

#include "IfxCpu.h"
#include "IfxGtm.h"
#include "IfxGtm_Cmu.h"
#include "IfxGtm_Tom_Timer.h"
#include "IfxPort.h"
#include "IfxVadc_Adc.h"

#define POT_ADC_CHANNEL_ID  IfxVadc_ChannelId_7
#define POT_ADC_GROUP_ID    IfxVadc_GroupId_4

#define STEER_DIR_PORT  MODULE_P00
#define STEER_DIR_PIN   1
#define STEER_BRK_PORT  MODULE_P00
#define STEER_BRK_PIN   2
#define STEER_PWM_OUT   IfxGtm_TOM1_0_TOUT26_P33_4_OUT

#define STEER_PWM_FREQ_HZ  20000.0f
#define MIN_PWM_PERCENT    3.0f
#define MAX_PWM_POS_PERCENT 85.0f
#define MAX_PWM_NEG_PERCENT 85.0f
#define CONTROL_DT_SEC     0.005f

static IfxVadc_Adc g_vadc;
static IfxVadc_Adc_Group g_adcGroup;
static IfxVadc_Adc_Channel g_adcChannel;

static IfxGtm_Tom_Timer g_steerTimer;
static uint32 g_steerPwmPeriodTicks = 0U;

static float32 g_targetAngle = 0.0f;
static boolean g_enable = FALSE;
static float32 g_integral = 0.0f;
static float32 g_previousError = 0.0f;

volatile uint32 g_steeringPotValue = POT_CENTER;
volatile sint32 g_steeringTargetPot = POT_CENTER;
volatile float32 g_steeringCurrentAngle = 0.0f;
volatile float32 g_steeringTargetAngle = 0.0f;
volatile float32 g_steeringPwmPercent = 0.0f;

static void initAdc(void)
{
    IfxVadc_Adc_Config adcConfig;
    IfxVadc_Adc_GroupConfig groupConfig;
    IfxVadc_Adc_ChannelConfig channelConfig;
    uint32 channels;

    IfxPort_setPinModeInput(&MODULE_P40, 9, IfxPort_Mode_inputNoPullDevice);

    IfxVadc_Adc_initModuleConfig(&adcConfig, &MODULE_VADC);
    IfxVadc_Adc_initModule(&g_vadc, &adcConfig);

    IfxVadc_Adc_initGroupConfig(&groupConfig, &g_vadc);
    groupConfig.groupId = POT_ADC_GROUP_ID;
    groupConfig.master = groupConfig.groupId;
    groupConfig.arbiter.requestSlotScanEnabled = TRUE;
    groupConfig.scanRequest.triggerConfig.gatingMode =
        IfxVadc_GatingMode_always;
    groupConfig.scanRequest.autoscanEnabled = FALSE;
    IfxVadc_Adc_initGroup(&g_adcGroup, &groupConfig);

    IfxVadc_Adc_initChannelConfig(&channelConfig, &g_adcGroup);
    channelConfig.channelId = POT_ADC_CHANNEL_ID;
    channelConfig.resultRegister = IfxVadc_ChannelResult_0;
    channelConfig.backgroundChannel = FALSE;
    IfxVadc_Adc_initChannel(&g_adcChannel, &channelConfig);

    channels = (1U << POT_ADC_CHANNEL_ID);
    IfxVadc_Adc_setScan(&g_adcGroup, channels, channels);
}

static void initPwm(void)
{
    boolean interruptState;
    IfxGtm_Tom_Timer_Config timerConfig;

    interruptState = IfxCpu_disableInterrupts();

    IfxGtm_enable(&MODULE_GTM);
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_FXCLK);

    IfxPort_setPinModeOutput(&STEER_DIR_PORT, STEER_DIR_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(&STEER_BRK_PORT, STEER_BRK_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinLow(&STEER_DIR_PORT, STEER_DIR_PIN);
    IfxPort_setPinHigh(&STEER_BRK_PORT, STEER_BRK_PIN);

    IfxGtm_Tom_Timer_initConfig(&timerConfig, &MODULE_GTM);
    timerConfig.base.frequency = STEER_PWM_FREQ_HZ;
    timerConfig.base.isrPriority = 0U;
    timerConfig.base.isrProvider = IfxSrc_Tos_cpu0;
    timerConfig.base.minResolution =
        (1.0f / timerConfig.base.frequency) / 1000.0f;
    timerConfig.tom = STEER_PWM_OUT.tom;
    timerConfig.timerChannel = STEER_PWM_OUT.channel;
    timerConfig.clock = IfxGtm_Tom_Ch_ClkSrc_cmuFxclk1;
    timerConfig.triggerOut = &STEER_PWM_OUT;
    timerConfig.base.trigger.enabled = TRUE;
    timerConfig.base.trigger.outputEnabled = TRUE;
    timerConfig.base.trigger.risingEdgeAtPeriod = TRUE;
    timerConfig.base.trigger.triggerPoint = 0U;

    IfxGtm_Tom_Timer_init(&g_steerTimer, &timerConfig);
    IfxGtm_Tom_Timer_run(&g_steerTimer);
    g_steerPwmPeriodTicks =
        (uint32)IfxGtm_Tom_Timer_getPeriod(&g_steerTimer);

    IfxCpu_restoreInterrupts(interruptState);
}

static sint32 angleToPot(float32 angle)
{
    if (angle > STEER_MAX_ANGLE)
    {
        angle = STEER_MAX_ANGLE;
    }
    else if (angle < -STEER_MAX_ANGLE)
    {
        angle = -STEER_MAX_ANGLE;
    }

    if (angle >= 0.0f)
    {
        return POT_CENTER +
               (sint32)(((float32)(POT_LEFT - POT_CENTER) * angle) /
                        STEER_MAX_ANGLE);
    }

    return POT_CENTER +
           (sint32)(((float32)(POT_CENTER - POT_RIGHT) * angle) /
                    STEER_MAX_ANGLE);
}

static float32 potToAngle(sint32 pot)
{
    if (pot >= POT_CENTER)
    {
        return ((float32)(pot - POT_CENTER) * STEER_MAX_ANGLE) /
               (float32)(POT_LEFT - POT_CENTER);
    }

    return ((float32)(pot - POT_CENTER) * STEER_MAX_ANGLE) /
           (float32)(POT_CENTER - POT_RIGHT);
}

static float32 calculatePwm(sint32 currentPot, sint32 targetPot)
{
    const float32 kp = 0.15f;
    const float32 ki = 0.001f;
    const float32 kd = 0.0f;
    const float32 integralLimit = 120.0f;
    float32 error = (float32)(targetPot - currentPot);
    float32 derivative;
    float32 pwm;

    if ((error <= (float32)POT_TOLERANCE) &&
        (error >= -(float32)POT_TOLERANCE))
    {
        g_integral = 0.0f;
        g_previousError = error;
        return 0.0f;
    }

    g_integral += error * CONTROL_DT_SEC;

    if (g_integral > integralLimit)
    {
        g_integral = integralLimit;
    }
    else if (g_integral < -integralLimit)
    {
        g_integral = -integralLimit;
    }

    derivative = (error - g_previousError) / CONTROL_DT_SEC;
    g_previousError = error;

    pwm = (kp * error) + (ki * g_integral) + (kd * derivative);

    if (pwm > MAX_PWM_POS_PERCENT)
    {
        pwm = MAX_PWM_POS_PERCENT;
    }
    else if (pwm < -MAX_PWM_NEG_PERCENT)
    {
        pwm = -MAX_PWM_NEG_PERCENT;
    }

    if ((pwm > 0.0f) && (pwm < MIN_PWM_PERCENT))
    {
        pwm = MIN_PWM_PERCENT;
    }
    else if ((pwm < 0.0f) && (pwm > -MIN_PWM_PERCENT))
    {
        pwm = -MIN_PWM_PERCENT;
    }

    return pwm;
}

static void setMotorPercent(sint32 percent)
{
    uint32 triggerPoint;
    uint32 absolutePercent;

    if (percent > 100)
    {
        percent = 100;
    }
    else if (percent < -100)
    {
        percent = -100;
    }

    IfxPort_setPinLow(&STEER_BRK_PORT, STEER_BRK_PIN);

    if (percent >= 0)
    {
        IfxPort_setPinLow(&STEER_DIR_PORT, STEER_DIR_PIN);
        absolutePercent = (uint32)percent;
    }
    else
    {
        IfxPort_setPinHigh(&STEER_DIR_PORT, STEER_DIR_PIN);
        absolutePercent = (uint32)(-percent);
    }

    triggerPoint =
        (g_steerPwmPeriodTicks * absolutePercent) / 100U;

    IfxGtm_Tom_Timer_disableUpdate(&g_steerTimer);
    IfxGtm_Tom_Timer_setTrigger(&g_steerTimer, triggerPoint);
    IfxGtm_Tom_Timer_applyUpdate(&g_steerTimer);

    g_steeringPwmPercent = (float32)percent;
}

void Steering_Init(void)
{
    initAdc();
    initPwm();
}

void Steering_SetTargetAngle(float32 targetAngle)
{
    g_targetAngle = targetAngle;
    g_steeringTargetAngle = targetAngle;
}

void Steering_SetEnable(boolean enable)
{
    g_enable = enable;
}

void Steering_Update_5ms(void)
{
    Ifx_VADC_RES result;
    sint32 currentPot;
    sint32 targetPot;
    float32 pwm;

    IfxVadc_Adc_startScan(&g_adcGroup);

    do
    {
        result = IfxVadc_Adc_getResult(&g_adcChannel);
    } while (result.B.VF == 0U);

    currentPot = (sint32)result.B.RESULT;
    g_steeringPotValue = (uint32)result.B.RESULT;

    if (currentPot < POT_RIGHT)
    {
        currentPot = POT_RIGHT;
    }
    else if (currentPot > POT_LEFT)
    {
        currentPot = POT_LEFT;
    }

    targetPot = angleToPot(g_targetAngle);
    g_steeringTargetPot = targetPot;
    g_steeringCurrentAngle = potToAngle(currentPot);

    if (g_enable == FALSE)
    {
        g_integral = 0.0f;
        g_previousError = 0.0f;
        setMotorPercent(0);
        return;
    }

    pwm = calculatePwm(currentPot, targetPot);
    setMotorPercent((sint32)pwm);
}
