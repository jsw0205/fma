#ifndef LAMP_CONTROL_H
#define LAMP_CONTROL_H

#include "IfxPort.h"
#include "PS2_Controller.h"
#include "Ps2_Ctrl.h"

#define BLUE_LED_PORT   (&MODULE_P02)
#define BLUE_LED_PIN    0U

#define RED_LED_PORT    (&MODULE_P02)
#define RED_LED_PIN     1U

#define REVERSE_LED_PORT   (&MODULE_P33)
#define REVERSE_LED_PIN    11U

#define REVERSE_LED_TOGGLE_COUNT_5MS  100U
#define FAULT_LED_ON_COUNT_5MS        3U

#define LED_ACTIVE_LOW  0

static uint8 g_faultLedCounter = 0U;
static uint16 g_reverseLedCounter = 0U;
static boolean g_reverseLedState = FALSE;
static boolean g_reverseLedWasActive = FALSE;

static void Lamp_Control_Write(Ifx_P *port, uint8 pin, boolean turnOn)
{
#if LED_ACTIVE_LOW
    if (turnOn != FALSE)
    {
        IfxPort_setPinLow(port, pin);
    }
    else
    {
        IfxPort_setPinHigh(port, pin);
    }
#else
    if (turnOn != FALSE)
    {
        IfxPort_setPinHigh(port, pin);
    }
    else
    {
        IfxPort_setPinLow(port, pin);
    }
#endif
}

static void Lamp_Control_Init(void)
{
    IfxPort_setPinModeOutput(BLUE_LED_PORT, BLUE_LED_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(RED_LED_PORT, RED_LED_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(REVERSE_LED_PORT, REVERSE_LED_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);

    Lamp_Control_Write(BLUE_LED_PORT, BLUE_LED_PIN, FALSE);
    Lamp_Control_Write(RED_LED_PORT, RED_LED_PIN, FALSE);
    Lamp_Control_Write(REVERSE_LED_PORT, REVERSE_LED_PIN, FALSE);

    g_faultLedCounter = 0U;
    g_reverseLedCounter = 0U;
    g_reverseLedState = FALSE;
    g_reverseLedWasActive = FALSE;
}

static void Lamp_Control_Update_5ms(void)
{
    boolean analogPad;
    boolean remoteOk;
    boolean reverseActive;

    analogPad = ((g_padId == 0x73U) || (g_padId == 0x79U));
    remoteOk = Ps2_Ctrl_GetEnable();
    reverseActive = ((remoteOk != FALSE) &&
                     (g_ps2ReverseMode != FALSE));

    if ((remoteOk != FALSE) && (analogPad != FALSE))
    {
        g_faultLedCounter = 0U;
        Lamp_Control_Write(BLUE_LED_PORT, BLUE_LED_PIN, TRUE);
        Lamp_Control_Write(RED_LED_PORT, RED_LED_PIN, FALSE);
    }
    else
    {
        Lamp_Control_Write(BLUE_LED_PORT, BLUE_LED_PIN, FALSE);

        if (g_faultLedCounter < FAULT_LED_ON_COUNT_5MS)
        {
            g_faultLedCounter++;
        }

        Lamp_Control_Write(
            RED_LED_PORT,
            RED_LED_PIN,
            (g_faultLedCounter >= FAULT_LED_ON_COUNT_5MS) ?
            TRUE : FALSE);
    }

    if (reverseActive == FALSE)
    {
        g_reverseLedCounter = 0U;
        g_reverseLedState = FALSE;
        g_reverseLedWasActive = FALSE;
        Lamp_Control_Write(REVERSE_LED_PORT, REVERSE_LED_PIN, FALSE);
        return;
    }

    if (g_reverseLedWasActive == FALSE)
    {
        g_reverseLedWasActive = TRUE;
        g_reverseLedCounter = 0U;
        g_reverseLedState = TRUE;
        Lamp_Control_Write(REVERSE_LED_PORT, REVERSE_LED_PIN, TRUE);
        return;
    }

    g_reverseLedCounter++;

    if (g_reverseLedCounter >= REVERSE_LED_TOGGLE_COUNT_5MS)
    {
        g_reverseLedCounter = 0U;
        g_reverseLedState =
            (g_reverseLedState == FALSE) ? TRUE : FALSE;
        Lamp_Control_Write(REVERSE_LED_PORT, REVERSE_LED_PIN,
                           g_reverseLedState);
    }
}

#endif /* LAMP_CONTROL_H */
