#include "PS2_Controller.h"

#include "IfxPort.h"
#include "IfxStm.h"

#define PS2_MISO_PORT  &MODULE_P10
#define PS2_MISO_PIN   1
#define PS2_MOSI_PORT  &MODULE_P10
#define PS2_MOSI_PIN   3
#define PS2_SCLK_PORT  &MODULE_P10
#define PS2_SCLK_PIN   2
#define PS2_CS_PORT    &MODULE_P10
#define PS2_CS_PIN     0

volatile uint8 g_ps2Rx[9];
volatile uint8 g_padId = 0U;
volatile uint8 g_btnLo = 0xFFU;
volatile uint8 g_btnHi = 0xFFU;

static void ps2SetAnalogMode(void);

static void ps2DelayUs(uint32 us)
{
    IfxStm_waitTicks(&MODULE_STM0,
                     IfxStm_getTicksFromMicroseconds(&MODULE_STM0, us));
}

void ps2PinsInit(void)
{
    IfxPort_setPinModeInput(PS2_MISO_PORT, PS2_MISO_PIN,
                            IfxPort_InputMode_pullUp);
    IfxPort_setPinModeOutput(PS2_MOSI_PORT, PS2_MOSI_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(PS2_SCLK_PORT, PS2_SCLK_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(PS2_CS_PORT, PS2_CS_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);

    IfxPort_setPinHigh(PS2_CS_PORT, PS2_CS_PIN);
    IfxPort_setPinHigh(PS2_SCLK_PORT, PS2_SCLK_PIN);
    IfxPort_setPinHigh(PS2_MOSI_PORT, PS2_MOSI_PIN);

    ps2SetAnalogMode();
}

static uint8 ps2TransferByte(uint8 tx)
{
    uint8 rx = 0U;
    sint32 i;

    for (i = 0; i < 8; i++)
    {
        if ((tx & (1U << i)) != 0U)
        {
            IfxPort_setPinHigh(PS2_MOSI_PORT, PS2_MOSI_PIN);
        }
        else
        {
            IfxPort_setPinLow(PS2_MOSI_PORT, PS2_MOSI_PIN);
        }

        ps2DelayUs(5U);
        IfxPort_setPinLow(PS2_SCLK_PORT, PS2_SCLK_PIN);
        ps2DelayUs(5U);

        if (IfxPort_getPinState(PS2_MISO_PORT, PS2_MISO_PIN) != FALSE)
        {
            rx |= (uint8)(1U << i);
        }

        IfxPort_setPinHigh(PS2_SCLK_PORT, PS2_SCLK_PIN);
        ps2DelayUs(5U);
    }

    return rx;
}

static void ps2Command(const uint8 *tx)
{
    sint32 i;

    IfxPort_setPinHigh(PS2_SCLK_PORT, PS2_SCLK_PIN);
    ps2DelayUs(10U);
    IfxPort_setPinLow(PS2_CS_PORT, PS2_CS_PIN);
    ps2DelayUs(20U);

    for (i = 0; i < 9; i++)
    {
        (void)ps2TransferByte(tx[i]);
    }

    ps2DelayUs(20U);
    IfxPort_setPinHigh(PS2_CS_PORT, PS2_CS_PIN);
    ps2DelayUs(1000U);
}

static void ps2SetAnalogMode(void)
{
    static const uint8 enterConfig[9] =
        {0x01U, 0x43U, 0x00U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    static const uint8 setAnalog[9] =
        {0x01U, 0x44U, 0x00U, 0x01U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U};
    static const uint8 exitConfig[9] =
        {0x01U, 0x43U, 0x00U, 0x00U, 0x5AU, 0x5AU, 0x5AU, 0x5AU, 0x5AU};

    ps2Command(enterConfig);
    ps2Command(setAnalog);
    ps2Command(exitConfig);
}

void ps2ConfigureAnalogMode(void)
{
    ps2SetAnalogMode();
}

void ps2ReadOnce(void)
{
    static const uint8 tx[9] =
        {0x01U, 0x42U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    sint32 i;

    IfxPort_setPinHigh(PS2_SCLK_PORT, PS2_SCLK_PIN);
    ps2DelayUs(10U);
    IfxPort_setPinLow(PS2_CS_PORT, PS2_CS_PIN);
    ps2DelayUs(20U);

    for (i = 0; i < 9; i++)
    {
        g_ps2Rx[i] = ps2TransferByte(tx[i]);
    }

    ps2DelayUs(20U);
    IfxPort_setPinHigh(PS2_CS_PORT, PS2_CS_PIN);
    ps2DelayUs(50U);

    g_padId = g_ps2Rx[1];
    g_btnLo = g_ps2Rx[3];
    g_btnHi = g_ps2Rx[4];
}
