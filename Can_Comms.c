#include "Can_Comms.h"

#include "Can_Status.h"
#include "IfxMultican_Can.h"
#include "IfxPort.h"
#include "Steering.h"

#define CAN_RX_OBJ_ID  3U
#define CAN_META_RX_OBJ_ID  5U
#define CAN_CMD_TIMEOUT_5MS  40U

static IfxMultican_Can g_can;
static IfxMultican_Can_Node g_canNode;
static IfxMultican_Can_MsgObj g_canTxObj1;
static IfxMultican_Can_MsgObj g_canTxObj2;
static IfxMultican_Can_MsgObj g_canTxObj3;
static IfxMultican_Can_MsgObj g_canTxObj4;
static IfxMultican_Can_MsgObj g_canRxObj;
static IfxMultican_Can_MsgObj g_canMetaRxObj;

/* last seq byte seen on CONTROL_META (0x203), echoed back in DIAG_STATUS.rx_seq_echo */
static uint8 g_canMetaSeqEcho = 0U;

volatile uint32 g_canTxCount = 0U;
volatile uint32 g_canTxBusyCount = 0U;
volatile uint32 g_canRxCount = 0U;
volatile uint32 g_canInitStatus = 0U;
volatile uint32 g_canNodeInitStatus = 0U;
volatile uint32 g_canMsgObjInitStatus = 0U;
volatile uint32 g_canLastSendStatus = 0U;
volatile uint8 g_canLastPayload[8] = {0U};
volatile sint16 g_canCmdTargetRpm = 0;
volatile sint16 g_canCmdSteerAngle = 0;
volatile boolean g_canCmdEnable = FALSE;
volatile boolean g_canCmdActive = FALSE;
volatile boolean g_canCmdSeen = FALSE;
volatile boolean g_canCmdFlatStopMode = TRUE;

static uint8 g_canCmdTimeoutCount = CAN_CMD_TIMEOUT_5MS;

static sint16 unpackS16(uint8 *data, uint8 index)
{
    return (sint16)((uint16)data[index] |
                    ((uint16)data[index + 1U] << 8U));
}

static void messageToBytes(IfxMultican_Message *message, uint8 *data)
{
    uint8 index;

    for (index = 0U; index < 4U; index++)
    {
        data[index] = (uint8)(message->data[0] >> (index * 8U));
        data[index + 4U] = (uint8)(message->data[1] >> (index * 8U));
    }
}

static void initCan(void)
{
    IfxMultican_Can_Config canConfig;
    IfxMultican_Can_NodeConfig nodeConfig;

    IfxPort_setPinModeOutput(&MODULE_P20,
                             6,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinLow(&MODULE_P20, 6);

    IfxMultican_Can_initModuleConfig(&canConfig, &MODULE_CAN);
    g_canInitStatus =
        (uint32)IfxMultican_Can_initModule(&g_can, &canConfig);

    IfxMultican_Can_Node_initConfig(&nodeConfig, &g_can);
    nodeConfig.nodeId = IfxMultican_NodeId_0;
    nodeConfig.baudrate = 500000U;
    nodeConfig.rxPin = &IfxMultican_RXD0B_P20_7_IN;
    nodeConfig.rxPinMode = IfxPort_InputMode_pullUp;
    nodeConfig.txPin = &IfxMultican_TXD0_P20_8_OUT;
    nodeConfig.txPinMode = IfxPort_OutputMode_pushPull;
    nodeConfig.pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    g_canNodeInitStatus =
        (uint32)IfxMultican_Can_Node_init(&g_canNode, &nodeConfig);
}

static void initCanTx(void)
{
    IfxMultican_Can_MsgObjConfig msgObjConfig;

    IfxMultican_Can_MsgObj_initConfig(&msgObjConfig, &g_canNode);
    msgObjConfig.msgObjId = 0U;
    msgObjConfig.messageId = 0x100U;
    msgObjConfig.frame = IfxMultican_Frame_transmit;
    msgObjConfig.control.messageLen = IfxMultican_DataLengthCode_8;
    IfxMultican_Can_MsgObj_init(&g_canTxObj1, &msgObjConfig);

    IfxMultican_Can_MsgObj_initConfig(&msgObjConfig, &g_canNode);
    msgObjConfig.msgObjId = 1U;
    msgObjConfig.messageId = 0x101U;
    msgObjConfig.frame = IfxMultican_Frame_transmit;
    msgObjConfig.control.messageLen = IfxMultican_DataLengthCode_8;
    IfxMultican_Can_MsgObj_init(&g_canTxObj2, &msgObjConfig);

    IfxMultican_Can_MsgObj_initConfig(&msgObjConfig, &g_canNode);
    msgObjConfig.msgObjId = 2U;
    msgObjConfig.messageId = CAN_DRIVE_STATUS_ID;
    msgObjConfig.frame = IfxMultican_Frame_transmit;
    msgObjConfig.control.messageLen = IfxMultican_DataLengthCode_8;
    g_canMsgObjInitStatus =
        (uint32)IfxMultican_Can_MsgObj_init(&g_canTxObj3, &msgObjConfig);

    IfxMultican_Can_MsgObj_initConfig(&msgObjConfig, &g_canNode);
    msgObjConfig.msgObjId = CAN_RX_OBJ_ID;
    msgObjConfig.messageId = CAN_COMMAND_ID;
    msgObjConfig.acceptanceMask = 0x7FFU;
    msgObjConfig.frame = IfxMultican_Frame_receive;
    msgObjConfig.control.messageLen = IfxMultican_DataLengthCode_8;
    msgObjConfig.control.extendedFrame = FALSE;
    msgObjConfig.control.matchingId = TRUE;
    IfxMultican_Can_MsgObj_init(&g_canRxObj, &msgObjConfig);

    IfxMultican_Can_MsgObj_initConfig(&msgObjConfig, &g_canNode);
    msgObjConfig.msgObjId = 4U;
    msgObjConfig.messageId = CAN_DIAG_STATUS_ID;
    msgObjConfig.frame = IfxMultican_Frame_transmit;
    msgObjConfig.control.messageLen = IfxMultican_DataLengthCode_8;
    IfxMultican_Can_MsgObj_init(&g_canTxObj4, &msgObjConfig);

    IfxMultican_Can_MsgObj_initConfig(&msgObjConfig, &g_canNode);
    msgObjConfig.msgObjId = CAN_META_RX_OBJ_ID;
    msgObjConfig.messageId = CAN_CONTROL_META_ID;
    msgObjConfig.acceptanceMask = 0x7FFU;
    msgObjConfig.frame = IfxMultican_Frame_receive;
    msgObjConfig.control.messageLen = IfxMultican_DataLengthCode_8;
    msgObjConfig.control.extendedFrame = FALSE;
    msgObjConfig.control.matchingId = TRUE;
    IfxMultican_Can_MsgObj_init(&g_canMetaRxObj, &msgObjConfig);
}

static void sendCanFrame(IfxMultican_Can_MsgObj *msgObj,
                         uint32 canId,
                         uint8 *data)
{
    IfxMultican_Message msg;

    IfxMultican_Message_init(&msg,
                             canId,
                             *(uint32 *)&data[0],
                             *(uint32 *)&data[4],
                             IfxMultican_DataLengthCode_8);

    IfxMultican_Can_MsgObj_sendMessage(msgObj, &msg);
}

void Can_Comms_Init(void)
{
    initCan();
    initCanTx();
}

void Can_Comms_Update_5ms(void)
{
    IfxMultican_Message msg;
    IfxMultican_Status status;
    uint8 data[8];

    IfxMultican_Message_init(&msg, CAN_COMMAND_ID, 0U, 0U,
                             IfxMultican_DataLengthCode_8);
    status = IfxMultican_Can_MsgObj_readMessage(&g_canRxObj, &msg);

    if ((status & IfxMultican_Status_newData) != 0U)
    {
        messageToBytes(&msg, data);

        g_canCmdTargetRpm = unpackS16(data, 0U);
        g_canCmdSteerAngle = unpackS16(data, 2U);
        g_canCmdEnable = (data[4] != 0U) ? TRUE : FALSE;

        if (data[5] == 1U)
        {
            g_canCmdFlatStopMode = TRUE;
        }
        else if (data[5] == 2U)
        {
            g_canCmdFlatStopMode = FALSE;
        }
        else if (g_canCmdTargetRpm != 0)
        {
            g_canCmdFlatStopMode = FALSE;
        }

        g_canCmdSeen = TRUE;
        g_canCmdActive = TRUE;
        g_canCmdTimeoutCount = 0U;
        g_canRxCount++;
        return;
    }

    if (g_canCmdSeen != FALSE)
    {
        if (g_canCmdTimeoutCount < CAN_CMD_TIMEOUT_5MS)
        {
            g_canCmdTimeoutCount++;
        }
        else
        {
            g_canCmdActive = FALSE;
            g_canCmdEnable = FALSE;
            g_canCmdTargetRpm = 0;
        }
    }

    /* CONTROL_META (0x203) is logging-only from host; we only need to
     * remember its seq byte so DIAG_STATUS.rx_seq_echo can round-trip it. */
    IfxMultican_Message_init(&msg, CAN_CONTROL_META_ID, 0U, 0U,
                             IfxMultican_DataLengthCode_8);
    status = IfxMultican_Can_MsgObj_readMessage(&g_canMetaRxObj, &msg);

    if ((status & IfxMultican_Status_newData) != 0U)
    {
        messageToBytes(&msg, data);
        g_canMetaSeqEcho = data[6];
    }
}

void Can_Comms_SendDriveStatus(sint16 pwmDuty, sint16 targetRpm)
{
    uint8 data[CAN_STATUS_PAYLOAD_SIZE];

    Can_Status_BuildPayload(data, pwmDuty, targetRpm);
    g_canLastPayload[0] = data[0];
    g_canLastPayload[1] = data[1];
    g_canLastPayload[2] = data[2];
    g_canLastPayload[3] = data[3];
    g_canLastPayload[4] = data[4];
    g_canLastPayload[5] = data[5];
    g_canLastPayload[6] = data[6];
    g_canLastPayload[7] = data[7];

    sendCanFrame(&g_canTxObj3, CAN_DRIVE_STATUS_ID, data);
}

void Can_Comms_SendSteeringStatus(void)
{
    uint8 data[CAN_STATUS_PAYLOAD_SIZE];
    sint16 currentAngle10;
    sint16 targetAngle10;

    currentAngle10 = (sint16)(g_steeringCurrentAngle * 10.0f);
    targetAngle10 = (sint16)(g_steeringTargetAngle * 10.0f);

    data[0] = (uint8)(g_steeringPotValue & 0xFFU);
    data[1] = (uint8)((g_steeringPotValue >> 8U) & 0xFFU);

    data[2] = (uint8)((uint16)g_steeringTargetPot & 0xFFU);
    data[3] = (uint8)(((uint16)g_steeringTargetPot >> 8U) & 0xFFU);

    data[4] = (uint8)((uint16)currentAngle10 & 0xFFU);
    data[5] = (uint8)(((uint16)currentAngle10 >> 8U) & 0xFFU);

    data[6] = (uint8)((uint16)targetAngle10 & 0xFFU);
    data[7] = (uint8)(((uint16)targetAngle10 >> 8U) & 0xFFU);

    sendCanFrame(&g_canTxObj2, CAN_STEERING_STATUS_ID, data);
}

void Can_Comms_SendDiagStatus(uint8 appliedStopMode,
                              uint8 faultFlags,
                              sint16 steerPwmDuty,
                              uint16 supplyVoltageMv)
{
    uint8 data[8];

    data[0] = appliedStopMode;
    data[1] = faultFlags;
    data[2] = (uint8)((uint16)steerPwmDuty & 0xFFU);
    data[3] = (uint8)(((uint16)steerPwmDuty >> 8U) & 0xFFU);
    data[4] = (uint8)(supplyVoltageMv & 0xFFU);
    data[5] = (uint8)((supplyVoltageMv >> 8U) & 0xFFU);
    data[6] = g_canMetaSeqEcho;
    data[7] = 0U;

    sendCanFrame(&g_canTxObj4, CAN_DIAG_STATUS_ID, data);
}
