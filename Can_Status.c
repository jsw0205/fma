#include "Can_Status.h"

#include "Encoder_Motor.h"

void Can_Status_BuildPayload(uint8 payload[CAN_STATUS_PAYLOAD_SIZE],
                             sint16 pwmDuty,
                             sint16 targetRpm)
{
    sint16 encoderCount;
    sint16 actualRpm10;
    float32 actualRpm;

    encoderCount = (sint16)g_encoderCount;
    actualRpm = g_encoderActualRpm * 10.0f;

    if (actualRpm > 32767.0f)
    {
        actualRpm10 = 32767;
    }
    else if (actualRpm < -32768.0f)
    {
        actualRpm10 = -32768;
    }
    else
    {
        actualRpm10 = (sint16)actualRpm;
    }

    payload[0] = (uint8)((uint16)encoderCount & 0xFFU);
    payload[1] = (uint8)(((uint16)encoderCount >> 8U) & 0xFFU);

    payload[2] = (uint8)((uint16)actualRpm10 & 0xFFU);
    payload[3] = (uint8)(((uint16)actualRpm10 >> 8U) & 0xFFU);

    payload[4] = (uint8)(pwmDuty & 0xFFU);
    payload[5] = (uint8)((pwmDuty >> 8U) & 0xFFU);

    payload[6] = (uint8)((uint16)targetRpm & 0xFFU);
    payload[7] = (uint8)(((uint16)targetRpm >> 8U) & 0xFFU);
}
