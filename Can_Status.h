#ifndef CAN_STATUS_H
#define CAN_STATUS_H

#include "Ifx_Types.h"

#define CAN_STATUS_PAYLOAD_SIZE  8U

void Can_Status_BuildPayload(uint8 payload[CAN_STATUS_PAYLOAD_SIZE],
                             sint16 pwmDuty,
                             sint16 targetRpm);

#endif /* CAN_STATUS_H */
