#ifndef PS2_CTRL_H
#define PS2_CTRL_H

#include "Ifx_Types.h"

extern volatile uint16 g_ps2TargetRpm;
extern volatile uint8 g_ps2InvalidReadCount;
extern volatile boolean g_ps2ReverseMode;
extern volatile boolean g_ps2Connected;
extern volatile boolean g_ps2AnalogMode;
extern volatile boolean g_ps2AnalogOffPatternDetected;
extern volatile uint16 g_ps2OffPatternCount;
extern volatile boolean g_ps2FlatStopMode;
extern volatile boolean g_ps2CanControlMode;

void Ps2_Ctrl_Init(void);
void Ps2_Ctrl_Update_5ms(void);
float32 Ps2_Ctrl_GetTargetRpm(void);
float32 Ps2_Ctrl_GetTargetSteer(void);
boolean Ps2_Ctrl_GetEnable(void);

#endif /* PS2_CTRL_H */
