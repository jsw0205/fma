#ifndef STEERING_H
#define STEERING_H

#include "Ifx_Types.h"

#define STEER_MAX_ANGLE    30.0f
#define POT_RIGHT          330
#define POT_CENTER         1700
#define POT_LEFT           3070
#define POT_TOLERANCE      15

extern volatile uint32 g_steeringPotValue;
extern volatile sint32 g_steeringTargetPot;
extern volatile float32 g_steeringCurrentAngle;
extern volatile float32 g_steeringTargetAngle;
extern volatile float32 g_steeringPwmPercent;

void Steering_Init(void);
void Steering_SetTargetAngle(float32 targetAngle);
void Steering_SetEnable(boolean enable);
void Steering_Update_5ms(void);

#endif /* STEERING_H */
