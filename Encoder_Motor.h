#ifndef ENCODER_MOTOR_H
#define ENCODER_MOTOR_H

#include "Ifx_Types.h"

extern volatile sint32 g_encoderCount;
extern volatile sint32 g_encoderDeltaCount;
extern volatile uint8 g_encoderState;
extern volatile uint8 g_encoderRawA;
extern volatile uint8 g_encoderRawB;
extern volatile uint8 g_encoderPollA;
extern volatile uint8 g_encoderPollB;
extern volatile uint32 g_encoderIsrACount;
extern volatile uint32 g_encoderIsrBCount;
extern volatile uint32 g_encoderRejectedEdgeCount;
extern volatile uint32 g_encoderUpdateCount;
extern volatile uint32 g_encoderCountsPerRev;
extern volatile uint32 g_encoderAverageEdgeTicks;
extern volatile uint8 g_encoderActiveBufferSize;
extern volatile float32 g_encoderStmFrequency;
extern volatile float32 g_encoderRawTimeRpm;
extern volatile float32 g_encoderActualRpm;
extern volatile float32 g_encoderDeltaRpm;
extern volatile sint32 g_encoderWindowDeltaCount;
extern volatile float32 g_driveTargetRpm;
extern volatile float32 g_drivePwmPercent;
extern volatile float32 g_driveTargetPwmPercent;
extern volatile boolean g_driveReverseActive;
extern volatile boolean g_driveTestEnable;
extern volatile float32 g_driveTestPwmPercent;
extern volatile float32 g_driveStopBrakePwmPercent;
extern volatile float32 g_driveHoldPwmPercent;
extern volatile sint32 g_driveHoldPosError;
extern volatile sint8 g_driveHoldDirection;

void Encoder_Motor_Init(void);
void Encoder_Motor_SetEncoderCountsPerRev(uint32 countsPerRev);
void Encoder_Motor_AdvanceRpmBufferSize(void);
void Encoder_Motor_ReduceRpmBufferSize(void);
void Encoder_Motor_SetOpenLoopCommand(float32 pwmPercent,
                                      boolean reverse);
void Encoder_Motor_SetTargetRpm(float32 targetRpm, boolean reverse);
void Encoder_Motor_SetStopHoldEnable(boolean enable);
void Encoder_Motor_SetEnable(boolean enable);
float32 Encoder_Motor_GetActualRpm(void);
void Encoder_Motor_Update_5ms(void);

#endif /* ENCODER_MOTOR_H */
