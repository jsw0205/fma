#ifndef PS2_CONTROLLER_H
#define PS2_CONTROLLER_H

#include "Ifx_Types.h"

extern volatile uint8 g_ps2Rx[9];
extern volatile uint8 g_padId;
extern volatile uint8 g_btnLo;
extern volatile uint8 g_btnHi;

void ps2PinsInit(void);
void ps2ConfigureAnalogMode(void);
void ps2ReadOnce(void);

#endif /* PS2_CONTROLLER_H */
