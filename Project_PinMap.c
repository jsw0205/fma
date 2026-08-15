/*
 * Project_PinMap.c
 *
 * Pin map memo for STM_Interrupt_1_KIT_TC275_LK.
 * This file is documentation only. Do not put runtime logic here.
 *
 * Board:
 *   AURIX TC275
 *
 * Timing:
 *   STM0 interrupt: 100 us tick
 *   appControlStep(): 5 ms, called from Cpu0_Main.c
 *
 *
 * ============================================================================
 * CAN
 * ============================================================================
 *
 * Board CAN header:
 *   CANH      : CAN header pin 1
 *   CANL      : CAN header pin 2
 *
 * MCU-side CAN0 signals:
 *   CAN_TX    : P20.8, CAN node 0 output, TXDCAN0
 *   CAN_RX    : P20.7, CAN node 0 input,  RXDCAN0B
 *   #NEN      : P20.6, GPIO, active-low CAN transceiver enable
 *
 * Current code:
 *   Can_Comms.c
 *     nodeConfig.nodeId  = IfxMultican_NodeId_0
 *     nodeConfig.rxPin   = &IfxMultican_RXD0B_P20_7_IN
 *     nodeConfig.txPin   = &IfxMultican_TXD0_P20_8_OUT
 *     P20.6              = LOW
 *
 * CAN message:
 *   ID        : 0x102, standard, DLC 8
 *   byte 0-1  : encoder count, signed 16-bit, little-endian
 *   byte 2-3  : rpm_x10, signed 16-bit, little-endian
 *   byte 4-5  : PWM duty %, signed 16-bit, little-endian
 *   byte 6-7  : target rpm, signed 16-bit, little-endian
 *
 *   ID        : 0x101, standard, DLC 8
 *   byte 0-1  : steering current pot, unsigned 16-bit, little-endian
 *   byte 2-3  : steering target pot, unsigned 16-bit, little-endian
 *   byte 4-5  : steering current angle x10, signed 16-bit, little-endian
 *   byte 6-7  : steering target angle x10, signed 16-bit, little-endian
 *
 *
 * ============================================================================
 * Lamp LEDs
 * ============================================================================
 *
 * Current code:
 *   Lamp_Control.h
 *
 * Pins:
 *   BLUE_LED     : P02.0, GPIO output
 *   RED_LED      : P02.1, GPIO output
 *   REVERSE_LED  : P33.11, GPIO output
 *
 * Logic:
 *   Analog PS2 mode       : BLUE ON,  RED OFF
 *   Other/fault           : BLUE OFF, RED ON
 *   Reverse selected      : REVERSE_LED blinks, 500 ms ON/OFF
 *   Reverse off/fault     : REVERSE_LED OFF
 *
 *
 * ============================================================================
 * PS2 controller
 * ============================================================================
 *
 * Current code:
 *   PS2_Controller.c
 *
 * Pins:
 *   PS2_CS    : P10.0, GPIO output
 *   PS2_MISO  : P10.1, GPIO input pull-up
 *   PS2_SCLK  : P10.2, GPIO output
 *   PS2_MOSI  : P10.3, GPIO output
 *
 *
 * ============================================================================
 * Steering motor
 * ============================================================================
 *
 * Current code:
 *   Steering.c
 *
 * Pins:
 *   STEER_PWM : P33.4, GTM TOM1 channel 0, TOUT26
 *   STEER_DIR : P00.1, GPIO output
 *   STEER_BRK : P00.2, GPIO output
 *   STEER_POT : VADC channel 7
 *
 * Current setting:
 *   Steering PWM frequency: 5 kHz
 *   Steering angle limit  : +/-20 deg
 *
 *
 * ============================================================================
 * Drive motor
 * ============================================================================
 *
 * Current code:
 *   Encoder_Motor.c
 *
 * Pins:
 *   DRIVE_PWM : P00.4, GTM TOM0 channel 11, TOUT13
 *   DRIVE_DIR : P00.8, GPIO output
 *   DRIVE_BRK : P00.9, GPIO output
 *
 * Current setting:
 *   Drive PWM frequency   : 20 kHz
 *
 *
 * ============================================================================
 * Drive encoder
 * ============================================================================
 *
 * Current code:
 *   Encoder_Motor.c
 *
 * Pins:
 *   ENC_A     : P15.4a
 *   ENC_B     : P33.7
 *
 * Current setting:
 *   Counts per wheel revolution: 300
 *   Encoder A interrupt request: IfxScu_REQ0_P15_4_IN
 *   Encoder B interrupt request: IfxScu_REQ8_P33_7_IN
 *
 *
 * ============================================================================
 * Notes
 * ============================================================================
 *
 * Do not edit iLLD pinmap files for normal project pin changes.
 * Use module source files instead:
 *   CAN      : Can_Comms.c
 *   PS2      : PS2_Controller.c
 *   Steering : Steering.c
 *   Drive    : Encoder_Motor.c
 */
