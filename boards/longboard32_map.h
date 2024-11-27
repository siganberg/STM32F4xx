/*
  longboard32_map.h - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

  Copyright (C) Sienci Labs Inc.

   This file is part of the SuperLongBoard family of products.

   This source describes Open Hardware and is licensed under the "CERN-OHL-S v2"

   You may redistribute and modify this source and make products using
   it under the terms of the CERN-OHL-S v2 (https://ohwr.org/cern_ohl_s_v2.t).
   This source is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY,
   INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A
   PARTICULAR PURPOSE. Please see the CERN-OHL-S v2 for applicable conditions.

   As per CERN-OHL-S v2 section 4, should You produce hardware based on this
   source, You must maintain the Source Location clearly visible on the external
   case of the CNC Controller or other product you make using this source.

   You should have received a copy of the CERN-OHL-S v2 license with this source.
   If not, see <https://ohwr.org/project/cernohl/wikis/Documents/CERN-OHL-version-2>.

   Contact for information regarding this program and its license
   can be sent through gSender@sienci.com or mailed to the main office
   of Sienci Labs Inc. in Waterloo, Ontario, Canada.
*/

#if N_ABC_MOTORS > 2
#error "Axis configuration is not supported!"
#endif

#if !defined(STM32F412Vx) || HSE_VALUE != 25000000
#error "This board has STM32F412 processor with a 25MHz crystal, select a corresponding build!"
#endif

#ifndef BOARD_NAME
#define BOARD_NAME "SuperLongBoard"
#endif
#define BOARD_URL "Sienci Super Longboard"

#ifdef SLB_ORANGE_BOARD
  #undef SLB_EEPROM_ENABLE
  #undef I2C_ENABLE
  #undef EEPROM_ENABLE
  #undef SD_CARD_ENABLE
  #undef SIENCI_LASER_PWM
  #undef SDCARD_ENABLE
  #undef SAFETY_DOOR_ENABLE
  #undef _WIZCHIP_
  #undef TELNET_ENABLE
  #undef ETHERNET_ENABLE
  #undef FTP_ENABLE
  #undef WEBSOCKET_ENABLE
  #undef NETWORK_IPMODE
  #undef MODBUS_ENABLE
#endif

#if SLB_EEPROM_ENABLE
#undef I2C_ENABLE
#undef EEPROM_ENABLE
#define I2C_ENABLE 1
#define EEPROM_ENABLE 2
#define I2C_PORT 1
#define I2C1_ALT_PINMAP 1
#endif
#define HAS_IOPORTS

#define HAS_BOARD_INIT
#define WIZCHIP_SPI_PRESCALER SPI_BAUDRATEPRESCALER_2

#if MODBUS_ENABLE
#ifdef SLB_ORANGE_BOARD
#define SERIAL2_MOD 3
#define MODBUS_SERIAL_PORT 2
#else
#define SERIAL2_MOD 2
#define MODBUS_SERIAL_PORT 1
#endif
#endif

#if MPG_MODE == 1
#define MPG_MODE_PORT           GPIOA
#define MPG_MODE_PIN            4
#endif

// Define step pulse output pins.
#define X_STEP_PORT             GPIOE
#define X_STEP_PIN              2
#define Y_STEP_PORT             GPIOE
#define Y_STEP_PIN              3
#define Z_STEP_PORT             GPIOE
#define Z_STEP_PIN              4
#define STEP_OUTMODE            GPIO_BITBAND
//#define STEP_PINMODE            PINMODE_OD // Uncomment for open drain outputs

// Define step direction output pins.
#define X_DIRECTION_PORT        GPIOE
#define X_DIRECTION_PIN         7
#define Y_DIRECTION_PORT        GPIOE
#define Y_DIRECTION_PIN         8
#define Z_DIRECTION_PORT        GPIOE
#define Z_DIRECTION_PIN         9
#define DIRECTION_OUTMODE       GPIO_BITBAND
//#define DIRECTION_PINMODE       PINMODE_OD // Uncomment for open drain outputs

// Define stepper driver enable/disable output pin.
#define X_ENABLE_PORT               GPIOD
#define X_ENABLE_PIN                4
#define Y_ENABLE_PORT               GPIOC
#define Y_ENABLE_PIN                7
#define Z_ENABLE_PORT               GPIOD
#define Z_ENABLE_PIN                3
//#define STEPPERS_ENABLE_PORT       GPIOA
//#define STEPPERS_ENABLE_PIN        14
#define STEPPERS_ENABLE_OUTMODE    GPIO_BITBAND

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT            GPIOB
#define X_LIMIT_PIN             5
#define Y_LIMIT_PORT            GPIOB
#define Y_LIMIT_PIN             9
#define Z_LIMIT_PORT            GPIOC
#define Z_LIMIT_PIN             13
#define LIMIT_INMODE            GPIO_BITBAND

// Define ganged axis or A axis step pulse and step direction output pins.
#if (N_ABC_MOTORS > 0) && (N_AXIS == 3)

#ifdef SLB_ORANGE_BOARD
#error "Not a supported configuration"
#endif

#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOE
#define M3_STEP_PIN             5
#define M3_DIRECTION_PORT       GPIOE
#define M3_DIRECTION_PIN        10
#define M3_LIMIT_PORT           GPIOE
#define M3_LIMIT_PIN            6
#define M3_ENABLE_PORT          GPIOC
#define M3_ENABLE_PIN           7
#endif

// Define ganged axis or A axis step pulse and step direction output pins.
//note that because of how GRBLHAL iterates the axes, the M3 and M4 need to swap
#if (N_ABC_MOTORS == 2) && (N_AXIS == 4)

#ifdef SLB_ORANGE_BOARD

#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOE
#define M3_STEP_PIN             6
#define M3_DIRECTION_PORT       GPIOE
#define M3_DIRECTION_PIN        11
#define M3_LIMIT_PORT           GPIOE
#define M3_LIMIT_PIN            14
#define M3_ENABLE_PORT          GPIOC
#define M3_ENABLE_PIN           10

#define M4_AVAILABLE
#define M4_STEP_PORT            GPIOE
#define M4_STEP_PIN             5
#define M4_DIRECTION_PORT       GPIOE
#define M4_DIRECTION_PIN        10
#define M4_LIMIT_PORT           GPIOB
#define M4_LIMIT_PIN            6
#define M4_ENABLE_PORT          GPIOC
#define M4_ENABLE_PIN           7

#else

#define M4_AVAILABLE
#define M4_STEP_PORT            GPIOE
#define M4_STEP_PIN             5
#define M4_DIRECTION_PORT       GPIOE
#define M4_DIRECTION_PIN        10
#define M4_LIMIT_PORT           GPIOE
#define M4_LIMIT_PIN            6
#define M4_ENABLE_PORT          GPIOC
#define M4_ENABLE_PIN           7

#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOB
#define M3_STEP_PIN             10
#define M3_DIRECTION_PORT       GPIOE
#define M3_DIRECTION_PIN        11
#define M3_LIMIT_PORT           GPIOE
#define M3_LIMIT_PIN            14
#define M3_ENABLE_PORT          GPIOC
#define M3_ENABLE_PIN           10

#endif

#endif

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     GPIOE
#define SPINDLE_ENABLE_PIN      13
#define SPINDLE_DIRECTION_PORT  GPIOB
#define SPINDLE_DIRECTION_PIN   1

// Define spindle PWM output pin.
#ifdef SIENCI_LASER_PWM
#ifndef SLB_ORANGE_BOARD
#define LASER_ENABLE_PORT     GPIOC
#define LASER_ENABLE_PIN      12
#define LASER_PWM_PORT_BASE   GPIOC_BASE
#define LASER_PWM_PIN         6
//#define SPINDLE_PWM_TIMER_N     3
#endif
#endif
#define SPINDLE_PWM_PORT_BASE   GPIOA_BASE
#define SPINDLE_PWM_PIN         8
#define SPINDLE_PWM_PORT            ((GPIO_TypeDef *)SPINDLE_PWM_PORT_BASE)


// Define flood and mist coolant enable output pins.
#ifdef SLB_ORANGE_BOARD
#define COOLANT_FLOOD_PORT      GPIOD
#define COOLANT_FLOOD_PIN       7
#else
#define COOLANT_FLOOD_PORT      GPIOB
#define COOLANT_FLOOD_PIN       14
#endif
#define COOLANT_MIST_PORT       GPIOB
#define COOLANT_MIST_PIN        4
#define COOLANT_OUTMODE         GPIO_BITBAND


//*****Switchbank will always claim the first 4 aux outputs******
#ifdef SLB_ORANGE_BOARD
#define AUXOUTPUT0_PORT         GPIOC
#define AUXOUTPUT0_PIN          14
#define AUXOUTPUT1_PORT         GPIOC
#define AUXOUTPUT1_PIN          1
#define AUXOUTPUT2_PORT         GPIOC
#define AUXOUTPUT2_PIN          2
#define AUXOUTPUT3_PORT         GPIOC
#define AUXOUTPUT3_PIN          4
#else
#define AUXOUTPUT3_PORT         GPIOC
#define AUXOUTPUT3_PIN          14
#define AUXOUTPUT2_PORT         GPIOC
#define AUXOUTPUT2_PIN          0
#define AUXOUTPUT1_PORT         GPIOC
#define AUXOUTPUT1_PIN          1
#define AUXOUTPUT0_PORT         GPIOC
#define AUXOUTPUT0_PIN          8
#endif

/*modbus direction pin*/
#define AUXOUTPUT4_PORT         GPIOB
#define AUXOUTPUT4_PIN          0
#define MODBUS_DIR_AUX          4

/*defines for neopixel bitbang pins*/
#if STATUS_LIGHT_ENABLE
#define AUXOUTPUT5_PORT         GPIOC //rail LED strip
#define AUXOUTPUT5_PIN          9    //rail LED strip
#define AUXOUTPUT6_PORT         GPIOA //ring LED strip
#define AUXOUTPUT6_PIN          13    //ring LED strip
#endif
#define RING_LED_AUXOUT         6
#define RAIL_LED_AUXOUT         5

#define AUXOUTPUT_OUTMODE       GPIO_BITBAND

#define AUXINPUT0_PORT          GPIOD
#define AUXINPUT0_PIN           12
#define AUXINPUT1_PORT          GPIOD
#define AUXINPUT1_PIN           13
#define AUXINPUT2_PORT          GPIOD
#define AUXINPUT2_PIN           14

/*TLS/PRB DET pin*/
#ifdef SLB_ORANGE_BOARD
#define AUXINPUT3_PORT          GPIOD
#define AUXINPUT3_PIN           15
#else
#define AUXINPUT3_PORT          GPIOC
#define AUXINPUT3_PIN           15
#endif

/*maco pins, aux for now*/
//macro 1
#define AUXINPUT4_PORT          GPIOE
#define AUXINPUT4_PIN           1
#define MACRO_1_AUXIN           4
//macro 2
#define AUXINPUT5_PORT          GPIOE
#define AUXINPUT5_PIN           0
#define MACRO_2_AUXIN           5
//macro 3 (RUN)
#define AUXINPUT6_PORT          GPIOC
#define AUXINPUT6_PIN           11
#define MACRO_3_AUXIN           6

//not currently supported in GRBLHAL
//#define AUXINPUT7_PORT          GPIOD
//#define AUXINPUT7_PIN           15

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define RESET_PORT              GPIOB
#define RESET_PIN               12
#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        GPIOD
#define SAFETY_DOOR_PIN         7
#endif
#define CONTROL_INMODE          GPIO_BITBAND

// Define probe switch input pin.
#define PROBE_PORT         GPIOC
#define PROBE_PIN          4

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         GPIOB
#define I2C_STROBE_PIN          3
#endif

#define TMC_STEALTHCHOP 0

// SPI2 is used: GPIOB pin 12 (SCK) GPIOC pin 2 (MISO) and 3 (MOSI)
#define MOTOR_CSX_PORT              GPIOD
#define MOTOR_CSX_PIN               11
#define MOTOR_CSY_PORT              GPIOE
#define MOTOR_CSY_PIN               15
#define MOTOR_CSZ_PORT              GPIOD
#define MOTOR_CSZ_PIN               9

//stallguard inputs
#define MOTOR_SGX_PORT              GPIOA
#define MOTOR_SGX_PIN               15
#define MOTOR_SGY1_PORT             GPIOA
#define MOTOR_SGY1_PIN              1
#define MOTOR_SGZ_PORT              GPIOA
#define MOTOR_SGZ_PIN               2
#define MOTOR_SGY2_PORT             GPIOA
#define MOTOR_SGY2_PIN              3

#ifdef SLB_ORANGE_BOARD
#define MOTOR_SGA_PORT              GPIOB
#define MOTOR_SGA_PIN               2
#else
#define MOTOR_SGA_PORT              GPIOD
#define MOTOR_SGA_PIN               2
#endif

#if (N_ABC_MOTORS > 0) && (N_AXIS == 3)
#define MOTOR_CSM3_PORT             GPIOE
#define MOTOR_CSM3_PIN              12
#endif

#if (N_ABC_MOTORS == 2) && (N_AXIS == 4)
#define MOTOR_CSM4_PORT             GPIOE
#define MOTOR_CSM4_PIN              12
#endif

#if SDCARD_ENABLE || ETHERNET_ENABLE
#define SPI_PORT                1 // GPIOA, SCK_PIN = 5, MISO_PIN = 6, MOSI_PIN = 7  probably needs fixing
#endif

#if ETHERNET_ENABLE
#undef SPI_ENABLE
#define SPI_ENABLE 1
#define SPI_CS_PORT             GPIOA //CS_JOG_SW
#define SPI_CS_PIN              4
#define SPI_IRQ_PORT            GPIOA //RXD_INT
#define SPI_IRQ_PIN             10
#define SPI_RST_PORT            GPIOA // TXD_INT
#define SPI_RST_PIN             9
#endif

#if SDCARD_ENABLE
#define SD_CS_PORT              GPIOB
#define SD_CS_PIN               15
#endif

  #if LASER_PWM_PIN == 6 // PC6 - TIM3_CH1N
    #define LASER_PWM_TIMER_N     3
    #define LASER_PWM_TIMER_CH    1
    #define LASER_PWM_TIMER_INV   0
    #define LASER_PWM_TIMER_AF    2
  #endif

    #define LASER_PWM_CCR 1

    #define LASER_PWM_TIMER           timer(LASER_PWM_TIMER_N)
    #define LASER_PWM_TIMER_CCR       timerCCR(LASER_PWM_TIMER_N, LASER_PWM_TIMER_CH)
    #define LASER_PWM_TIMER_CCMR      timerCCMR(LASER_PWM_TIMER_N, LASER_PWM_CCR)
    #define LASER_PWM_CCMR_OCM_SET    timerOCM(LASER_PWM_CCR, LASER_PWM_TIMER_CH)
    #define LASER_PWM_CCMR_OCM_CLR    timerOCMC(LASER_PWM_CCR, LASER_PWM_TIMER_CH)

    #if LASER_PWM_TIMER_INV
    #define LASER_PWM_CCER_EN         timerCCEN(LASER_PWM_TIMER_CH, N)
    #define LASER_PWM_CCER_POL        timerCCP(LASER_PWM_TIMER_CH, N)
    #define LASER_PWM_CR2_OIS         timerCR2OIS(LASER_PWM_TIMER_CH, N)
    #else
    #define LASER_PWM_CCER_EN         timerCCEN(LASER_PWM_TIMER_CH, )
    #define LASER_PWM_CCER_POL        timerCCP(LASER_PWM_TIMER_CH, )
    #define LASER_PWM_CR2_OIS         timerCR2OIS(LASER_PWM_TIMER_CH, )
    #endif

    #define LASER_PWM_PORT            ((GPIO_TypeDef *)LASER_PWM_PORT_BASE)
    #define LASER_PWM_AF              timerAF(LASER_PWM_TIMER_N, LASER_PWM_TIMER_AF)
    #define LASER_PWM_CLOCK_ENA       timerCLKENA(LASER_PWM_TIMER_N)

    #define timerCLKENA(t) timercken(t)
    #define timercken(t) __HAL_RCC_TIM ## t ## _CLK_ENABLE