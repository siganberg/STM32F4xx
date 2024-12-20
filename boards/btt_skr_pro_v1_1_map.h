/*
  skr_pro_v1_1_map.h - driver code for STM32F407 ARM processors

  Part of grblHAL

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

// IMPORTANT: Control inputs are mapped differently when build is configured for more than 3 axes!

#if N_ABC_MOTORS > 3 || N_AUTO_SQUARED > 2
#error "Axis configuration is not supported!"
#endif

#if IS_NUCLEO_DEVKIT
// When debugging with Nucleo-144
#elif !defined(STM32F407xx) || HSE_VALUE != 8000000
#error "This board has STM32F407 processor with a 8MHz crystal, select a corresponding build!"
#endif

#ifdef BOARD_BTT_SKR_PRO_1_2
#define BOARD_NAME "BTT SKR PRO v1.2"
#define BOARD_URL "https://github.com/bigtreetech/BIGTREETECH-SKR-PRO-V1.1/tree/master/SKR-PRO-V1.2"
#else
#define BOARD_NAME "BTT SKR PRO v1.1"
#define BOARD_URL "https://github.com/bigtreetech/BIGTREETECH-SKR-PRO-V1.1"
#endif

#define SERIAL_PORT     1       // GPIOA: TX = 9, RX = 10
#define SERIAL1_PORT   32       // GPIOD: TX = 8, RX = 9
#define SERIAL2_PORT    6       // GPIOC: TX = 6, RX = 7
#define I2C_PORT        1
#define I2C1_ALT_PINMAP         // GPIOB: SCL = 6, SDA = 7

// Define step pulse output pins.
#define X_STEP_PORT                 GPIOE
#define X_STEP_PIN                  9                   // X
#define Y_STEP_PORT                 GPIOE
#define Y_STEP_PIN                  11                  // Y
#define Z_STEP_PORT                 GPIOE
#define Z_STEP_PIN                  13                  // Z
#define STEP_OUTMODE                GPIO_BITBAND
//#define STEP_PINMODE                PINMODE_OD // Uncomment for open drain outputs

// Define step direction output pins.
#define X_DIRECTION_PORT            GPIOF
#define X_DIRECTION_PIN             1
#define Y_DIRECTION_PORT            GPIOE
#define Y_DIRECTION_PIN             8
#define Z_DIRECTION_PORT            GPIOC
#define Z_DIRECTION_PIN             2
#define DIRECTION_OUTMODE           GPIO_BITBAND
//#define DIRECTION_PINMODE           PINMODE_OD // Uncomment for open drain outputs

// Define stepper driver enable/disable output pin.
#define X_ENABLE_PORT               GPIOF
#define X_ENABLE_PIN                2
#define Y_ENABLE_PORT               GPIOD
#define Y_ENABLE_PIN                7
#define Z_ENABLE_PORT               GPIOC
#define Z_ENABLE_PIN                0
//#define STEPPERS_ENABLE_PINMODE   PINMODE_OD // Uncomment for open drain outputs

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT                GPIOB
#define X_LIMIT_PIN                 10                          // X- Limit
#define Y_LIMIT_PORT                GPIOE
#define Y_LIMIT_PIN                 12                          // Y- Limit
#define Z_LIMIT_PORT                GPIOG
#define Z_LIMIT_PIN                 8                           // Z- Limit
#define LIMIT_INMODE                GPIO_BITBAND

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE                // E0
#define M3_STEP_PORT                GPIOE
#define M3_STEP_PIN                 14
#define M3_DIRECTION_PORT           GPIOA
#define M3_DIRECTION_PIN            0
#define M3_LIMIT_PORT               GPIOE
#define M3_LIMIT_PIN                15
#define M3_ENABLE_PORT              GPIOC
#define M3_ENABLE_PIN               3
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 1
#define M4_AVAILABLE                // E1
#define M4_STEP_PORT                GPIOD
#define M4_STEP_PIN                 15
#define M4_DIRECTION_PORT           GPIOE
#define M4_DIRECTION_PIN            7
#define M4_LIMIT_PORT               GPIOG // NOTE: Cannot use the E1 input due to conflict
#define M4_LIMIT_PIN                5     // with X limit switch IRQ. Using E2 input instead.
#define M4_ENABLE_PORT              GPIOA
#define M4_ENABLE_PIN               3
#endif

// Define ganged axis or C axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 3
#define M5_AVAILABLE                // E2
#define M5_STEP_PORT                GPIOD
#define M5_STEP_PIN                 13
#define M5_DIRECTION_PORT           GPIOG
#define M5_DIRECTION_PIN            9
#define M5_ENABLE_PORT              GPIOF
#define M5_ENABLE_PIN               0
#endif

#define AUXOUTPUT0_PORT             GPIOA // Spindle PWM, EXP1
#define AUXOUTPUT0_PIN              8
#define AUXOUTPUT1_PORT             GPIOE // Spindle direction, FAN2
#define AUXOUTPUT1_PIN              6
#define AUXOUTPUT2_PORT             GPIOE // Spindle enable, FAN1
#define AUXOUTPUT2_PIN              5
#define AUXOUTPUT3_PORT             GPIOB // Coolant flood, HEAT0
#define AUXOUTPUT3_PIN              1
#define AUXOUTPUT4_PORT             GPIOD // Coolant mist, HEAT1
#define AUXOUTPUT4_PIN              14

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT         AUXOUTPUT2_PORT
#define SPINDLE_ENABLE_PIN          AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT            AUXOUTPUT0_PORT
#define SPINDLE_PWM_PIN             AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT      AUXOUTPUT1_PORT
#define SPINDLE_DIRECTION_PIN       AUXOUTPUT1_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT          AUXOUTPUT3_PORT
#define COOLANT_FLOOD_PIN           AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT           AUXOUTPUT4_PORT
#define COOLANT_MIST_PIN            AUXOUTPUT4_PIN
#endif

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define RESET_PORT                  GPIOG
#define RESET_PIN                   6                           // EXP1 PG6
#define FEED_HOLD_PORT              GPIOD
#define FEED_HOLD_PIN               11                          // EXP1 PD11
#define CYCLE_START_PORT            GPIOG
#define CYCLE_START_PIN             2                           // EXP1 PG2
#define CONTROL_INMODE GPIO_BITBAND

#define AUXINPUT0_PORT              GPIOG
#define AUXINPUT0_PIN               4                           // EXP1 PG4
#define AUXINPUT1_PORT              GPIOA
#define AUXINPUT1_PIN               1                           // BLTouch PA1

#if PROBE_ENABLE
#define PROBE_PORT                  AUXINPUT1_PORT
#define PROBE_PIN                   AUXINPUT1_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT            AUXINPUT0_PORT
#define SAFETY_DOOR_PIN             AUXINPUT0_PIN
#endif

#if MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT            AUXINPUT0_PORT
#define MOTOR_FAULT_PIN             AUXINPUT0_PIN
#endif

#if SDCARD_ENABLE
// Connect to the  EXP2 Header
#define SD_CS_PORT                  GPIOB
#define SD_CS_PIN                   12
#define SPI_PORT                    2 // GPIOB, SCK_PIN = 13, MISO_PIN = 14, MOSI_PIN = 15
#endif

/*
#if SDCARD_ENABLE
// On board socket- flaky?
#define SD_CS_PORT                  GPIOA
#define SD_CS_PIN                   4
#define SPI_PORT                    11 // (MCU port 1) GPIOA: SCK_PIN = 5, MISO_PIN = 5, GBIOB: MOSI_PIN = 5
// PB11 - TF_SW - card detect
#endif
*/

#if TRINAMIC_UART_ENABLE

#undef TRINAMIC_UART_ENABLE
#define TRINAMIC_UART_ENABLE        2

#define MOTOR_UARTX_PORT            GPIOC
#define MOTOR_UARTX_PIN             13
#define MOTOR_UARTY_PORT            GPIOE
#define MOTOR_UARTY_PIN             3
#define MOTOR_UARTZ_PORT            GPIOE
#define MOTOR_UARTZ_PIN             1

#ifdef  M3_AVAILABLE
#define MOTOR_UARTM3_PORT           GPIOD
#define MOTOR_UARTM3_PIN            4
#endif

#ifdef  M4_AVAILABLE
#define MOTOR_UARTM4_PORT           GPIOD
#define MOTOR_UARTM4_PIN            1
#endif

#ifdef  M5_AVAILABLE
#define MOTOR_UARTM5_PORT           GPIOD
#define MOTOR_UARTM5_PIN            6
#endif

#endif

#if TRINAMIC_SPI_ENABLE

#define TRINAMIC_SPI_PORT           3 // GPIOC, SCK_PIN = 10, MISO_PIN = 12, MOSI_PIN = 11

#define MOTOR_CSX_PORT              GPIOA
#define MOTOR_CSX_PIN               15
#define MOTOR_CSY_PORT              GPIOB
#define MOTOR_CSY_PIN               8
#define MOTOR_CSZ_PORT              GPIOB
#define MOTOR_CSZ_PIN               9

#ifdef  M3_AVAILABLE
#define MOTOR_CSM3_PORT             GPIOB
#define MOTOR_CSM3_PIN              3
#endif

#ifdef  M4_AVAILABLE
#define MOTOR_CSM4_PORT             GPIOG
#define MOTOR_CSM4_PIN              15
#endif

#ifdef  M5_AVAILABLE
#define MOTOR_CSM5_PORT             GPIOG
#define MOTOR_CSM5_PIN              12
#endif

#endif

#define CAN_PORT                GPIOD
#define CAN_RX_PIN              0
#define CAN_TX_PIN              1
