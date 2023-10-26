/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Mightyboard Rev.G pin assignments
 */

/**
 *
 * This is a starting-point to support the Makerbot Replicator series of 3D printers.
 * It's not functional because Marlin has limited support for some features.
 * Marlin will need the following augmentations before it will be supportable:
 *
 *   - Support for two or more MAX6675 thermocouples
 *   - Support for multiple i2c buses to control the MCP4018 digital pots
 *   - Support for one additional unidirectional SPI bus, to read the thermocouples
 *   - Support for an RGB LED that may work differently from BLINKM
 *
 * The MCP4018 requires separate I2C buses because it has a fixed address (0x2F << 1 = 0x5E)
 * The thermocouples share the same SCK and DO pins, with their own CS pins.
 * The controller interface port connects to a 3-wire shift-register display controller
 *
 */

/**
 * Rev B  2 JAN 2017
 *
 *  Added pin definitions for:
 *    M3, M4 & M5 spindle control commands
 *    case light
 *
 *  Corrected pin assignment for MOSFET_B_PIN pin. Changed it from 9 to 11.  The port
 *  number (B5) agrees with the schematic but B5 is assigned to logical pin 11.
 */

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error "Oops! Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu."
#endif

#define DEFAULT_MACHINE_NAME    "MB Replicator 2X"
#define BOARD_NAME              "Mightyboard"

//
// Servos NONE
//
//#define SERVO0_PIN         36   // C1 (1280-EX1)
//#define SERVO1_PIN         37   // C0 (1280-EX2)
//#define SERVO2_PIN         40   // G1 (1280-EX3)
//#define SERVO3_PIN         41   // G0 (1280-EX4)

//
// Limit Switches
// all active high (enable pull up)
//
#define X_MIN_PIN          72   // J2 (Not connected)
#define X_MAX_PIN          30   // C7
#define Y_MIN_PIN          14   // J1 (Not connected)
#define Y_MAX_PIN          31   // C6
#define Z_MIN_PIN          32   // C5
#define Z_MAX_PIN          15   // J0 (Not connected)

//
// Steppers
//
#define X_STEP_PIN         83   // D6
#define X_DIR_PIN          38   // D7
#define X_ENABLE_PIN       81   // D4

#define Y_STEP_PIN         44   // L5
#define Y_DIR_PIN          42   // L7
#define Y_ENABLE_PIN       45   // L4

#define Z_STEP_PIN         48   // L1
#define Z_DIR_PIN          47   // L2
#define Z_ENABLE_PIN       49   // L0

#define E0_STEP_PIN        25   // A3
#define E0_DIR_PIN         24   // A2
#define E0_ENABLE_PIN      27   // A5

#define E1_STEP_PIN        22   // A0
#define E1_DIR_PIN         69   // K7
#define E1_ENABLE_PIN      23   // A1

//
// I2C Digipots - MCP4018
// Address 5E (2F << 1)
// Set from 0 - 127 with stop bit.
// (Ex. 3F << 1 | 1)
//
#define DIGIPOTS_I2C_SCL    76   // A4
#define DIGIPOTS_I2C_SDA_X  57   // D5
#define DIGIPOTS_I2C_SDA_Y  61   // L7
#define DIGIPOTS_I2C_SDA_Z  65   // L3
#define DIGIPOTS_I2C_SDA_E0 27   // A4
#define DIGIPOTS_I2C_SDA_E1 77   // J7
#define DIGIPOT_I2C_ADDRESS_A 0x2F   // unshifted slave address (5E <- 2F << 1)
// Make sure to enable DIGIPOT_I2C and DIGIPOT_MCP4018 in Configuration_adv.h
// DIGIPOT_I2C_NUM_CHANNELS should be 5 w/ matching values for DIGIPOT_I2C_MOTOR_CURRENTS

//
// Temperature Sensors
//
#define TEMP_BED_PIN        3   // F3 / ADC3 / 94

// SPI for TI ADS1118 Dual thermocouple
// Uses a separate SPI bus
//
// 78 E2 SCK
// 79 E6 CS
// 80 E7 DI (MOSI)
// 84 H2 DO (MISO)
//
#define THERMO_SCK_PIN     78   // E2
#define THERMO_CS_PIN      79   // E6
#define THERMO_MOSI_PIN    80   // E7
#define THERMO_MISO_PIN    84   // H2

#define ADS1118_SCK_PIN    THERMO_SCK_PIN
#define ADS1118_CS         THERMO_CS_PIN
#define ADS1118_DI_PIN     THERMO_MOSI_PIN
#define ADS1118_DO_PIN     THERMO_MISO_PIN

//
// Heaters / Fans
//
// Extruders & bed are high to enable
#define HEATER_0_PIN        3   // E5 - Extruder A / R Extruder
#define HEATER_1_PIN        5   // E3 - Extruder B / L Extruder
#define HEATER_BED_PIN      8   // H5

#define FAN_PIN             4   // G5 - External cooling fan (Not connected)
#define E0_AUTO_FAN_PIN     7   // H4
#define E1_AUTO_FAN_PIN     2   // E4

//
// LED's
//
#define DEBUG0             70   // G4 - Corresponds to LED DS4
#define DEBUG1             13   // B7 - Corresponds to LED DS5
#define DEBUG2             71   // G3 - Corresponds to LED DS6
#define DEBUG3             85   // H7 - Corresponds to LED DS7

// HOW TO ENABLED(PCA9632)
#define HAS_CASE_LIGHT      1   // case light is a PCA9632 RGB controller
// For some reason, this pin is used as the main LED pin by most people
#define LED_PIN            DEBUG1
/*#define CUTOFF_RESET_PIN   16   // H1
#define CUTOFF_TEST_PIN    17   // H0
#define CASE_LIGHT_PIN     44   // L5   MUST BE HARDWARE PWM

//
// LCD / Controller
//
#ifdef REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER

  #define LCD_PINS_RS      33   // C4: LCD-STROBE
  #define LCD_PINS_ENABLE  72   // J2: LEFT
  #define LCD_PINS_D4      35   // C2: LCD-CLK
  #define LCD_PINS_D5      32   // C5: RLED
  #define LCD_PINS_D6      34   // C3: LCD-DATA
  #define LCD_PINS_D7      31   // C6: GLED

  #define BTN_EN2          75   // J4, UP
  #define BTN_EN1          73   // J3, DOWN
  //STOP button connected as KILL_PIN
  #define KILL_PIN         14   // J1, RIGHT
  //KILL - not connected

  #define BEEPER_PIN        8   // H5, SD_WP

  #define BTN_CENTER       15   // J0
  #define BTN_ENC          BTN_CENTER

  //on board leds
  #define STAT_LED_RED_LED  SERVO0_PIN   // C1 (1280-EX1, DEBUG2)
  #define STAT_LED_BLUE_PIN SERVO1_PIN   // C0 (1280-EX2, DEBUG3)

#else
  // Replicator uses a 3-wire SR controller with HD44780
  // For now, pretend it's the SAV
  //

  #define SAV_3DLCD
  #define SR_DATA_PIN      34   // C3
  #define SR_CLK_PIN       35   // C2
  #define SR_STROBE_PIN    33   // C4

  #define BTN_UP           75   // J4
  #define BTN_DOWN         73   // J3
  #define BTN_LEFT         72   // J2
  #define BTN_RIGHT        14   // J1
  #define BTN_CENTER       15   // J0
  #define BTN_ENC          BTN_CENTER

  #define BEEPER_PIN        4   // G5

  #define STAT_LED_RED_PIN  32   // C5
  #define STAT_LED_BLUE_PIN 31   // C6 (Actually green)

#endif

//
// SD Card
//
#define SDSS               53   // B0
#define SD_DETECT_PIN       9   // H6

#define MAX_PIN             THERMO_SCK_PIN


// Check if all pins are defined in mega/pins_arduino.h
#include <Arduino.h>
static_assert(NUM_DIGITAL_PINS > MAX_PIN, "add missing pins to [arduino dir]/hardware/arduino/avr/variants/mega/pins_arduino.h based on fastio.h"
                                          "to digital_pin_to_port_PGM, digital_pin_to_bit_mask_PGM, digital_pin_to_timer_PGM, NUM_DIGITAL_PINS, see below");
*/

