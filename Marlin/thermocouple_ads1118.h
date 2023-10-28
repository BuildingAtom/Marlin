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
 * Arduino library for MicroChip MCP4728 I2C D/A converter.
 */

#ifndef THERMOCOUPLE_ADS1118_H
#define THERMOCOUPLE_ADS1118_H

#include "MarlinConfig.h"

#if ENABLED(TEMP_ADS1118)

// Make some register maps
#include "softspi.h"

// MSB registers
#define ADS1118_START_SS        0B10000000      // Start a single shot conversion
#define ADS1118_MUX_SEL_0       0B00000000
#define ADS1118_MUX_SEL_1       0B00010000
#define ADS1118_MUX_SEL_2       0B00100000
#define ADS1118_MUX_SEL_3       0B00110000
#define ADS1118_MUX_SEL_4       0B01000000
#define ADS1118_MUX_SEL_5       0B01010000
#define ADS1118_MUX_SEL_6       0B01100000
#define ADS1118_MUX_SEL_7       0B01110000
#define ADS1118_PGA_0_256       0B00001110      // We ignore the other gain modes
#define ADS1118_MODE_SS         0B00000001      // Enable Single Shot Mode

// LSB registers
#define ADS1118_DR_64_SPS       0B01100000      // 64 Hz sample rate (based off of sailfish) (16ms between reads / 80ms max if 4 probes + reference)
#define ADS1118_TS_MODE         0B00010000      // Reference temperature sensor
#define ADS1118_MISO_PULLUP     0B00001000      // Pullup on ADS1118 DO pin when CS is high
#define ADS1118_WRITE_FLAG      0B00000010      // Write the config register

// Some conversion constants
#define ADS1118_CONST_0_256_mV_STEP   (0.0078125)   // mV per ADC step (USED TO CALCULATE TABLE)
#define ADS1118_CONST_TEMP_C_STEP       (0.03125)   // C per ADC step

#define ADS1118_mV_STEP ADS1118_CONST_0_256_mV_STEP
#define ADS1118_mV_TO_STEP(V) int16_t((V)/(ADS1118_mV_STEP))

// How many cycles to read the reference temperature
// Each cycle is about ??? seconds, so we check every minute
#define ADS1118_TS_CHECK_CYCLES        120

// BAD_SENSOR
#define ADS1118_UNPLUGGED_BITS         0x7fff

#define ADS1118_CHANNEL_PROBE(C, P) (ENABLED(HEATER_##P##_USES_ADS1118) && HEATER_##P##_CHANNEL == C)
#define ADS1118_CHANNEL(C) (ADS1118_CHANNEL_PROBE(C, 0) || ADS1118_CHANNEL_PROBE(C, 1)      \
                            || ADS1118_CHANNEL_PROBE(C, 2) || ADS1118_CHANNEL_PROBE(C, 3)   \
                            || ADS1118_CHANNEL_PROBE(C, 4) || ADS1118_CHANNEL_PROBE(C, BED) \
                            || ADS1118_CHANNEL_PROBE(C, CHAMBER))

class Ads1118 {
private:
    static int16_t reference_compensation;
    // static const uint8_t common_msb = ADS1118_START_SS | ADS1118_PGA_0_256 | ADS1118_MODE_SS;
    static const uint8_t common_msb = ADS1118_PGA_0_256;
    #if ADS1118_CHANNEL(0)
        static int16_t channel_0_reading;
        static const uint8_t channel_0_config_msb = common_msb | ADS1118_MUX_SEL_0;
    #endif
    #if ADS1118_CHANNEL(1)
        static int16_t channel_1_reading;
        static const uint8_t channel_1_config_msb = common_msb | ADS1118_MUX_SEL_1;
    #endif
    #if ADS1118_CHANNEL(2)
        static int16_t channel_2_reading;
        static const uint8_t channel_2_config_msb = common_msb | ADS1118_MUX_SEL_2;
    #endif
    #if ADS1118_CHANNEL(3)
        static int16_t channel_3_reading;
        static const uint8_t channel_3_config_msb = common_msb | ADS1118_MUX_SEL_3;
    #endif
    #if ADS1118_CHANNEL(4)
        static int16_t channel_4_reading;
        static const uint8_t channel_4_config_msb = common_msb | ADS1118_MUX_SEL_4;
    #endif
    #if ADS1118_CHANNEL(5)
        static int16_t channel_5_reading;
        static const uint8_t channel_5_config_msb = common_msb | ADS1118_MUX_SEL_5;
    #endif
    #if ADS1118_CHANNEL(6)
        static int16_t channel_6_reading;
        static const uint8_t channel_6_config_msb = common_msb | ADS1118_MUX_SEL_6;
    #endif
    #if ADS1118_CHANNEL(7)
        static int16_t channel_7_reading;
        static const uint8_t channel_7_config_msb = common_msb | ADS1118_MUX_SEL_7;
    #endif
    // softspi doesn't enable pullup, so we want the ADS1118 to do so instead
    static const uint8_t common_config_lsb = ADS1118_DR_64_SPS | ADS1118_WRITE_FLAG | ADS1118_MISO_PULLUP;
    static const uint8_t reference_config_lsb = ADS1118_TS_MODE | ADS1118_DR_64_SPS | ADS1118_WRITE_FLAG | ADS1118_MISO_PULLUP;
    static uint8_t config_state;
    static uint8_t read_state;
    static uint8_t temp_check_counter;
    // ADC on left, TEMP_C on right
    static constexpr short ads1118_typek_table[21][2] = {
        { -287, ADS1118_mV_TO_STEP(-2.2430) },
        { -181, ADS1118_mV_TO_STEP(-1.4157) },
        {  -70, ADS1118_mV_TO_STEP(-0.5464) },
        {   46, ADS1118_mV_TO_STEP( 0.3573) },
        {  164, ADS1118_mV_TO_STEP( 1.2848) },
        {  285, ADS1118_mV_TO_STEP( 2.2295) },
        {  408, ADS1118_mV_TO_STEP( 3.1838) },
        {  530, ADS1118_mV_TO_STEP( 4.1373) },
        {  651, ADS1118_mV_TO_STEP( 5.0832) },
        {  770, ADS1118_mV_TO_STEP( 6.0171) },
        {  888, ADS1118_mV_TO_STEP( 6.9410) },
        { 1006, ADS1118_mV_TO_STEP( 7.8587) },
        { 1124, ADS1118_mV_TO_STEP( 8.7796) },
        { 1242, ADS1118_mV_TO_STEP( 9.7066) },
        { 1362, ADS1118_mV_TO_STEP(10.6430) },
        { 1483, ADS1118_mV_TO_STEP(11.5885) },
        { 1605, ADS1118_mV_TO_STEP(12.5410) },
        { 1728, ADS1118_mV_TO_STEP(13.4987) },
        { 1851, ADS1118_mV_TO_STEP(14.4610) },
        { 1975, ADS1118_mV_TO_STEP(15.4277) },
        { 2099, ADS1118_mV_TO_STEP(16.3970) },
    };
    static short C_to_ADC_steps(int16_t C_reading);
    // For softspi, we want the output to be low, and clock to be low, so we need lsb 1 to be true
    static SoftSPI<ADS1118_DO_PIN, ADS1118_DI_PIN, ADS1118_SCK_PIN, 1> ads1118_spi;
public:
    void init();
    uint8_t update();
    int16_t channel_raw(uint8_t channel);
    float channel_temp(uint8_t channel);
    static float ADC_steps_to_C(int16_t ADC_steps);
};

#if ENABLED(HEATER_0_USES_ADS1118)
    #define HEATER_0_RAW_HI_TEMP 2099
    #define HEATER_0_RAW_LO_TEMP -287
#endif
#if ENABLED(HEATER_1_USES_ADS1118)
    #define HEATER_1_RAW_HI_TEMP 2099
    #define HEATER_1_RAW_LO_TEMP -287
#endif
#if ENABLED(HEATER_2_USES_ADS1118)
    #define HEATER_2_RAW_HI_TEMP 2099
    #define HEATER_2_RAW_LO_TEMP -287
#endif
#if ENABLED(HEATER_3_USES_ADS1118)
    #define HEATER_3_RAW_HI_TEMP 2099
    #define HEATER_3_RAW_LO_TEMP -287
#endif
#if ENABLED(HEATER_4_USES_ADS1118)
    #define HEATER_4_RAW_HI_TEMP 2099
    #define HEATER_4_RAW_LO_TEMP -287
#endif
#if ENABLED(HEATER_BED_USES_ADS1118)
    #define HEATER_BED_RAW_HI_TEMP 2099
    #define HEATER_BED_RAW_LO_TEMP -287
#endif
#if ENABLED(HEATER_CHAMBER_USES_ADS1118)
    #define HEATER_CHAMBER_RAW_HI_TEMP 2099
    #define HEATER_CHAMBER_RAW_LO_TEMP -287
#endif

// Type K table
// { .temp_C=  -60, .mV= -2.243},
// { .temp_C=  -50, .mV= -1.889},
// { .temp_C=  -40, .mV= -1.527},
// { .temp_C=  -30, .mV= -1.156},
// { .temp_C=  -20, .mV= -0.778},
// { .temp_C=  -10, .mV= -0.392},
// { .temp_C=    0, .mV=  0.000},
// { .temp_C=   10, .mV=  0.397},
// { .temp_C=   20, .mV=  0.798},
// { .temp_C=   30, .mV=  1.203},
// { .temp_C=   40, .mV=  1.612},
// { .temp_C=   50, .mV=  2.023},
// { .temp_C=   60, .mV=  2.436},
// { .temp_C=   70, .mV=  2.851},
// { .temp_C=   80, .mV=  3.267},
// { .temp_C=   90, .mV=  3.682},
// { .temp_C=  100, .mV=  4.096},
// { .temp_C=  110, .mV=  4.509},
// { .temp_C=  120, .mV=  4.920},
// { .temp_C=  130, .mV=  5.328},
// { .temp_C=  140, .mV=  5.735},
// { .temp_C=  150, .mV=  6.138},
// { .temp_C=  160, .mV=  6.540},
// { .temp_C=  170, .mV=  6.941},
// { .temp_C=  180, .mV=  7.340},
// { .temp_C=  190, .mV=  7.739},
// { .temp_C=  200, .mV=  8.138},
// { .temp_C=  210, .mV=  8.539},
// { .temp_C=  220, .mV=  8.940},
// { .temp_C=  230, .mV=  9.343},
// { .temp_C=  240, .mV=  9.747},
// { .temp_C=  250, .mV= 10.153},
// { .temp_C=  260, .mV= 10.561},
// { .temp_C=  270, .mV= 10.971},
// { .temp_C=  280, .mV= 11.382},
// { .temp_C=  290, .mV= 11.795},
// { .temp_C=  300, .mV= 12.209},
// { .temp_C=  310, .mV= 12.624},
// { .temp_C=  320, .mV= 13.040},
// { .temp_C=  330, .mV= 13.457},
// { .temp_C=  340, .mV= 13.874},
// { .temp_C=  350, .mV= 14.293},
// { .temp_C=  360, .mV= 14.713},
// { .temp_C=  370, .mV= 15.133},
// { .temp_C=  380, .mV= 15.554},
// { .temp_C=  390, .mV= 15.975},
// { .temp_C=  400, .mV= 16.397},
// flatten
// -60, -50, -40, -30, -20, -10,   0,  10,  20,  30,  40,  50,  60,  70,  80,  90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 310, 320, 330, 340, 350, 360, 370, 380, 390, 400
// -2.243,-1.889,-1.527,-1.156,-0.778,-0.392, 0.000, 0.397, 0.798, 1.203, 1.612, 2.023, 2.436, 2.851, 3.267, 3.682, 4.096, 4.509, 4.920, 5.328, 5.735, 6.138, 6.540, 6.941, 7.340, 7.739, 8.138, 8.539, 8.940, 9.343, 9.747,10.153,10.561,10.971,11.382,11.795,12.209,12.624,13.040,13.457,13.874,14.293,14.713,15.133,15.554,15.975,16.397,
// interp1
// -60   -37   -14     9    32    55    78   101   124   147   170   193   216   239   262   285   308   331   354   377   400
// -2.2430	-1.4157	-0.5464	0.3573	1.2848	2.2295	3.1838	4.1373	5.0832	6.0171	6.9410	7.8587	8.7796	9.7066	10.6430	11.5885	12.5410	13.4987	14.4610	15.4277	16.3970


// MOVE, send's MSB first which makes life EASY
// SPI<ADS1118_DO_PIN, ADS1118_DI_PIN, ADS1118_SCK_PIN> ads1118_spi;
// ads1118_spi.transfer();

#endif
#endif // THERMOCOUPLE_ADS1118_H
