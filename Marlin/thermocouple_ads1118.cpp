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
 * mcp4728.cpp - Arduino library for MicroChip MCP4728 I2C D/A converter
 *
 * For implementation details, please take a look at the datasheet:
 * http://ww1.microchip.com/downloads/en/DeviceDoc/22187a.pdf
 *
 * For discussion and feedback, please go to:
 * http://arduino.cc/forum/index.php/topic,51842.0.html
 */

#include "MarlinConfig.h"

#if ENABLED(TEMP_ADS1118)

#include "thermocouple_ads1118.h"
#include "delay.h"

// Initialize statics
int16_t Ads1118::reference_compensation = 0;
#if ADS1118_CHANNEL(0)
  int16_t Ads1118::channel_0_reading = ADS1118_UNPLUGGED_BITS;
#endif
#if ADS1118_CHANNEL(1)
  int16_t Ads1118::channel_1_reading = ADS1118_UNPLUGGED_BITS;
#endif
#if ADS1118_CHANNEL(2)
  int16_t Ads1118::channel_2_reading = ADS1118_UNPLUGGED_BITS;
#endif
#if ADS1118_CHANNEL(3)
  int16_t Ads1118::channel_3_reading = ADS1118_UNPLUGGED_BITS;
#endif
#if ADS1118_CHANNEL(4)
  int16_t Ads1118::channel_4_reading = ADS1118_UNPLUGGED_BITS;
#endif
#if ADS1118_CHANNEL(5)
  int16_t Ads1118::channel_5_reading = ADS1118_UNPLUGGED_BITS;
#endif
#if ADS1118_CHANNEL(6)
  int16_t Ads1118::channel_6_reading = ADS1118_UNPLUGGED_BITS;
#endif
#if ADS1118_CHANNEL(7)
  int16_t Ads1118::channel_7_reading = ADS1118_UNPLUGGED_BITS;
#endif
uint8_t Ads1118::read_state = 0;
uint8_t Ads1118::config_state = 0;
uint8_t Ads1118::temp_check_counter = 0;
constexpr short Ads1118::ads1118_typek_table[22][2] PROGMEM;

#define TRANSFERBIT(B,n,O,m) do{\
  WRITE(ADS1118_SCK_PIN, HIGH); \
  WRITE(ADS1118_DI_PIN, TEST(B,n)); \
  DELAY_NS(50); \
  WRITE(ADS1118_SCK_PIN, LOW);\
  DELAY_NS(50); \
  if(READ(ADS1118_DO_PIN)) SBI(O,m);\
  }while(0)

#define SENDBIT(B,n) do{\
  WRITE(ADS1118_SCK_PIN, HIGH); \
  WRITE(ADS1118_DI_PIN, TEST(B,n)); \
  DELAY_NS(50); \
  WRITE(ADS1118_SCK_PIN, LOW); \
  DELAY_NS(100); \
  }while(0)

#define ADS1118_SEND(M,L) do{\
  SENDBIT(M,7); \
  SENDBIT(M,6); \
  SENDBIT(M,5); \
  SENDBIT(M,4); \
  SENDBIT(M,3); \
  SENDBIT(M,2); \
  SENDBIT(M,1); \
  SENDBIT(M,0); \
  SENDBIT(L,7); \
  SENDBIT(L,6); \
  SENDBIT(L,5); \
  SENDBIT(L,4); \
  SENDBIT(L,3); \
  SENDBIT(L,2); \
  SENDBIT(L,1); \
  SENDBIT(L,0); \
  WRITE(ADS1118_DI_PIN, LOW); \
  }while(0)

#define ADS1118_TRANSFER(M,L,O) do{ \
  TRANSFERBIT(M,7,O,15); \
  TRANSFERBIT(M,6,O,14); \
  TRANSFERBIT(M,5,O,13); \
  TRANSFERBIT(M,4,O,12); \
  TRANSFERBIT(M,3,O,11); \
  TRANSFERBIT(M,2,O,10); \
  TRANSFERBIT(M,1,O,9); \
  TRANSFERBIT(M,0,O,8); \
  TRANSFERBIT(L,7,O,7); \
  TRANSFERBIT(L,6,O,6); \
  TRANSFERBIT(L,5,O,5); \
  TRANSFERBIT(L,4,O,4); \
  TRANSFERBIT(L,3,O,3); \
  TRANSFERBIT(L,2,O,2); \
  TRANSFERBIT(L,1,O,1); \
  TRANSFERBIT(L,0,O,0); \
  WRITE(ADS1118_DI_PIN, LOW); \
  }while(0)

void Ads1118::init()
{
  // NEEDED CONFIGUCATION
  // Unselect the chip
  OUT_WRITE(ADS1118_CS, HIGH);
  // SET_INPUT_PULLUP(ADS1118_DO_PIN);
  OUT_WRITE(ADS1118_SCK_PIN, LOW);
  OUT_WRITE(ADS1118_DI_PIN, LOW);
  DELAY_NS(200);       // Ensure 200ns delay to reset SPI

  // ads1118_spi.begin();

  // Select chip
  WRITE(ADS1118_CS, LOW);
  DELAY_NS(100);       // Ensure 100ns delay to start sending

  SERIAL_ECHOLN("Initializing ADS1118");
  SERIAL_ECHOLNPAIR("ADS1118 unplugged reading: ", (int16_t)ADS1118_UNPLUGGED_BITS);
  SERIAL_ECHOLNPAIR("ADS1118 CH0 config: ", channel_0_config_msb);
  // SERIAL_ECHOLNPAIR("ADS1118 CH3 config: ", channel_3_config_msb);

  // Start the first update
  ADS1118_SEND(common_msb, reference_config_lsb);
  ADS1118_SEND(common_msb, reference_config_lsb);
  // ads1118_spi.send(common_msb);
  // ads1118_spi.send(reference_config_lsb);
  // ads1118_spi.send(common_msb);
  // ads1118_spi.send(reference_config_lsb);
  // Setup the next config
  config_state = 0xFF;

  // Done with this chip (so we can do 16 bit transmissions)
  // DELAY_NS(100);       // Ensure 100ns delay
  // WRITE(ADS1118_CS, HIGH);
}

uint8_t Ads1118::update()
{
  // Select to read
  // // SERIAL_ECHOLN("Up");
  // WRITE(ADS1118_CS, LOW);
  // DELAY_NS(100);       // Ensure 100ns delay

  // check that data ready flag is low
  // if it is high, return 1 so the calling function knows to try again later
  if (READ(ADS1118_DO_PIN)) {
    // WRITE(ADS1118_CS, HIGH);
    // SERIAL_ECHOLN("NR");
    return 1;
  }

  // uint8_t config_msb = common_msb;
  // uint8_t config_lsb = common_config_lsb;
  uint8_t last_read_state = read_state;
  int16_t raw = 0;
  uint16_t config = 0;

  // Macro to make the following expansion less repetitive
  // #define ADS1118_CONFIG_AND_READ(C)                            \
  //   config_msb = channel_##C##_config_msb;                      \
  //   raw |= ads1118_spi.transfer(config_msb);   \
  //   raw <<= 8;\
  //   raw |= ads1118_spi.transfer(config_lsb);          \
  //   ads1118_spi.transfer(config_msb); \
  //   ads1118_spi.transfer(config_lsb); \
  //   read_state = (0x01 << C);                                   \
  //   temp_check_counter++;
  #define ADS1118_CONFIG_AND_READ(C)                            \
    ADS1118_TRANSFER(channel_##C##_config_msb, common_config_lsb, raw);    \
    ADS1118_TRANSFER(0, 0, config); \
    read_state = (uint8_t)(0x01 << C);                                   \
    temp_check_counter++;
    // SERIAL_ECHOLNPAIR("sent ", channel_##C##_config_msb);  \
    // SERIAL_ECHOLNPAIR("msb ", config>>8);  \

  // If we want to do a reference reading
  if (config_state == 0) {
    // SERIAL_ECHOLN("RR");
    // config_lsb = reference_config_lsb;
    // // Send the MSB and LSB and get the data
    // raw |= ads1118_spi.transfer(config_msb);
    // raw <<= 8;
    // raw |= ads1118_spi.transfer(config_lsb);
    // ads1118_spi.transfer(config_msb);
    // ads1118_spi.transfer(config_lsb);
    ADS1118_TRANSFER(common_msb, reference_config_lsb, raw);
    ADS1118_TRANSFER(0, 0, config);
    // SERIAL_ECHOLNPAIR("sent ", reference_config_lsb);
    // SERIAL_ECHOLNPAIR("lsb ", config&0xFF);
    read_state = 0;
    // We can end early here as this is the 16-bit transmission approach
    config_state = 0xFF;
    // Temporary to keep things from being too wacky
    if (reference_config_lsb != config&0xFE) return 1;
  }
  #if ADS1118_CHANNEL(0)
    // We want to read channel 0
    else if(config_state & (0x01 << 0)) {
      ADS1118_CONFIG_AND_READ(0)
      #if (ADS1118_CHANNEL(1) || ADS1118_CHANNEL(2) || ADS1118_CHANNEL(3) \
          || ADS1118_CHANNEL(4) || ADS1118_CHANNEL(5) || ADS1118_CHANNEL(6) || ADS1118_CHANNEL(7))
        config_state = (uint8_t)(0xFF << 1);
      #else
        config_state = 0xFF;
      #endif
    }
  #endif
  #if ADS1118_CHANNEL(1)
    // We want to read channel 1
    else if(config_state & (0x01 << 1)) {
      ADS1118_CONFIG_AND_READ(1)
      #if (ADS1118_CHANNEL(2) || ADS1118_CHANNEL(3) || ADS1118_CHANNEL(4) \
          || ADS1118_CHANNEL(5) || ADS1118_CHANNEL(6) || ADS1118_CHANNEL(7))
        config_state = (uint8_t)(0xFF << 2);
      #else
        config_state = 0xFF;
      #endif
    }
  #endif
  #if ADS1118_CHANNEL(2)
    // We want to read channel 2
    else if(config_state & (0x01 << 2)) {
      ADS1118_CONFIG_AND_READ(2)
      #if (ADS1118_CHANNEL(3) || ADS1118_CHANNEL(4) || ADS1118_CHANNEL(5) \
          || ADS1118_CHANNEL(6) || ADS1118_CHANNEL(7))
        config_state = (uint8_t)(0xFF << 3);
      #else
        config_state = 0xFF;
      #endif
    }
  #endif
  #if ADS1118_CHANNEL(3)
    // We want to read channel 3
    else if(config_state & (0x01 << 3)) {
      ADS1118_CONFIG_AND_READ(3)
      #if (ADS1118_CHANNEL(4) || ADS1118_CHANNEL(5) || ADS1118_CHANNEL(6) || ADS1118_CHANNEL(7))
        config_state = (uint8_t)(0xFF << 4);
      #else
        config_state = 0xFF;
      #endif
    }
  #endif
  #if ADS1118_CHANNEL(4)
    // We want to read channel 4
    else if(config_state & (0x01 << 4)) {
      ADS1118_CONFIG_AND_READ(4)
      #if (ADS1118_CHANNEL(5) || ADS1118_CHANNEL(6) || ADS1118_CHANNEL(7))
        config_state = (0xFF << 5);
      #else
        config_state = 0xFF;
      #endif
    }
  #endif
  #if ADS1118_CHANNEL(5)
    // We want to read channel 5
    else if(config_state & (0x01 << 5)) {
      ADS1118_CONFIG_AND_READ(5)
      #if (ADS1118_CHANNEL(6) || ADS1118_CHANNEL(7))
        config_state = (uint8_t)(0xFF << 6);
      #else
        config_state = 0xFF;
      #endif
    }
  #endif
  #if ADS1118_CHANNEL(6)
    // We want to read channel 6
    else if(config_state & (0x01 << 6)) {
      ADS1118_CONFIG_AND_READ(6)
      #if ADS1118_CHANNEL(7)
        config_state = (uint8_t)(0xFF << 7);
      #else
        config_state = 0xFF;
      #endif
      temp_check_counter++;
    }
  #endif
  #if ADS1118_CHANNEL(7)
    // We want to read channel 7
    else if(config_state & (0x01 << 7)) {
      ADS1118_CONFIG_AND_READ(7)
      config_state = 0xFF;
    }
  #endif
  // SOMETHING WENT WRONG
  else {
    // ERROR OUT HARD HERE
    SERIAL_ECHOLN("BC");
  }
  // WRITE(ADS1118_CS, HIGH);

  // See if next time we're going to check the reference temperature
  if (temp_check_counter >= ADS1118_TS_CHECK_CYCLES) {
    temp_check_counter = 0;
    config_state = 0;
  }


  // Macro to make the following expansion less repetitive
  #define ADS1118_SAVE_READ(C) \
    else if (last_read_state & (0x01 << C)) { \
      channel_##C##_reading = raw;}

  // Save the raw value
  if (last_read_state == 0) {
    // Temp sensor is 14 bit
    if (raw < 0)
      raw = -((-raw) >> 2);
    else raw >>= 2;
    // Loses precision here, but we'll say that's fine for now.
    // This way we can do cold junction compensation
    // SERIAL_ECHOLN("SRR");
    reference_compensation = C_to_ADC_steps(raw * ADS1118_CONST_TEMP_C_STEP);
  }
  #if ADS1118_CHANNEL(0)
    ADS1118_SAVE_READ(0)
  #endif
  #if ADS1118_CHANNEL(1)
    ADS1118_SAVE_READ(1)
  #endif
  #if ADS1118_CHANNEL(2)
    ADS1118_SAVE_READ(2)
  #endif
  #if ADS1118_CHANNEL(3)
    ADS1118_SAVE_READ(3)
  #endif
  #if ADS1118_CHANNEL(4)
    ADS1118_SAVE_READ(4)
  #endif
  #if ADS1118_CHANNEL(5)
    ADS1118_SAVE_READ(5)
  #endif
  #if ADS1118_CHANNEL(6)
    ADS1118_SAVE_READ(6)
  #endif
  #if ADS1118_CHANNEL(7)
    ADS1118_SAVE_READ(7)
  #endif
  // SOMETHING WENT WRONG
  else {
    // ERROR OUT HARD HERE
    SERIAL_ECHOLN("BR");
  }

  // SENSOR ISN'T ATTACHED?!?!
  if (raw == (int16_t)ADS1118_UNPLUGGED_BITS) {
    // ERROR OUT HARD HERE
    SERIAL_ECHOLN("BS");
  }
  // SERIAL_ECHOLN("G");
  return 0;
}

short Ads1118::C_to_ADC_steps(int16_t C_reading)
{
  uint8_t l = 0, r = 22, m;
  for (;;) {
    m = (l + r) >> 1;
    if (m == l || m == r) return (short)pgm_read_word(&ads1118_typek_table[21][0]);
    short v00 = pgm_read_word(&ads1118_typek_table[m-1][1]),
          v10 = pgm_read_word(&ads1118_typek_table[m-0][1]);
         if (C_reading < v00) r = m;
    else if (C_reading > v10) l = m;
    else {
        const short v01 = (short)pgm_read_word(&ads1118_typek_table[m-1][0]),
                    v11 = (short)pgm_read_word(&ads1118_typek_table[m-0][0]);
        return v01 + (C_reading - v00) * float(v11 - v01) / float(v10 - v00);
    }
  }
}

float Ads1118::ADC_steps_to_C(int16_t C_reading)
{
  uint8_t l = 0, r = 22, m;
  for (;;) {
    m = (l + r) >> 1;
    if (m == l || m == r) return (short)pgm_read_word(&ads1118_typek_table[21][1]);//ads1118_typek_table[20][1];
    short v00 = pgm_read_word(&ads1118_typek_table[m-1][0]),
          v10 = pgm_read_word(&ads1118_typek_table[m-0][0]);
         if (C_reading < v00) r = m;
    else if (C_reading > v10) l = m;
    else {
        const short v01 = (short)pgm_read_word(&ads1118_typek_table[m-1][1]),
                    v11 = (short)pgm_read_word(&ads1118_typek_table[m-0][1]);
        return v01 + (C_reading - v00) * float(v11 - v01) / float(v10 - v00);
    }
  }
}

int16_t Ads1118::channel_raw(uint8_t channel)
{
  // SERIAL_ECHOLNPAIR("ch ", channel);
  switch(channel){
    #if ADS1118_CHANNEL(0)
      case 0:
        // SERIAL_ECHOLNPAIR("raw ", channel_0_reading);
        return channel_0_reading + reference_compensation;
    #endif
    #if ADS1118_CHANNEL(1)
      case 1:
        // SERIAL_ECHOLNPAIR("raw ", channel_1_reading);
        return channel_1_reading + reference_compensation;
    #endif
    #if ADS1118_CHANNEL(2)
      case 2:
        // SERIAL_ECHOLNPAIR("raw ", channel_2_reading);
        return channel_2_reading + reference_compensation;
    #endif
    #if ADS1118_CHANNEL(3)
      case 3:
        // SERIAL_ECHOLNPAIR("raw ", channel_3_reading);
        return channel_3_reading + reference_compensation;
    #endif
    #if ADS1118_CHANNEL(4)
      case 4:
        // SERIAL_ECHOLNPAIR("raw ", channel_4_reading);
        return channel_4_reading + reference_compensation;
    #endif
    #if ADS1118_CHANNEL(5)
      case 5:
        // SERIAL_ECHOLNPAIR("raw ", channel_5_reading);
        return channel_5_reading + reference_compensation;
    #endif
    #if ADS1118_CHANNEL(6)
      case 6:
        // SERIAL_ECHOLNPAIR("raw ", channel_6_reading);
        return channel_6_reading + reference_compensation;
    #endif
    #if ADS1118_CHANNEL(7)
      case 7:
        // SERIAL_ECHOLNPAIR("raw ", channel_7_reading);
        return channel_7_reading + reference_compensation;
    #endif
    default:
      return ADS1118_UNPLUGGED_BITS;
  }
}

float Ads1118::channel_temp(uint8_t channel)
{
  float ret = ADC_steps_to_C(channel_raw(channel) + reference_compensation);
  // SERIAL_ECHOLNPAIR("conv. ", ret);
  // switch(channel){
  //   #if ADS1118_CHANNEL(0)
  //     case 0:
  //       channel_0_reading = ADS1118_UNPLUGGED_BITS;
  //       break;
  //   #endif
  //   #if ADS1118_CHANNEL(1)
  //     case 1:
  //       channel_1_reading = ADS1118_UNPLUGGED_BITS;
  //       break;
  //   #endif
  //   #if ADS1118_CHANNEL(2)
  //     case 2:
  //       channel_2_reading = ADS1118_UNPLUGGED_BITS;
  //       break;
  //   #endif
  //   #if ADS1118_CHANNEL(3)
  //     case 3:
  //       channel_3_reading = ADS1118_UNPLUGGED_BITS;
  //       break;
  //   #endif
  //   #if ADS1118_CHANNEL(4)
  //     case 4:
  //       channel_4_reading = ADS1118_UNPLUGGED_BITS;
  //       break;
  //   #endif
  //   #if ADS1118_CHANNEL(5)
  //     case 5:
  //       channel_5_reading = ADS1118_UNPLUGGED_BITS;
  //       break;
  //   #endif
  //   #if ADS1118_CHANNEL(6)
  //     case 6:
  //       channel_6_reading = ADS1118_UNPLUGGED_BITS;
  //       break;
  //   #endif
  //   #if ADS1118_CHANNEL(7)
  //     case 7:
  //       channel_7_reading = ADS1118_UNPLUGGED_BITS;
  //       break;
  //   #endif
  //   default:
  //     break;
  // }
  return ret;
}

#endif // TEMP_ADS1118
