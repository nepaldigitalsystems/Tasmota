/*
  xnrg_33_mcp39f511.ino - MCP39F511 energy sensor support for Tasmota

  Copyright (C) 2024  Theo Arends

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_ENERGY_SENSOR
#ifdef USE_MCP39F511
/*********************************************************************************************\
 * MCP39F511 - Energy
 * Based on datasheet from https://www.microchip.com/en-us/product/MCP39F511
\*********************************************************************************************/

#define XNRG_33 33

#define MCP_F511_BAUDRATE 115200
#define MCP_F511_TIMEOUT 4
#define MCP_F511_CALIBRATION_TIMEOUT 2

/**
 *  MCP39F511 INSTRUCTION SET
 */
#define MCP_F511_CMD_START_FRAME 0xA5
#define MCP_F511_CMD_ACK_FRAME 0x06
#define MCP_F511_CMD_ERROR_NAK 0x15
#define MCP_F511_CMD_ERROR_CRC 0x51
#define MCP_F511_CMD_SINGLE_WIRE 0xAB // TO BE CONFIRMED 
#define MCP_F511_CMD_SET_ADDRESS 0x41   // Set Address Pointer
#define MCP_F511_CMD_READ_N 0x4E    // Register Read, N bytes
#define MCP_F511_CMD_READ_16 0x52
#define MCP_F511_CMD_READ_32 0x44
#define MCP_F511_CMD_WRITE_N 0x4D   // Register Write, N bytes
#define MCP_F511_CMD_WRITE_16 0x57
#define MCP_F511_CMD_WRITE_32 0x45
#define MCP_F511_CMD_SAVE_REGS_FLASH 0x53 // Save Registers To Flash

/**
 *  MCP39F511 REGISTER MAP
 */
// System Info and versions
#define MCP_F511_REG_SYS_STATUS 0x0002
#define MCP_F511_REG_SYS_VER 0x0004

// Power related parameters
#define MCP_F511_REG_VOLTAGE_RMS 0x0006
#define MCP_F511_REG_LINE_FREQ 0x0008
#define MCP_F511_REG_ANALOG_IN_VOLTAGE 0x000A
#define MCP_F511_REG_POWER_FACTOR 0x000C
#define MCP_F511_REG_CURRNET_RMS 0x000E
#define MCP_F511_REG_ACTIVE_PWR 0x0012
#define MCP_F511_REG_REACTIVE_PWR 0x0016
#define MCP_F511_REG_APPARENT_PWR 0x001A

// Calibration related
#define MCP_F511_REG_CALIBRATE_CURRENT 0x0086
#define MCP_F511_REG_CALIBRATE_POWER_ACTIVE 0x008C
#define MCP_F511_REG_CALIBRATE_POWER_REACTIVE 0x0090

// #define MCP_CALIBRATE_POWER     0x001

// #define MCP_CALIBRATE_VOLTAGE   0x002
// #define MCP_CALIBRATE_CURRENT   0x004
// #define MCP_CALIBRATE_FREQUENCY 0x008
// #define MCP_SINGLE_WIRE_FLAG    0x100

// TBD
#define MCP_F511_CMD_CALIBRATION_BASE 0x0060
#define MCP_F511_CMD_CALIBRATION_LEN 52
#define MCP_F511_CMD_FREQUENCY_REF_BASE 0x0094
#define MCP_F511_CMD_FREQUENCY_GAIN_BASE 0x00AE
#define MCP_F511_CMD_FREQUENCY_LEN 4

#define MCP_F511_BUFFER_SIZE 60

#include <TasmotaSerial.h>
TasmotaSerial *McpF511Serial = nullptr;

typedef struct mcp_f511_cal_registers_type {
  uint16_t gain_current_rms;
  uint16_t gain_voltage_rms;
  uint16_t gain_active_power;
  uint16_t gain_reactive_power;
  sint32_t offset_current_rms;
  sint32_t offset_active_power;
  sint32_t offset_reactive_power;
  sint16_t dc_offset_current;
  sint16_t phase_compensation;
  uint16_t apparent_power_divisor;

  uint32_t system_configuration;
  uint16_t event_configuration;
  uint32_t range;

  uint32_t calibration_current;
  uint16_t calibration_voltage;
  uint32_t calibration_power_active;
  uint32_t calibration_power_reactive;
  uint16_t line_frequency_reference;
  uint16_t accumulation_interval;
  uint16_t voltage_sag_limit;
  uint16_t voltage_surge_limit;
  uint16_t over_current_limit;
  uint16_t over_power_limit;
} mcp_f511_cal_registers_type;


char *mcp_f511_buffer = nullptr;
unsigned long mcp_f511_window = 0;
unsigned long mcp_f511_kWhcounter = 0;
uint32_t mcp_f511_system_configuration = 0x1B000000;
uint32_t mcp_f511_active_power;
uint32_t mcp_f511_reactive_power;
uint32_t mcp_f511_apparent_power;
uint32_t mcp_f511_sys_version;
uint32_t mcp_f511_current_rms;
uint16_t mcp_f511_voltage_rms;
uint16_t mcp_f511_line_frequency;
uint16_t mcp_f511_adc_op;
//sint16_t mcp_f511_power_factor;
uint8_t mcp_f511_address = 0;
uint8_t mcp_f511_calibration_active = 0;
uint8_t mcp_f511_init = 0;
uint8_t mcp_f511_timeout = 0;
uint8_t mcp_f511_calibrate = 0;
uint8_t mcp_f511_byte_counter = 0;


uint8_t McpF511Checksum(uint8_t *data)
{
  uint8_t checksum = 0;
  uint8_t offset = 0;
  uint8_t len = data[1] -1;

  for (uint32_t i = offset; i < len; i++) { checksum += data[i];	}
  return checksum;
}

unsigned long McpF511ExtractInt(char *data, uint8_t offset, uint8_t size)
{
	unsigned long result = 0;
	unsigned long pow = 1;

	for (uint32_t i = 0; i < size; i++) {
		result = result + (uint8_t)data[offset + i] * pow;
		pow = pow * 256;
	}
	return result;
}

void McpF511SetInt(unsigned long value, uint8_t *data, uint8_t offset, size_t size)
{
	for (uint32_t i = 0; i < size; i++) {
		data[offset + i] = ((value >> (i * 8)) & 0xFF);
	}
}

void McpF511Send(uint8_t *data)
{
  if (mcp_f511_timeout) { return; }
  mcp_f511_timeout = MCP_F511_TIMEOUT;

  data[0] = MCP_F511_CMD_START_FRAME;
  data[data[1] -1] = McpF511Checksum(data);

 AddLogBuffer(LOG_LEVEL_INFO, data, data[1]);

  for (uint32_t i = 0; i < data[1]; i++) {
    McpF511Serial->write(data[i]);
  }
}

/********************************************************************************************/

void McpF511GetAddress(void)
{
  uint8_t data[] = { MCP_F511_CMD_START_FRAME, 7, MCP_F511_CMD_SET_ADDRESS, 0x00, 0x26, MCP_F511_CMD_READ_16, 0x00 };

  McpF511Send(data);
}

void McpF511AddressReceive(void)
{
  // 06 05 004D 58
  mcp_f511_address = mcp_f511_buffer[3];
}

/********************************************************************************************/
#if 0 // TBD
void McpF511GetCalibration(void)
{
   // TODO
}

void McpF511ParseCalibration(void)
{
  // TODO

}

bool McpCalibrationCalc(struct mcp_cal_registers_type *cal_registers, uint8_t range_shift)
{
  // TODO

  return true;
}

void McpSetCalibration(struct mcp_cal_registers_type *cal_registers)
{
    // TODO
}

#endif 

void McpF511GetFrequency(void)
{
  // TODO
}

void McpF511SetSystemConfiguration(uint16 interval)
{
  // A5 11 41 00 42 45 03 00 01 00 41 00 5A 57 00 06 7A
  uint8_t data[17];

  data[ 1] = sizeof(data);
  data[ 2] = MCP_F511_CMD_SET_ADDRESS;                          // Set address pointer
  data[ 3] = 0x00;                                     // address
  data[ 4] = 0x7A;                                     // address
  data[ 5] = MCP_F511_CMD_WRITE_32;                             // Write 4 bytes
  data[ 6] = (mcp_f511_system_configuration >> 24) & 0xFF;  // system_configuration
  data[ 7] = (mcp_f511_system_configuration >> 16) & 0xFF;  // system_configuration
  data[ 8] = (mcp_f511_system_configuration >>  8) & 0xFF;  // system_configuration
  data[ 9] = (mcp_f511_system_configuration >>  0) & 0xFF;  // system_configuration
  data[10] = MCP_F511_CMD_SET_ADDRESS;                          // Set address pointer
  data[11] = 0x00;                                     // address
  data[12] = 0x9E;                                     // address
  data[13] = MCP_WRITE_16;                             // Write 2 bytes
  data[14] = (interval >>  8) & 0xFF;                  // interval
  data[15] = (interval >>  0) & 0xFF;                  // interval

  McpF511Send(data);
}

void McpF511GetData(void)
{
  uint8_t data[] = { MCP_F511_CMD_START_FRAME, 8, MCP_F511_CMD_SET_ADDRESS, 0x00, 0x04, MCP_F511_CMD_READ_N, 26, 0x00 };

  McpF511Send(data);
}

void McpF511ParseData(void)
{
  //  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24
  // 06 19 61 06 00 00 FE 08 9B 0E 00 00 0B 00 00 00 97 0E 00 00 FF 7F 0C C6 35
  // 06 19 CE 18 00 00 F2 08 3A 38 00 00 66 00 00 00 93 38 00 00 36 7F 9A C6 B7
  // Ak Ln Sys Version - Volt- Line Freq - A In - PF - Current - ActivePower ReActivePow ApparentPow Ck
    mcp_f511_sys_version = McpF511ExtractInt(mcp_f511_buffer,  2, 2);
    mcp_f511_voltage_rms    = McpF511ExtractInt(mcp_f511_buffer,  4, 2);
    mcp_f511_line_frequency   = McpF511ExtractInt(mcp_f511_buffer,  6, 2);
    mcp_f511_adc_op    = McpF511ExtractInt(mcp_f511_buffer,  8, 2);
    // mcp_f511_power_factor   = McpF511ExtractInt(mcp_f511_buffer, 10, 2);
    mcp_f511_current_rms    = McpF511ExtractInt(mcp_f511_buffer,  12, 4);
    mcp_f511_active_power = McpF511ExtractInt(mcp_f511_buffer, 16, 4);
    mcp_f511_reactive_power = McpF511ExtractInt(mcp_f511_buffer, 20, 4);
    mcp_f511_apparent_power = McpF511ExtractInt(mcp_f511_buffer, 24, 4);

    AddLog(LOG_LEVEL_INFO, PSTR(" ########### MCP39F511 Data Readings ###############"));
    // AddLog(LOG_LEVEL_INFO, PSTR("System Version: 0%x"), mcp_f511_sys_version);
    AddLog(LOG_LEVEL_INFO, PSTR("Voltage RMS: %03f V"), (float)(mcp_f511_voltage_rms / 10.0));
    AddLog(LOG_LEVEL_INFO, PSTR("Line Frequency: %03f Hz"), (float)(mcp_f511_line_frequency / 1000.0));
    // AddLog(LOG_LEVEL_INFO, PSTR("ADC Output: %d", mcp_f511_adc_op));
    // AddLog(LOG_LEVEL_INFO, PSTR("Power Factor: %f", mcp_f511_power_factor));
    AddLog(LOG_LEVEL_INFO, PSTR("Current RMS: %03f A"), (float)(mcp_f511_current_rms / 10000.0));
    AddLog(LOG_LEVEL_INFO, PSTR("Active Power: %03f W"), (float)(mcp_f511_active_power / 100.0));
    // AddLog(LOG_LEVEL_INFO, PSTR("Reactive Power: %f VAR", mcp_f511_reactive_power));
    // AddLog(LOG_LEVEL_INFO, PSTR("Apparent Power: %f VA", mcp_f511_apparent_power));

    AddLog(LOG_LEVEL_INFO, PSTR(" ##################################################"));


  if (Energy->power_on) {  // Powered on
    Energy->data_valid[0] = 0;
    Energy->frequency[0] = (float)mcp_f511_line_frequency / 1000;
    Energy->voltage[0] = (float)mcp_f511_voltage_rms / 10;
    Energy->active_power[0] = (float)mcp_f511_active_power / 100;
    if (0 == Energy->active_power[0]) {
      Energy->current[0] = 0;
    } else {
      Energy->current[0] = (float)mcp_f511_current_rms / 10000;
    }
/*
  } else {  // Powered off
    Energy->data_valid[0] = ENERGY_WATCHDOG;
*/
  }
}

void McpF511SerialInput(void)
{
  while ((McpF511Serial->available()) && (mcp_f511_byte_counter < MCP_F511_BUFFER_SIZE)) {
    yield();
    mcp_f511_buffer[mcp_f511_byte_counter++] = McpF511Serial->read();
    mcp_f511_window = millis();
  }

  // Ignore until non received after 2 chars (= 12 bits/char) time
  if ((mcp_f511_byte_counter) && (millis() - mcp_f511_window > (24000 / MCP_F511_BAUDRATE) +1)) {
    AddLogBuffer(LOG_LEVEL_DEBUG_MORE, (uint8_t*)mcp_f511_buffer, mcp_f511_byte_counter);

    if (MCP_F511_BUFFER_SIZE == mcp_f511_byte_counter) {
//      AddLog(LOG_LEVEL_DEBUG, PSTR("MCP: Overflow"));
    }
    else if (1 == mcp_f511_byte_counter) {
      if (MCP_F511_CMD_ERROR_CRC == mcp_f511_buffer[0]) {
//        AddLog(LOG_LEVEL_DEBUG, PSTR("MCP: Send " D_CHECKSUM_FAILURE));
        mcp_f511_timeout = 0;
      }
      else if (MCP_F511_CMD_ERROR_NAK == mcp_f511_buffer[0]) {
//        AddLog(LOG_LEVEL_DEBUG, PSTR("MCP: NAck"));
        mcp_f511_timeout = 0;
      }
    }
    else if (MCP_F511_CMD_ACK_FRAME == mcp_f511_buffer[0]) {
      if (mcp_f511_byte_counter == mcp_f511_buffer[1]) {

        if (McpF511Checksum((uint8_t *)mcp_f511_buffer) != mcp_f511_buffer[mcp_f511_byte_counter -1]) {
          AddLog(LOG_LEVEL_DEBUG, PSTR("MCP: " D_CHECKSUM_FAILURE));
        } else {
          if (5 == mcp_f511_buffer[1]) { McpF511AddressReceive(); }
          if (29 == mcp_f511_buffer[1]) { McpF511ParseData(); }
          // TODO
          // if (MCP_CALIBRATION_LEN + 3 == mcp_f511_buffer[1]) { McpF511ParseCalibration(); }
          // if (MCP_FREQUENCY_LEN + 3 == mcp_f511_buffer[1]) { McpF511ParseFrequency(); }
        }

      }
      mcp_f511_timeout = 0;
    }

    mcp_f511_byte_counter = 0;
    McpF511Serial->flush();
  }
}

void McpF511EverySecond(void)
{
  if (Energy->data_valid[0] > ENERGY_WATCHDOG) {
    mcp_f511_voltage_rms = 0;
    mcp_f511_current_rms = 0;
    mcp_f511_active_power = 0;
    mcp_f511_line_frequency = 0;
  }

  if (mcp_f511_active_power) {
    Energy->kWhtoday_delta[0] += ((mcp_f511_active_power * 10) / 36);
    EnergyUpdateToday();
  }

  if (mcp_f511_timeout) {
    mcp_f511_timeout--;
  }
  // TODO
  // else if (mcp_calibration_active) {
  //   mcp_calibration_active--;
  // }
  // else if (mcp_f511_init) {
  //   if (2 == mcp_f511_init) {
  //     McpF511GetCalibration();           // Get calibration parameters and disable SingleWire mode if enabled
  //   }
  //   else if (1 == mcp_f511_init) {
  //     McpF511GetFrequency();             // Get calibration parameter
  //   }
  //   mcp_f511_init--;
  // }
  else if (!mcp_f511_address) {
    McpF511GetAddress();                 // Get device address for future calibration changes
  }
  else {
    McpF511GetData();                    // Get energy data
  }
  AddLog(LOG_LEVEL_INFO, PSTR("MCP39F511 Readings are being updated !"));
}

void McpF511SnsInit(void)
{
  // Software serial init needs to be done here as earlier (serial) interrupts may lead to Exceptions
  McpF511Serial = new TasmotaSerial(Pin(GPIO_MCP39F511_RX), Pin(GPIO_MCP39F511_TX), 1);
  if (McpF511Serial->begin(MCP_F511_BAUDRATE)) {
    if (McpF511Serial->hardwareSerial()) {
      ClaimSerial();
      mcp_f511_buffer = TasmotaGlobal.serial_in_buffer;  // Use idle serial buffer to save RAM
    } else {
      mcp_f511_buffer = (char*)(malloc(MCP_F511_BUFFER_SIZE));
    }
#ifdef ESP32
    AddLog(LOG_LEVEL_INFO, PSTR("MCP: Serial UART%d"), McpF511Serial->getUart());
#endif
    DigitalWrite(GPIO_MCP39F511_RST, 0, 1);  // MCP enable
    Energy->use_overtemp = true;            // Use global temperature for overtemp detection
    AddLog(LOG_LEVEL_INFO, PSTR("MCP39F511 has been Initialized !"));
  } else {
    TasmotaGlobal.energy_driver = ENERGY_NONE;
    AddLog(LOG_LEVEL_INFO, PSTR("MCP39F511 failed to initialize!"));
  }
  
}

void McpF511DrvInit(void)
{
  if (PinUsed(GPIO_MCP39F511_RX) && PinUsed(GPIO_MCP39F511_TX)) {
    if (PinUsed(GPIO_MCP39F511_RST)) {
      pinMode(Pin(GPIO_MCP39F511_RST), OUTPUT);
      digitalWrite(Pin(GPIO_MCP39F511_RST), 0);  // MCP disable - Reset Delta Sigma ADC's
    }
    mcp_f511_calibrate = 0;
    mcp_f511_timeout = 2;               // Initial wait
    mcp_f511_init = 2;                  // Initial setup steps
    TasmotaGlobal.energy_driver = XNRG_33;
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xnrg33(uint32_t function)
{
  bool result = false;
  switch (function) {
    case FUNC_LOOP:
      if (McpF511Serial) { McpF511SerialInput(); }
      break;
    case FUNC_ENERGY_EVERY_SECOND:
      if (McpF511Serial) { McpF511EverySecond(); }
      break;
    case FUNC_COMMAND:
      // result = McpF511Command();
      AddLog(LOG_LEVEL_INFO, PSTR("Command not yet enabled for MCP39F511!"));
      break;
    case FUNC_INIT:
      McpF511SnsInit();
      break;
    case FUNC_PRE_INIT:
      McpF511DrvInit();
      break;
  }
  // return result;
  return true;
}
#endif // USE_MCP39F511

#endif // USE_ENERGY_SENSOR