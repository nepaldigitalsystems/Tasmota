/*
  xnrg_31_mcp39f511.ino - MCP39F511 energy sensor support for Tasmota

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

#define XNRG_31 31

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
  uint16_t dio_configuration;
  uint32_t range;

  uint32_t calibration_current;
  uint16_t calibration_voltage;
  uint32_t calibration_active_power;
  uint32_t calibration_reactive_power;
  uint16_t accumulation_interval;
} mcp_f511_cal_registers_type;


char *mcp_f511_buffer = nullptr;
unsigned long mcp_f511_window = 0;
unsigned long mcp_f511_kWhcounter = 0;
uint32_t mcp_f511_system_configuration = 0x03000000;
uint32_t mcp_f511_active_power;
//uint32_t mcp_f511_reactive_power;
//uint32_t mcp_f511_apparent_power;
uint32_t mcp_f511_current_rms;
uint16_t mcp_f511_voltage_rms;
uint16_t mcp_f511_line_frequency;
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
  if (mcp_timeout) { return; }
  mcp_timeout = MCP_F511_TIMEOUT;

  data[0] = MCP_F511_CMD_START_FRAME;
  data[data[1] -1] = McpF511Checksum(data);

//  AddLogBuffer(LOG_LEVEL_DEBUG_MORE, data, data[1]);

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
#if 0 // Does not seems applicable for F511
void McpF511GetCalibration(void)
{
  if (mcp_f511_calibration_active) { return; }
  mcp_f511_calibration_active = MCP_F511_CALIBRATION_TIMEOUT;

  uint8_t data[] = { MCP_F511_CMD_START_FRAME, 8, MCP_F511_CMD_SET_ADDRESS, (MCP_CALIBRATION_BASE >> 8) & 0xFF, MCP_CALIBRATION_BASE & 0xFF, MCP_READ, MCP_CALIBRATION_LEN, 0x00 };

  McpSend(data);
}
#endif 

#endif // USE_MCP39F511

#endif // USE_ENERGY_SENSOR