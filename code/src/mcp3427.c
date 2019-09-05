/**
 ******************************************************************************
 * @file    mcp3427.c
 * @author  Pablo Fuentes
 * @version V1.0.1
 * @date    2019
 * @brief   MCP3427 Functions
 ******************************************************************************
 */

#include "mcp3427.h"

/**
 ===============================================================================
              ##### Macro definitions #####
 ===============================================================================
 */

#define MCP3427_BASEADDR 0xD0 // 1 1 0 1 A2 A1 A0
#define MCP3427_RDY_MASK 0x80
#define MCP3427_CONT_MODE 0x10
#define MCP3427_ONESHOT_MODE 0x00

/**
 ===============================================================================
              ##### Global variables #####
 ===============================================================================
 */

static char tx_i2c_data[20];
static char rx_i2c_data[20];
static uint32_t sign_bit, sign_extend;
static int32_t result;

/**
 ===============================================================================
              ##### Public functions #####
 ===============================================================================
 */

// ---------  General calls  ------------
bool mcp3427_generalCallReset(I2C_TypeDef *I2Cx) {
  tx_i2c_data[0] = 0x06; // General Call Reset
  if (i2c_write(I2Cx, 0x00, tx_i2c_data, 1, I2C_STOP) == 1) return true;
  return false;
}

bool mcp3427_generalCallLatch(I2C_TypeDef *I2Cx) {
  tx_i2c_data[0] = 0x04; // General Call Latch
  if (i2c_write(I2Cx, 0x00, tx_i2c_data, 1, I2C_STOP) == 1) return true;
  return false;
}

bool mcp3427_generalCallConversion(I2C_TypeDef *I2Cx) {
  tx_i2c_data[0] = 0x08; // General Call Conversion
  if (i2c_write(I2Cx, 0x00, tx_i2c_data, 1, I2C_STOP) == 1) return true;
  return false;
}

// --------- Start Conversions -------------
bool mcp3427_start1ShotConv(mcp3427_t *mcp, uint8_t channel) {
  tx_i2c_data[0] = MCP3427_RDY_MASK | channel | MCP3427_ONESHOT_MODE |
                   mcp->resolution | mcp->gain;
  if (i2c_write(mcp->I2Cx, (MCP3427_BASEADDR | (mcp->addr << 1)), tx_i2c_data, 1, I2C_STOP) == 1) {
    return true;
  }
  return false;
}

bool mcp3427_startContConv(mcp3427_t *mcp, uint8_t channel) {
  tx_i2c_data[0] = MCP3427_RDY_MASK | channel | MCP3427_CONT_MODE |
                   mcp->resolution | mcp->gain;
  if (i2c_write(mcp->I2Cx, (MCP3427_BASEADDR | (mcp->addr << 1)), tx_i2c_data, 1, I2C_STOP) == 1) {
    return true;
  }
  return false;
}

// --------- Reading the adc ---------------
int32_t mcp3427_read(mcp3427_t *mcp) { // read adc digital value
  uint8_t config_byte;
  i2c_read(mcp->I2Cx, (MCP3427_BASEADDR | (mcp->addr << 1)), rx_i2c_data, 3, I2C_STOP);
  config_byte = rx_i2c_data[2]; // The 3th byte is the configuration byte

  if ((config_byte & MCP3427_RDY_MASK) != 0) { // If RDY bit is not 0, the conversion is not ready
    return MCP3427_ER_CONVNOTRDY;
  }

  switch (mcp->resolution) {
    case MCP3427_12BITS:
      sign_bit = 0x800;
      sign_extend = 0xFFFFF000;
      break;
    case MCP3427_14BITS:
      sign_bit = 0x2000;
      sign_extend = 0xFFFFC000;
      break;
    case MCP3427_16BITS:
      sign_bit = 0x8000;
      sign_extend = 0xFFFF0000;
      break;
  }

  result = (uint32_t)(rx_i2c_data[0] << 8);
  result |= rx_i2c_data[1];

  if ((result & sign_bit) != 0) { // it means a negative value
    result |= sign_extend;        // fill blank bytes for 2 complement negative number
  }

  return result;
}

double mcp3427_readMv(mcp3427_t *mcp) { // read adc in mv
  uint8_t config_byte;
  uint16_t max_code;
  double mv;
  i2c_read(mcp->I2Cx, (MCP3427_BASEADDR | (mcp->addr << 1)), rx_i2c_data, 3, I2C_STOP);
  config_byte = rx_i2c_data[2]; // The 3th byte is the configuration byte

  if ((config_byte & MCP3427_RDY_MASK) != 0) { // If RDY bit is not 0, the conversion is not ready
    return MCP3427_ER_CONVNOTRDY;
  }

  switch (mcp->resolution) {
    case MCP3427_12BITS:
      sign_bit = 0x800;
      sign_extend = 0xFFFFF000;
      max_code = 2048; // 2^12/2 = 2048, because of the negative range
      break;
    case MCP3427_14BITS:
      sign_bit = 0x2000;
      sign_extend = 0xFFFFC000;
      max_code = 8192; // 2^14/2 = 8192, because of the negative range
      break;
    case MCP3427_16BITS:
      sign_bit = 0x8000;
      sign_extend = 0xFFFF0000;
      max_code = 32768; // 2^16/2 = 32768, because of the negative range
      break;
  }

  result = (uint32_t)(rx_i2c_data[0] << 8);
  result |= rx_i2c_data[1];

  if ((result & sign_bit) != 0) { // it means a negative value
    result |= sign_extend;        // fill blank bytes for 2 complement negative number
  }

  mv = (double) (result * 2048) / (max_code * (1 << (mcp->gain)));

  return mv;
}

// ----------- Start conversion and read (One Shot) ------------
int32_t mcp3427_convRead(mcp3427_t *mcp, uint8_t channel) {
  if (mcp3427_start1ShotConv(mcp, channel) != true)
    return MCP3427_ER_NORESPONSE;
  switch (mcp->resolution) {
    // According to the datasheet, when resolution is 12 bits, there are 240
    // SPS, so there is 1 sample per 4.17 ms
    case MCP3427_12BITS: delay(5); break;
    // According to the datasheet, when resolution is 14 bits, there are 60 SPS,
    // so there is 1 sample per 16.67 ms
    case MCP3427_14BITS: delay(17); break;
    // According to the datasheet, when resolution is 16 bits, there are 15 SPS,
    // so there is 1 sample per 66.67 ms
    case MCP3427_16BITS: delay(67); break;
  }
  return mcp3427_read(mcp);
}

double mcp3427_convReadMv(mcp3427_t *mcp, uint8_t channel) {
  if (mcp3427_start1ShotConv(mcp, channel) != true)
    return MCP3427_ER_NORESPONSE;
  switch (mcp->resolution) {
    // According to the datasheet, when resolution is 12 bits, there are 240
    // SPS, so there is 1 sample per 4.17 ms
    case MCP3427_12BITS: delay(5); break;
    // According to the datasheet, when resolution is 14 bits, there are 60 SPS,
    // so there is 1 sample per 16.67 ms
    case MCP3427_14BITS: delay(17); break;
    // According to the datasheet, when resolution is 16 bits, there are 15 SPS,
    // so there is 1 sample per 66.67 ms
    case MCP3427_16BITS: delay(67); break;
  }
  return mcp3427_readMv(mcp);
}