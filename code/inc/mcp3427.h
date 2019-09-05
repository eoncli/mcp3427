/**
 ******************************************************************************
 * @file    mcp3427.h
 * @author  Pablo Fuentes
 * @version V1.0.1
 * @date    2019
 * @brief   Header de MCP3427 Library
 ******************************************************************************
 */

#ifndef __MCP3427_H
#define __MCP3427_H

#include "eonOS.h"

// MCP3427 Channels
#define MCP3427_CH1 0x00
#define MCP3427_CH2 0x20

// MCP3427 Resolution
#define MCP3427_12BITS 0x00
#define MCP3427_14BITS 0x04
#define MCP3427_16BITS 0x08

// MCP3427 Gains
#define MCP3427_GAIN1 0x00
#define MCP3427_GAIN2 0x01
#define MCP3427_GAIN4 0x02
#define MCP3427_GAIN8 0x03

// MCP3427 Errors
#define MCP3427_ER 60000
#define MCP3427_ER_CONVNOTRDY 60001
#define MCP3427_ER_NORESPONSE 60002

typedef struct {
  I2C_TypeDef *I2Cx;
  uint8_t addr;
  uint8_t resolution;
  uint8_t gain;
} mcp3427_t;

/**
 ===============================================================================
              ##### FUNCIONES #####
 ===============================================================================
 */

// ==== General calls
// All devices will abort the current conversion an perform an internal reset
// and latch the logic status of external address pins
bool mcp3427_generalCallReset(I2C_TypeDef *I2Cx);
// All devices will latch the logic state of the external address pins
bool mcp3427_generalCallLatch(I2C_TypeDef *I2Cx);
// All devices will start a one shot conversion
bool mcp3427_generalCallConversion(I2C_TypeDef *I2Cx);

// === Start Conversions
bool mcp3427_start1ShotConv(mcp3427_t *mcp, uint8_t channel);
bool mcp3427_startContConv(mcp3427_t *mcp, uint8_t channel);

// === Read the adc
int32_t mcp3427_read(mcp3427_t *mcp);  // read adc digital value
double mcp3427_readMv(mcp3427_t *mcp); // read adc in mv

// === Start conversion and read (One Shot)
// start one shot conversion and read adc digital value
int32_t mcp3427_convRead(mcp3427_t *mcp, uint8_t channel);
// start one shot conversion and read adc in mv
double mcp3427_convReadMv(mcp3427_t *mcp, uint8_t channel);

#endif