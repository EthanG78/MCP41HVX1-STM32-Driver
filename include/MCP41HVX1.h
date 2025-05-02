/**
 *      MCP41HVX1 STM32F7 SPI Driver
 * 
 *      Author:     Ethan Garnier
 *      Date:       2025
*/
#ifndef MCP41HVX1_SPI_DRIVER_H
#define MCP41HVX1_SPI_DRIVER_H

#include "stm32f7xx_hal.h" /* Needed for structure defs */
#include "stdint.h"

#define ADS1256_ID      3
#define VREF            5
#define BIT_RANGE       0x7FFFFF
#define MASTER_CLK      7680000UL

#define SELECT_CHIP(device) \
    device.csPort.BSRR = (uint32_t)device.csPin << 16;

/* MCP41HVX1 Register Addresses */
typedef enum
{
    WIPER           =   0x00,
    TCON            =   0x04,
} MCP41HVX1_Register;

/* MCP41HVX1 SPI Command Bytes */
typedef enum
{
    WRITE_DATA      =   0b00,
    INCR_WIPER      =   0b01,
    DECR_WIPER      =   0b10,
    READ_DATA       =   0b11,
} MCP41HVX1_Command;

/* MCP41HVX1 Sensor Struct */
typedef struct
{ 
    // STM32F7 HAL specific handler for SPI communication
    SPI_HandleTypeDef *spiHandle;

    // Port where the chip select GPIO pin is located
    GPIO_TypeDef *csPort;

    // 16 bit pin number of the chip select GPIO pin (active low)
    uint16_t csPin;

} MCP41HVX1;


HAL_StatusTypeDef MCP41HVX1_Increment_Resistance(MCP41HVX1 *mcp);
HAL_StatusTypeDef MCP41HVX1_Decrement_Resistance(MCP41HVX1 *mcp);
HAL_StatusTypeDef MCP41HVX1_Set_Resistance(MCP41HVX1 *mcp, float resistance);
float MCP41HVX1_Get_Resistance(MCP41HVX1 *mcp);

#endif
