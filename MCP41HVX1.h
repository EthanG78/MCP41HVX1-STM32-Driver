/**
 *      MCP41HVX1 STM32F7 SPI Driver
 *
 *      Author:     Ethan Garnier
 *      Date:       2025
 */
#ifndef MCP41HVX1_SPI_DRIVER_H
#define MCP41HVX1_SPI_DRIVER_H

#include "stm32f7xx_hal.h" /* Needed for structure defs */

// NOTE(Ethan): This step resistance is calculated from
// the following formula:
//      Rs = (Rab - Rfs - Rzs) / FSV
//  where Rab is the total, terminal to terminal, resistance
//  of the digitpot, Rfs is the full-scale resistance, Rzs
//  is the zero-scale resistance, and FSV is the full-scale value
//  or codeword of the digipot. Here we assume that
//  Rfs = Rzs = 0 ohms. This might be really bad and we
//  may need to actually measure these values!
#define STEP_RESISTANCE 196.08f
#define FSV 255
#define R_FS 0.0f
#define R_ZS 0.0f

/* MCP41HVX1 SPI Wiper Command Bytes */
typedef enum
{
    INCR_WIPER = 0x04,
    DECR_WIPER = 0x08,
} MCP41HVX1_Wiper_Command;

/* MCP41HVX1 Sensor Struct */
typedef struct
{
    // STM32F7 HAL specific handler for SPI communication
    SPI_HandleTypeDef *spiHandle;

    // Port where the chip select GPIO pin is located
    GPIO_TypeDef *csPort;

    // 16 bit pin number of the chip select GPIO pin (active low)
    unsigned short csPin;

} MCP41HVX1;

HAL_StatusTypeDef MCP41HVX1_Init (MCP41HVX1 *MCP41HVX1,
                                  SPI_HandleTypeDef *spiHandle,
                                  GPIO_TypeDef *csPort,
                                  unsigned short csPin);
HAL_StatusTypeDef MCP41HVX1_Move_Wiper (MCP41HVX1 *mcp, MCP41HVX1_Wiper_Command cmd);
HAL_StatusTypeDef MCP41HVX1_Set_Resistance (MCP41HVX1 *mcp, float resistance);
HAL_StatusTypeDef MCP41HVX1_Get_Resistance (MCP41HVX1 *mcp, float *resistance);
HAL_StatusTypeDef MCP41HVX1_Startup (MCP41HVX1 *mcp);
HAL_StatusTypeDef MCP41HVX1_Shutdown (MCP41HVX1 *mcp);

#endif
