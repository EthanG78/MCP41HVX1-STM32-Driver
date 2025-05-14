/**
 *      MCP41HVX1 STM32F7 SPI Driver
 *
 *      Author:     Ethan Garnier
 *      Date:       2025
 */
#include "MCP41HVX1.h"
#include "stdint.h"

#define __MCP_SELECT(__MCP__) (__MCP__)->csPort->BSRR = (uint32_t)((__MCP__)->csPin << 16)
#define __MCP_UNSELECT(__MCP__) (__MCP__)->csPort->BSRR = (__MCP__)->csPin

/**
 *  HAL_StatusTypeDef _spi_disable(SPI_HandleTypeDef *spiHandle)
 *
 *  Disable the provided STM32 SPI handler based on the standard
 *  SPI master procedure outlined in Section 32.5.9 of Reference
 *  Manual 0385.
 *
 *  Returns a HAL_StatusTypeDef indicating success or failure.
 */
static HAL_StatusTypeDef
_spi_disable (SPI_HandleTypeDef *spiHandle)
{
    // 1. Wait for FIFO Tx buffer to finish transmitting
    // by waiting until FTLVL[1:0] is 0b00
    while ((spiHandle->Instance->SR & 0x1800) == 0x1800)
        ;

    // 2. Wait until BSY flag is 0
    while ((spiHandle->Instance->SR & 0x0080) == 0x0080)
        ;

    // 3. Disable SPI by clearing SPE bit (bit 6)
    spiHandle->Instance->CR1 &= ~0x0040;

    // 4. Flush FIFO Rx buffer until FRLVL[1:0] is 0b00
    volatile uint8_t *tempRx = (volatile uint8_t *)&spiHandle->Instance->DR;
    volatile uint8_t tempReg = 0x00;
    while ((spiHandle->Instance->SR & 0x0600) == 0x0600)
    {
        tempReg = *tempRx;
        (void)tempReg; // Avoids GCC unused warning
    }

    return HAL_OK;
}

static float
_code_to_resistance (uint8_t code)
{
    return MCP_R_FS + (float)(MCP_FSV - code) * MCP_STEP_RESISTANCE;
}

static uint8_t
_resistance_to_code (float resistance)
{
    return MCP_FSV - (uint8_t)((resistance - MCP_R_FS) / MCP_STEP_RESISTANCE);
}

HAL_StatusTypeDef
MCP41HVX1_Init (MCP41HVX1 *MCP41HVX1,
                SPI_HandleTypeDef *spiHandle,
                GPIO_TypeDef *csPort,
                uint16_t csPin)
{
    MCP41HVX1->spiHandle = spiHandle;
    MCP41HVX1->csPort = csPort;
    MCP41HVX1->csPin = csPin;

    // TODO(Ethan): Other startup stuff?

    return HAL_OK;
}

HAL_StatusTypeDef
MCP41HVX1_Move_Wiper (MCP41HVX1 *mcp, MCP41HVX1_Wiper_Command cmd)
{
    __HAL_LOCK (mcp->spiHandle);
    __MCP_SELECT (mcp);

    // Enable SPI by setting SPE bit (bit 6)
    mcp->spiHandle->Instance->CR1 |= 0x0040;

    // Set the RXNE event to fire when Rx buffer is 1/4 full (8 bits)
    mcp->spiHandle->Instance->CR2 |= 0x1000;

    // Wait until the SPI transmit buffer is empty
    while (!(mcp->spiHandle->Instance->SR & 0x0002))
        ;

    // Send the specified command to the SPI data register
    *((volatile uint8_t *)(&(mcp->spiHandle->Instance->DR))) = (uint8_t)cmd;

    // Wait until the receive buffer is full and read it
    while (!(mcp->spiHandle->Instance->SR & 0x0001))
        ;

    // Store the 8 bit value received by MCP on command
    uint8_t rx = *(volatile uint8_t *)(&(mcp->spiHandle->Instance->DR));

    _spi_disable (mcp->spiHandle);

    __MCP_UNSELECT (mcp);
    __HAL_UNLOCK (mcp->spiHandle);

    // TODO(Ethan): What order do we receive from MCP?
    // If CMDERR (bit 7) is low, then an error has occured
    return (rx & 0x02) ? HAL_OK : HAL_ERROR;
}

// TODO(Ethan): Test 16-bit receives. Do we need to break it up?
HAL_StatusTypeDef
MCP41HVX1_Set_Resistance (MCP41HVX1 *mcp, float resistance)
{
    uint8_t resistanceCode = _resistance_to_code (resistance);

    __HAL_LOCK (mcp->spiHandle);
    __MCP_SELECT (mcp);

    // Enable SPI by setting SPE bit (bit 6)
    mcp->spiHandle->Instance->CR1 |= 0x0040;

    // TODO(Ethan): Test that this actually works!
    // Set the RXNE event to fire when Rx buffer is 1/2 full (16 bits)
    mcp->spiHandle->Instance->CR2 &= ~0x1000;

    // Wait until the SPI transmit buffer is empty
    while (!(mcp->spiHandle->Instance->SR & 0x0002))
        ;

    // Send the write data command for the wiper register
    *((volatile uint8_t *)(&(mcp->spiHandle->Instance->DR))) = (uint8_t)0x00;

    // Wait until the SPI transmit buffer is empty
    while (!(mcp->spiHandle->Instance->SR & 0x0002))
        ;

    // Send the data to be written to the register
    *((volatile uint8_t *)(&(mcp->spiHandle->Instance->DR))) = resistanceCode;

    // Wait until the receive buffer is full and read it
    while (!(mcp->spiHandle->Instance->SR & 0x0001))
        ;

    // Store the 16 bit value received by MCP on command
    uint16_t rx = *(volatile uint16_t *)(&(mcp->spiHandle->Instance->DR));

    _spi_disable (mcp->spiHandle);

    __MCP_UNSELECT (mcp);
    __HAL_UNLOCK (mcp->spiHandle);

    // TODO(Ethan): What order do we receive from MCP?
    // If CMDERR (bit 7) is low, then an error has occured
    return (rx & 0x0200) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef
MCP41HVX1_Get_Resistance (MCP41HVX1 *mcp, float *resistance)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    __HAL_LOCK (mcp->spiHandle);
    __MCP_SELECT (mcp);

    // Enable SPI by setting SPE bit (bit 6)
    mcp->spiHandle->Instance->CR1 |= 0x0040;

    // Set the RXNE event to fire when Rx buffer is 1/4 full (8 bits)
    mcp->spiHandle->Instance->CR2 |= 0x1000;

    // Wait until the SPI transmit buffer is empty
    while (!(mcp->spiHandle->Instance->SR & 0x0002))
        ;

    // Send the read data command
    *((volatile uint8_t *)(&(mcp->spiHandle->Instance->DR))) = (uint8_t)0x0C;

    // Wait until the receive buffer is full and read it
    while (!(mcp->spiHandle->Instance->SR & 0x0001))
        ;

    // Store the first 8 bits returned from the MCP and check for error
    uint8_t rx = *(volatile uint8_t *)(&(mcp->spiHandle->Instance->DR));
    if (rx & 0x02)
    {
        // No error, continue with read

        // Wait until the SPI transmit buffer is empty
        while (!(mcp->spiHandle->Instance->SR & 0x0002))
            ;

        // Send dummy clocks
        *((volatile uint8_t *)(&(mcp->spiHandle->Instance->DR))) = 0x00;

        // Wait until the receive buffer is full and read it
        while (!(mcp->spiHandle->Instance->SR & 0x0001))
            ;

        // Read the returned resistance code from the MCP and
        // convert into a floating point resistance
        uint8_t resistanceCode = *(volatile uint8_t *)(&(mcp->spiHandle->Instance->DR));
        *resistance = _code_to_resistance (resistanceCode);

        status = HAL_OK;
    }

    _spi_disable (mcp->spiHandle);

    __MCP_UNSELECT (mcp);
    __HAL_UNLOCK (mcp->spiHandle);

    return status;
}

// TODO(Ethan): Test 16-bit receives. Do we need to break it up?
HAL_StatusTypeDef
MCP41HVX1_Startup (MCP41HVX1 *mcp)
{
    // To startup, we need to reconnect the A terminal and the wiper. This
    // is accomplished by writing 0xFF to the TCON register (0x04).
    __HAL_LOCK (mcp->spiHandle);
    __MCP_SELECT (mcp);

    // Enable SPI by setting SPE bit (bit 6)
    mcp->spiHandle->Instance->CR1 |= 0x0040;

    // TODO(Ethan): Test that this actually works!
    // Set the RXNE event to fire when Rx buffer is 1/2 full (16 bits)
    mcp->spiHandle->Instance->CR2 &= ~0x1000;

    // Wait until the SPI transmit buffer is empty
    while (!(mcp->spiHandle->Instance->SR & 0x0002))
        ;

    // Send the write data command for the TCON register
    *((volatile uint8_t *)(&(mcp->spiHandle->Instance->DR))) = (uint8_t)0x40;

    // Wait until the SPI transmit buffer is empty
    while (!(mcp->spiHandle->Instance->SR & 0x0002))
        ;

    // Send the data to tell the TCON register to connect terminal A and W
    *((volatile uint8_t *)(&(mcp->spiHandle->Instance->DR))) = 0xFF;

    // Wait until the receive buffer is full and read it
    while (!(mcp->spiHandle->Instance->SR & 0x0001))
        ;

    // Store the 16 bit value received by MCP on command
    uint16_t rx = *(volatile uint16_t *)(&(mcp->spiHandle->Instance->DR));

    _spi_disable (mcp->spiHandle);

    __MCP_UNSELECT (mcp);
    __HAL_UNLOCK (mcp->spiHandle);

    // TODO(Ethan): What order do we receive from MCP?
    // If CMDERR (bit 7) is low, then an error has occured
    return (rx & 0x0200) ? HAL_OK : HAL_ERROR;
    return HAL_OK;
}

// TODO(Ethan): Test 16-bit receives. Do we need to break it up?
HAL_StatusTypeDef
MCP41HVX1_Shutdown (MCP41HVX1 *mcp)
{
    // To shutdown, we need to disconnect the A terminal and the wiper. This
    // is accomplished by writing 0xF9 to the TCON register (0x04).

    __HAL_LOCK (mcp->spiHandle);
    __MCP_SELECT (mcp);

    // Enable SPI by setting SPE bit (bit 6)
    mcp->spiHandle->Instance->CR1 |= 0x0040;

    // TODO(Ethan): Test that this actually works!
    // Set the RXNE event to fire when Rx buffer is 1/2 full (16 bits)
    mcp->spiHandle->Instance->CR2 &= ~0x1000;

    // Wait until the SPI transmit buffer is empty
    while (!(mcp->spiHandle->Instance->SR & 0x0002))
        ;

    // Send the write data command for the TCON register
    *((volatile uint8_t *)(&(mcp->spiHandle->Instance->DR))) = (uint8_t)0x40;

    // Wait until the SPI transmit buffer is empty
    while (!(mcp->spiHandle->Instance->SR & 0x0002))
        ;

    // Send the data to tell the TCON register to disconnect terminal A and W
    *((volatile uint8_t *)(&(mcp->spiHandle->Instance->DR))) = 0xF9;

    // Wait until the receive buffer is full and read it
    while (!(mcp->spiHandle->Instance->SR & 0x0001))
        ;

    // Store the 16 bit value received by MCP on command
    uint16_t rx = *(volatile uint16_t *)(&(mcp->spiHandle->Instance->DR));

    _spi_disable (mcp->spiHandle);

    __MCP_UNSELECT (mcp);
    __HAL_UNLOCK (mcp->spiHandle);

    // TODO(Ethan): What order do we receive from MCP?
    // If CMDERR (bit 7) is low, then an error has occured
    return (rx & 0x0200) ? HAL_OK : HAL_ERROR;
}
