/**
 *      MCP41HVX1 STM32F7 SPI Driver
 *
 *      Author:     Ethan Garnier
 *      Date:       2025
 */
#include "MCP41HVX1.h"
#include "math.h"
#include "stdint.h"

#define __MCP_SELECT(__MCP__) ((__MCP__)->csPort->BSRR = (uint32_t)((__MCP__)->csPin << 16))
#define __MCP_UNSELECT(__MCP__) ((__MCP__)->csPort->BSRR = (__MCP__)->csPin)

static uint8_t old_spi_polarity;
static uint8_t old_spi_phase;

static void
_spi_change_settings (MCP41HVX1 *mcp)
{
    // Store the original values
    old_spi_polarity = ((mcp->spiHandle->Instance->CR1 & 0x0002) >> 1);
    old_spi_phase = (mcp->spiHandle->Instance->CR1 & 0x0001);

    // Update to values required for MCP operation: clear
    // the polarity bit and clear the phase bit.
    mcp->spiHandle->Instance->CR1 &= ~0x0002;
    mcp->spiHandle->Instance->CR1 &= ~0x0001;
}

static void
_spi_revert_settings (MCP41HVX1 *mcp)
{
    // Revert the spi handle's settings to what was
    // originally stored by _spi_change_settings
    if (old_spi_polarity)
        mcp->spiHandle->Instance->CR1 |= 0x0002;

    if (old_spi_phase)
        mcp->spiHandle->Instance->CR1 |= 0x0001;
}

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
    while ((spiHandle->Instance->SR & 0x1800))
        ;

    // 2. Wait until BSY flag is 0
    while ((spiHandle->Instance->SR & 0x0080))
        ;

    // 3. Disable SPI by clearing SPE bit (bit 6)
    spiHandle->Instance->CR1 &= ~0x0040;

    // 4. Flush FIFO Rx buffer until FRLVL[1:0] is 0b00
    volatile uint8_t tempReg = 0x00;
    while ((spiHandle->Instance->SR & 0x0600))
    {
        tempReg = (volatile uint8_t)spiHandle->Instance->DR;
        (void)tempReg; // Avoids GCC unused warning
    }

    return HAL_OK;
}

static void
_spi_16bit_write (MCP41HVX1 *mcp, uint16_t data)
{
    // Wait until the SPI transmit buffer is empty
    while (!(mcp->spiHandle->Instance->SR & 0x0002))
        ;

    // Send the write data command for the wiper register
    *((volatile uint8_t *)(&(mcp->spiHandle->Instance->DR))) = (uint8_t)((data & 0xFF00) >> 8);

    // Wait until the SPI transmit buffer is empty
    while (!(mcp->spiHandle->Instance->SR & 0x0002))
        ;

    // Send the data to be written to the register
    *((volatile uint8_t *)(&(mcp->spiHandle->Instance->DR))) = (uint8_t)(data & 0x00FF);
}

static void
_spi_8bit_read (MCP41HVX1 *mcp, uint8_t *buffer)
{
    // Wait until the receive buffer RXNE flag
    while (!(mcp->spiHandle->Instance->SR & 0x0001))
        ;

    // Store the first 8 bits
    *buffer = *(volatile uint8_t *)(&(mcp->spiHandle->Instance->DR));
}

static void
_spi_16bit_read (MCP41HVX1 *mcp, uint8_t *buffer)
{
    // Wait until the receive buffer RXNE flag
    while (!(mcp->spiHandle->Instance->SR & 0x0001))
        ;

    // Store the first 8 bits
    buffer[0] = *(volatile uint8_t *)(&(mcp->spiHandle->Instance->DR));

    // Wait until the receive buffer RXNE flag
    while (!(mcp->spiHandle->Instance->SR & 0x0001))
        ;

    // Store the second 8 bits
    buffer[1] = *(volatile uint8_t *)(&(mcp->spiHandle->Instance->DR));
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

float
MCP41HVX1_To_Resistance (uint8_t code)
{
    return MCP_R_FS + (float)(MCP_FSV - code) * MCP_STEP_RESISTANCE;
}

uint8_t
MCP41HVX1_To_Code (float resistance)
{
    return MCP_FSV - (uint8_t)roundf ((float)resistance / MCP_STEP_RESISTANCE);
}

HAL_StatusTypeDef
MCP41HVX1_Move_Wiper (MCP41HVX1 *mcp, MCP41HVX1_Wiper_Command cmd)
{
    __HAL_LOCK (mcp->spiHandle);
    _spi_change_settings (mcp);
    __MCP_SELECT (mcp);

    // Set the RXNE event to fire when Rx buffer is 1/4 full (8 bits)
    mcp->spiHandle->Instance->CR2 |= 0x1000;

    // Enable SPI by setting SPE bit (bit 6)
    mcp->spiHandle->Instance->CR1 |= 0x0040;

    // Wait until the SPI transmit buffer is empty
    while (!(mcp->spiHandle->Instance->SR & 0x0002))
        ;

    // Send the specified command to the SPI data register
    *((volatile uint8_t *)(&(mcp->spiHandle->Instance->DR))) = (uint8_t)cmd;

    // Store the 8 bit value received by MCP on command
    uint8_t rx;
    _spi_8bit_read (mcp, &rx);

    _spi_disable (mcp->spiHandle);

    __MCP_UNSELECT (mcp);
    _spi_revert_settings (mcp);
    __HAL_UNLOCK (mcp->spiHandle);

    // If CMDERR (bit 7) is low, then an error has occured
    uint16_t cmderr = (~rx & 0x02);
    return (!cmderr) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef
MCP41HVX1_Set_Resistance_Code (MCP41HVX1 *mcp, uint8_t code)
{
    __HAL_LOCK (mcp->spiHandle);
    _spi_change_settings (mcp);
    __MCP_SELECT (mcp);

    // Set the RXNE event to fire when Rx buffer is 1/4 full (8 bits)
    mcp->spiHandle->Instance->CR2 |= 0x1000;

    // Enable SPI by setting SPE bit (bit 6)
    mcp->spiHandle->Instance->CR1 |= 0x0040;

    // Set the wiper resistance by writing the resistance code to 0x00
    _spi_16bit_write (mcp, 0x0000 | code);

    // Store the 16-bit response from MCP.
    uint8_t rx[2];
    _spi_16bit_read (mcp, rx);

    _spi_disable (mcp->spiHandle);

    __MCP_UNSELECT (mcp);
    _spi_revert_settings (mcp);
    __HAL_UNLOCK (mcp->spiHandle);

    // If CMDERR (bit 7) is low, then an error has occured
    uint8_t cmderr = (~rx[0] & 0x02);
    return (!cmderr) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef
MCP41HVX1_Set_Resistance (MCP41HVX1 *mcp, float resistance)
{
    if (resistance <= 0)
    {
        return HAL_ERROR;
    }

    uint8_t code = MCP41HVX1_To_Code (resistance);
    return MCP41HVX1_Set_Resistance_Code (mcp, code);
}

HAL_StatusTypeDef
MCP41HVX1_Get_Resistance (MCP41HVX1 *mcp, float *resistance)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    __HAL_LOCK (mcp->spiHandle);
    _spi_change_settings (mcp);
    __MCP_SELECT (mcp);

    // Set the RXNE event to fire when Rx buffer is 1/4 full (8 bits)
    mcp->spiHandle->Instance->CR2 |= 0x1000;

    // Enable SPI by setting SPE bit (bit 6)
    mcp->spiHandle->Instance->CR1 |= 0x0040;

    // Wait until the SPI transmit buffer is empty
    while (!(mcp->spiHandle->Instance->SR & 0x0002))
        ;

    // Send the read data command
    *((volatile uint8_t *)(&(mcp->spiHandle->Instance->DR))) = (uint8_t)0x0C;

    // Store the first 8 bits returned from the MCP and check for error
    uint8_t rx;
    _spi_8bit_read (mcp, &rx);
    if (!(~rx & 0x02))
    {
        // No error, continue with read

        // Wait until the SPI transmit buffer is empty
        while (!(mcp->spiHandle->Instance->SR & 0x0002))
            ;

        // Send dummy clocks
        *((volatile uint8_t *)(&(mcp->spiHandle->Instance->DR))) = 0x00;

        // Read the returned resistance code from the MCP and
        // convert into a floating point resistance
        uint8_t resistanceCode;
        _spi_8bit_read (mcp, &resistanceCode);
        *resistance = MCP41HVX1_To_Resistance (resistanceCode);

        status = HAL_OK;
    }

    _spi_disable (mcp->spiHandle);

    __MCP_UNSELECT (mcp);
    _spi_revert_settings (mcp);
    __HAL_UNLOCK (mcp->spiHandle);

    return status;
}

HAL_StatusTypeDef
MCP41HVX1_Startup (MCP41HVX1 *mcp)
{
    // To startup, we need to reconnect the A terminal and the wiper. This
    // is accomplished by writing 0xFF to the TCON register (0x04).

    __HAL_LOCK (mcp->spiHandle);
    _spi_change_settings (mcp);
    __MCP_SELECT (mcp);

    // Set the RXNE event to fire when Rx buffer is 1/4 full (8 bits)
    mcp->spiHandle->Instance->CR2 |= 0x1000;

    // Enable SPI by setting SPE bit (bit 6)
    mcp->spiHandle->Instance->CR1 |= 0x0040;

    // Write 0xFF to the TCON register 0x04
    _spi_16bit_write (mcp, 0x04FF);

    // Store the 16 bit value received by MCP on command
    uint8_t rx[2];
    _spi_16bit_read (mcp, rx);

    _spi_disable (mcp->spiHandle);

    __MCP_UNSELECT (mcp);
    _spi_revert_settings (mcp);
    __HAL_UNLOCK (mcp->spiHandle);

    // If CMDERR (bit 7) is low, then an error has occured
    uint16_t cmderr = (~rx[0] & 0x02);
    return (!cmderr) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef
MCP41HVX1_Shutdown (MCP41HVX1 *mcp)
{
    // To shutdown, we need to disconnect the A terminal and the wiper. This
    // is accomplished by writing 0xF9 to the TCON register (0x04).

    __HAL_LOCK (mcp->spiHandle);
    _spi_change_settings (mcp);
    __MCP_SELECT (mcp);

    // Set the RXNE event to fire when Rx buffer is 1/4 full (8 bits)
    mcp->spiHandle->Instance->CR2 |= 0x1000;

    // Enable SPI by setting SPE bit (bit 6)
    mcp->spiHandle->Instance->CR1 |= 0x0040;

    // Write 0xF9 to the TCON register 0x04.
    _spi_16bit_write (mcp, 0x04F9);

    // Store the 16 bit value received by MCP on command
    uint8_t rx[2];
    _spi_16bit_read (mcp, rx);

    _spi_disable (mcp->spiHandle);

    __MCP_UNSELECT (mcp);
    _spi_revert_settings (mcp);
    __HAL_UNLOCK (mcp->spiHandle);

    // If CMDERR (bit 7) is low, then an error has occured
    uint16_t cmderr = (~rx[0] & 0x02);
    return (!cmderr) ? HAL_OK : HAL_ERROR;
}
