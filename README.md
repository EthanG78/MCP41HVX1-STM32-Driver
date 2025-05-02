## MCP41HVX1 STM32 Driver
A serial peripheral interface (SPI) driver for the [MCP41HVX1 8-bit digital potentiometer](https://www.mouser.ca/datasheet/2/268/MCP41HVX1_Data_Sheet_20005207-3443119.pdf) written in C for the ARM-based Cortex-M7 MCU running on STM32F7 devices. This driver has been tested using the STM NUCLEO-F746ZG board.

### Including the driver in your STM32 project
If you are building your STM32 project using the STM32CubeIDE, simply place the MCP41HVX1.c and MCP41HVX1.h files within the Src and Inc directories of your project, respectively.

In the future, time permitting, I will look into building this driver as a static library through the use of the [stm32-cmake project](https://github.com/ObKo/stm32-cmake/tree/master).

#### Author: Ethan Garnier
