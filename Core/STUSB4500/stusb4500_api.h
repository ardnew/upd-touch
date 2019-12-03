/*
 * stusb4500_api.h
 *
 *  Created on: Nov 28, 2019
 *      Author: andrew
 */

#ifndef __STUSB4500_API_H
#define __STUSB4500_API_H

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------- includes --

#if defined (STM32L476xx)
#include "stm32l4xx_hal.h"
#elif defined(STM32F072xx)
#include "stm32f0xx_hal.h"
#elif defined(STM32F401xx)
#include "stm32f4xx_hal.h"
#elif defined(STM32G431xx)
#include "stm32g4xx_hal.h"
#elif defined(STM32G031xx)
#include "stm32g0xx_hal.h"
#endif

#include "stusb4500_registers.h"

// ------------------------------------------------------------------ defines --

#define __STUSB4500_I2C_SLAVE_BASE_ADDR__ 0x28

// ------------------------------------------------------------------- macros --

/* nothing */

// ----------------------------------------------------------- exported types --

typedef HAL_StatusTypeDef stusb4500_status_t;

typedef enum
{
  sccERROR = -1,
  sccNotConnected, // = 0
  sccCC1Connected, // = 1
  sccCC2Connected, // = 2
  sccCOUNT
}
stusb4500_cable_connected_t;

typedef struct
{
  I2C_HandleTypeDef *i2c_hal;

  GPIO_TypeDef      *reset_port;
  uint16_t           reset_pin;

  uint8_t            i2c_slave_addr; // real address, NOT shifted
}
stusb4500_device_t;

// ------------------------------------------------------- exported variables --

/* nothing */

// ------------------------------------------------------- exported functions --

stusb4500_device_t *stusb4500_device_new(
    I2C_HandleTypeDef *i2c_hal,
    GPIO_TypeDef      *reset_port,
    uint16_t           reset_pin,
    uint8_t            i2c_slave_addr);

stusb4500_status_t stusb4500_ready(stusb4500_device_t *dev);

void stusb4500_interrupt(stusb4500_device_t *dev);

stusb4500_cable_connected_t stusb4500_cable_connected(stusb4500_device_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* __STUSB4500_API_H */
