/*
 * stusb4500_api.c
 *
 *  Created on: Nov 28, 2019
 *      Author: andrew
 */

// ----------------------------------------------------------------- includes --

#include "stusb4500_api.h"

#include <stdlib.h> // malloc()

// ---------------------------------------------------------- private defines --

#define __STUSB4500_I2C_MEM_ADDR_SIZE__     I2C_MEMADD_SIZE_8BIT
#define __STUSB4500_I2C_READ_TIMEOUT_MS__   2000
#define __STUSB4500_I2C_WRITE_TIMEOUT_MS__  2000
#define __STUSB4500_DEVICE_ID__             0x21 // in device ID register (0x2F)

#define __GPIO_PIN_CLR__                  GPIO_PIN_RESET
#define __GPIO_PIN_SET__                  GPIO_PIN_SET

// ----------------------------------------------------------- private macros --

#define __I2C_SLAVE_READ_ADDR(addr)  ((addr) << 1)
#define __I2C_SLAVE_WRITE_ADDR(addr) ((addr) << 1)

// ----------------------------------------------------------- private types --

/* nothing */

// ------------------------------------------------------- exported variables --

/* nothing */

// ------------------------------------------------------- private variables --

/* nothing */

// ---------------------------------------------- private function prototypes --

static void stusb4500_reset(stusb4500_device_t *dev);

static stusb4500_status_t stusb4500_i2c_read(stusb4500_device_t *dev,
    uint16_t mem_addr, uint8_t *buff_dst, uint16_t buff_dst_sz);
static stusb4500_status_t stusb4500_i2c_write(stusb4500_device_t *dev,
    uint16_t mem_addr, uint8_t *buff_src, uint16_t buff_src_sz);

// ------------------------------------------------------- exported functions --

stusb4500_device_t *stusb4500_device_new(
    I2C_HandleTypeDef *i2c_hal,
    GPIO_TypeDef      *reset_port,
    uint16_t           reset_pin,
    uint8_t            i2c_slave_addr)
{
  stusb4500_device_t *dev = NULL;

  if ( (NULL != i2c_hal)       &&
       (NULL != reset_port)    &&
       IS_GPIO_PIN(reset_pin)  &&
       /* slave address is 7-bit */
       (i2c_slave_addr < 0x80) ) {
    if (NULL != (dev = malloc(sizeof(stusb4500_device_t)))) {
      dev->i2c_hal        = i2c_hal;
      dev->reset_port     = reset_port;
      dev->reset_pin      = reset_pin;
      dev->i2c_slave_addr = i2c_slave_addr;

      stusb4500_reset(dev);
    }
  }

  return dev;
}

stusb4500_status_t stusb4500_ready(stusb4500_device_t *dev)
{
  stusb4500_status_t status;
  uint8_t device_id;

  if (NULL == dev)
    { return HAL_ERROR; }

  if (HAL_OK != (status = stusb4500_i2c_read(dev, REG_DEVICE_ID, &device_id, 1)))
    { return status; }

  if (__STUSB4500_DEVICE_ID__ != device_id)
    { return HAL_ERROR; }

  return HAL_OK;
}

void stusb4500_interrupt(stusb4500_device_t *dev)
{
  return;
}

stusb4500_cable_connected_t stusb4500_cable_connected(stusb4500_device_t *dev)
{
  stusb4500_status_t status;
  uint8_t port_status;
  uint8_t type_c_status;

  if (NULL == dev)
    { return sccERROR; }

  if (HAL_OK != (status = stusb4500_i2c_read(dev, REG_PORT_STATUS, &port_status, 1)))
    { return sccERROR; }

  if (VALUE_ATTACHED == (port_status & STUSBMASK_ATTACHED_STATUS))
  {
    if (HAL_OK != (status = stusb4500_i2c_read(dev, REG_TYPE_C_STATUS, &type_c_status, 1)))
      { return sccERROR; }

    if (0 == (type_c_status & MASK_REVERSE))
      { return sccCC1Connected; }
    else
      { return sccCC2Connected; }
  }
  else
  {
    return sccNotConnected;
  }
}

// ------------------------------------------------------- private functions --

static void stusb4500_reset(stusb4500_device_t *dev)
{
  stusb4500_status_t stat;

  // the reset pin on STUSB4500 is active high, so driving high temporarily
  // will reset the device
  HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, __GPIO_PIN_SET__);
  HAL_Delay(200);
  HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, __GPIO_PIN_CLR__);

  HAL_Delay(600);

  // clear all ALERT Status
  uint16_t addr = ALERT_STATUS_1;
  uint8_t data[40];
  for (uint16_t i = 0; i <= 12; ++i)
  {
    stat = stusb4500_i2c_read(dev, addr + i, data, 1);  // clear ALERT Status
    if (HAL_OK != stat)
      { return; }
  }

  STUSB_GEN1S_ALERT_STATUS_MASK_RegTypeDef alert_mask;

  alert_mask.d8 = 0xFF;
  alert_mask.b.PHY_STATUS_AL_MASK          = 0;
  alert_mask.b.PRT_STATUS_AL_MASK          = 0;
  alert_mask.b.PD_TYPEC_STATUS_AL_MASK     = 0;
  alert_mask.b.HW_FAULT_STATUS_AL_MASK     = 0;
  alert_mask.b.MONITORING_STATUS_AL_MASK   = 0;
  alert_mask.b.CC_DETECTION_STATUS_AL_MASK = 0;
  alert_mask.b.HARD_RESET_AL_MASK          = 0;

  stat = stusb4500_i2c_write(dev, ALERT_STATUS_MASK, &(alert_mask.d8), 1);
  if (HAL_OK != stat)
    { return; }
}

static stusb4500_status_t stusb4500_i2c_read(stusb4500_device_t *dev,
    uint16_t mem_addr, uint8_t *buff_dst, uint16_t buff_dst_sz)
{
  stusb4500_status_t status;

  while (HAL_BUSY == (status = HAL_I2C_Mem_Read(
      dev->i2c_hal,
      __I2C_SLAVE_READ_ADDR(dev->i2c_slave_addr),
      mem_addr,
      __STUSB4500_I2C_MEM_ADDR_SIZE__,
      buff_dst,
      buff_dst_sz,
      __STUSB4500_I2C_READ_TIMEOUT_MS__))) {

    // should not happen, unless during IRQ routine
    HAL_I2C_DeInit(dev->i2c_hal);
    HAL_I2C_Init(dev->i2c_hal);
  }

  return status;
}

static stusb4500_status_t stusb4500_i2c_write(stusb4500_device_t *dev,
    uint16_t mem_addr, uint8_t *buff_src, uint16_t buff_src_sz)
{
  return HAL_I2C_Mem_Write(
      dev->i2c_hal,
      __I2C_SLAVE_WRITE_ADDR(dev->i2c_slave_addr),
      mem_addr,
      __STUSB4500_I2C_MEM_ADDR_SIZE__,
      buff_src,
      buff_src_sz,
      __STUSB4500_I2C_WRITE_TIMEOUT_MS__); // unmask all alarm status
}
