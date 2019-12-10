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

#define __STUSB4500_USBPD_REV_3_0_SUPPORT__    1
#define __STUSB4500_USBPD_MESSAGE_QUEUE_SZ__   32
#define __STUSB4500_USBPD_INTERRUPT_QUEUE_SZ__ 32
#define __STUSB4500_NVM_SINK_PDO_COUNT__       3
#define __STUSB4500_NVM_SOURCE_PDO_MAX__       10

// ------------------------------------------------------------------- macros --

/* nothing */

// ----------------------------------------------------------- exported types --

typedef struct stusb4500_device stusb4500_device_t;
typedef struct stusb4500_usbpd_status stusb4500_usbpd_status_t;
typedef struct stusb4500_usbpd_state_machine stusb4500_usbpd_state_machine_t;

typedef enum
{
  sccERROR = -1,
  sccNotConnected, // = 0
  sccCC1Connected, // = 1
  sccCC2Connected, // = 2
  sccCOUNT         // = 3
}
stusb4500_cable_connected_t;

typedef enum
{
  srwNONE = -1,
  srwDoNotWait, // = 0
  srwWaitReady, // = 1
  srwCOUNT,     // = 2
}
stusb4500_reset_wait_t;

typedef HAL_StatusTypeDef stusb4500_status_t;

struct stusb4500_usbpd_status
{
    uint8_t                                    hw_reset;
    STUSB_GEN1S_HW_FAULT_STATUS_RegTypeDef     hw_fault_status;     // 8-bit
    STUSB_GEN1S_MONITORING_STATUS_RegTypeDef   monitoring_status;   // 8-bit
    STUSB_GEN1S_CC_DETECTION_STATUS_RegTypeDef cc_detection_status; // 8-bit
    STUSB_GEN1S_CC_STATUS_RegTypeDef           cc_status;           // 8-bit
    STUSB_GEN1S_PRT_STATUS_RegTypeDef          prt_status;          // 8-bit
    STUSB_GEN1S_PHY_STATUS_RegTypeDef          phy_status;          // 8-bit

    USB_PD_SNK_PDO_TypeDef pdo_snk[__STUSB4500_NVM_SINK_PDO_COUNT__];
    USB_PD_SRC_PDO_TypeDef pdo_src[__STUSB4500_NVM_SOURCE_PDO_MAX__];

    STUSB_GEN1S_RDO_REG_STATUS_RegTypeDef rdo_neg;
};

struct stusb4500_usbpd_state_machine
{
  volatile uint16_t irq_received;
  volatile uint16_t irq_hard_reset;
  volatile uint16_t attach_transition;
  volatile uint16_t src_pdo_received;
  volatile uint16_t psrdy_received;
  volatile uint16_t msg_received;
  volatile uint16_t msg_accept;
  volatile uint16_t msg_reject;
  volatile uint16_t msg_goodcrc;

  uint8_t msg[__STUSB4500_USBPD_MESSAGE_QUEUE_SZ__];
  uint8_t msg_head;
  uint8_t msg_tail;

  uint8_t irq[__STUSB4500_USBPD_INTERRUPT_QUEUE_SZ__];
  uint8_t irq_head;
  uint8_t irq_tail;
};

struct stusb4500_device
{
  I2C_HandleTypeDef *i2c_hal;
  uint8_t            i2c_slave_addr; // real address, NOT shifted

  GPIO_TypeDef *reset_port;
  uint16_t      reset_pin;

  stusb4500_usbpd_status_t        usbpd_status;
  stusb4500_usbpd_state_machine_t usbpd_state_machine;
};

// ------------------------------------------------------- exported variables --

/* nothing */

// ------------------------------------------------------- exported functions --

stusb4500_device_t *stusb4500_device_new(
    I2C_HandleTypeDef *i2c_hal,
    uint8_t            i2c_slave_addr,
    GPIO_TypeDef      *reset_port,
    uint16_t           reset_pin);

stusb4500_status_t stusb4500_device_init(stusb4500_device_t *dev);
stusb4500_status_t stusb4500_ready(stusb4500_device_t *dev);

void stusb4500_wait_until_ready(stusb4500_device_t *dev);
void stusb4500_hard_reset(stusb4500_device_t *dev, stusb4500_reset_wait_t wait);
void stusb4500_soft_reset(stusb4500_device_t *dev, stusb4500_reset_wait_t wait);

void stusb4500_process_events(stusb4500_device_t *dev);
void stusb4500_alert(stusb4500_device_t *dev);
void stusb4500_attach(stusb4500_device_t *dev);

stusb4500_cable_connected_t stusb4500_cable_connected(stusb4500_device_t *dev);
stusb4500_status_t stusb4500_get_source_capabilities(stusb4500_device_t *dev);


#ifdef __cplusplus
}
#endif

#endif /* __STUSB4500_API_H */
