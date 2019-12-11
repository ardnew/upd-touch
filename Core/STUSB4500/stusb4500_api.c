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
#define __STUSB4500_TLOAD_REG_INIT_MS__      250 // section 6.1.3 in datasheet
#define __STUSB4500_SW_RESET_DEBOUNCE_MS__    27

#define __STUSB4500_USBPD_MESSAGE_TYPE_DATA__ 'D'
#define __STUSB4500_USBPD_MESSAGE_TYPE_CTRL__ 'C'

#define __GPIO_PIN_CLR__ GPIO_PIN_RESET
#define __GPIO_PIN_SET__ GPIO_PIN_SET

// ----------------------------------------------------------- private macros --

#define __I2C_SLAVE_READ_ADDR(addr)  ((addr) << 1)
#define __I2C_SLAVE_WRITE_ADDR(addr) ((addr) << 1)

#define __U16_MSBYTE(u) (uint8_t)(((uint16_t)(u) >> 8U) & 0xFF)
#define __U16_LSBYTE(u) (uint8_t)(((uint16_t)(u)      ) & 0xFF)

// convert value at addr to little-endian (16-bit)
#define __U16_LEND(addr)                                  \
    ( ( (((uint16_t)(*(((uint8_t *)(addr)) + 0)))     ) + \
        (((uint16_t)(*(((uint8_t *)(addr)) + 1))) << 8) ) )

// convert value at addr to little-endian (32-bit)
#define __U32_LEND(addr)                                   \
    ( ( (((uint32_t)(*(((uint8_t *)(addr)) + 0)))      ) + \
        (((uint32_t)(*(((uint8_t *)(addr)) + 1))) <<  8) + \
        (((uint32_t)(*(((uint8_t *)(addr)) + 2))) << 16) + \
        (((uint32_t)(*(((uint8_t *)(addr)) + 3))) << 24) ) )

#define __NO_INTERRUPT(e) { __disable_irq(); (e); __enable_irq(); }

#define __CABLE_CONNECTED(c) \
    ((sccCC1Connected == (c)) || (sccCC2Connected == (c)))

// ------------------------------------------------------------ private types --

typedef union
{
  uint16_t d16;
  struct {
#if defined(__STUSB4500_USBPD_REV_3_0_SUPPORT__)
    uint16_t message_type      : 5; // USBPD rev >= 3.0 message type
#else
    uint16_t message_type      : 4; // USBPD rev  < 3.0 message type
    uint16_t reserved_4        : 1; // reserved
#endif
    uint16_t port_data_role    : 1; // port data role
    uint16_t spec_revision     : 2; // spec revision
    uint16_t port_power_role   : 1; // port power role/cable plug
    uint16_t message_id        : 3; // message ID
    uint16_t data_object_count : 3; // number of data objects
    uint16_t extended          : 1; // reserved
  } b;
}
stusb4500_usbpd_message_header_t;

typedef enum
{
  suaERROR = -1,
  suaDoNotUnmask,  // = 0
  suaUnmaskAlerts, // = 1
  suaCOUNT,        // = 2
}
stusb4500_unmask_alerts_t;

// ------------------------------------------------------- exported variables --

/* nothing */

// -------------------------------------------------------- private variables --

/* nothing */

// ---------------------------------------------- private function prototypes --

static stusb4500_status_t stusb4500_i2c_read(stusb4500_device_t *dev,
    uint16_t mem_addr, uint8_t *buff_dst, uint16_t buff_dst_sz);
static stusb4500_status_t stusb4500_i2c_write(stusb4500_device_t *dev,
    uint16_t mem_addr, uint8_t *buff_src, uint16_t buff_src_sz);

// STUSB4500 I2C operations
static stusb4500_status_t stusb4500_clear_all_alerts(stusb4500_device_t *dev,
    stusb4500_unmask_alerts_t unmask);
static stusb4500_status_t stusb4500_read_port_status(stusb4500_device_t *dev);
static stusb4500_status_t stusb4500_get_all_sink_pdo(stusb4500_device_t *dev);
static stusb4500_status_t stusb4500_set_num_sink_pdo(stusb4500_device_t *dev,
    uint8_t count);

// USB power delivery messaging
static stusb4500_status_t stusb4500_usbpd_cable_reset(stusb4500_device_t *dev);
//static stusb4500_get_source_capabilities(stusb4500_device_t *dev);

// interrupt message queueing handlers
static stusb4500_status_t stusb4500_usbpd_push_message(stusb4500_device_t *dev,
    uint8_t type, uint8_t value);
static stusb4500_status_t stusb4500_usbpd_pop_message(stusb4500_device_t *dev,
    uint8_t *type);
static stusb4500_status_t stusb4500_usbpd_push_interrupt(stusb4500_device_t *dev,
    uint8_t type);
static stusb4500_status_t stusb4500_usbpd_pop_interrupt(stusb4500_device_t *dev,
    uint8_t *type);

// ------------------------------------------------------- exported functions --

stusb4500_device_t *stusb4500_device_new(
    I2C_HandleTypeDef *i2c_hal,
    uint8_t            i2c_slave_addr,
    GPIO_TypeDef      *reset_port,
    uint16_t           reset_pin)
{
  stusb4500_device_t *dev = NULL;

  if ( (NULL != i2c_hal)       &&
       (i2c_slave_addr < 0x80) && /* I2C slave address is 7-bit */
       (NULL != reset_port)    &&
       IS_GPIO_PIN(reset_pin)  ) {
    if (NULL != (dev = malloc(sizeof(stusb4500_device_t)))) {
      dev->i2c_hal        = i2c_hal;
      dev->i2c_slave_addr = i2c_slave_addr;
      dev->reset_port     = reset_port;
      dev->reset_pin      = reset_pin;

      dev->usbpd_status = (stusb4500_usbpd_status_t){
        .hw_reset               = 0U,
        .hw_fault_status.d8     = 0U,
        .monitoring_status.d8   = 0U,
        .cc_detection_status.d8 = 0U,
        .cc_status.d8           = 0U,
        .prt_status.d8          = 0U,
        .phy_status.d8          = 0U,

        .pdo_snk_count = 0U,
        .pdo_snk = { { 0U } },

        .pdo_src_count = 0U,
        .pdo_src = { { 0U } },

        .rdo_neg = { 0U },
      };

      dev->usbpd_state_machine = (stusb4500_usbpd_state_machine_t){
        .irq_received      = 0U,
        .irq_hard_reset    = 0U,
        .attach_transition = 0U,
        .src_pdo_received  = 0U,
        .psrdy_received    = 0U,
        .msg_received      = 0U,
        .msg_accept        = 0U,
        .msg_reject        = 0U,
        .msg_goodcrc       = 0U,

        .msg      = { 0U },
        .msg_head = 0U,
        .msg_tail = 0U,

        .irq      = { 0U },
        .irq_head = 0U,
        .irq_tail = 0U,
      };
    }
  }

  return dev;
}

/****
 * initializes the STUSB4500 device by waiting until it is responding to I2C
 * commands, clearing any current alerts, and unmasking useful interrupts
 *
 ****/
stusb4500_status_t stusb4500_device_init(stusb4500_device_t *dev)
{
  stusb4500_status_t stat;

  if (NULL == dev)
    { return HAL_ERROR; }

  //stusb4500_wait_until_ready(dev);
  stusb4500_soft_reset(dev, srwWaitReady);

  if (HAL_OK != (stat = stusb4500_clear_all_alerts(dev, suaUnmaskAlerts)))
    { return stat; }

  if (HAL_OK != (stat = stusb4500_read_port_status(dev)))
    { return stat; }

  if (HAL_OK != (stat = stusb4500_get_source_capabilities(dev)))
    { return stat; }

  return HAL_OK;
}

/****
 * query the internal device ID register of the STUSB4500 and verify it matches
 * the expected manufacturer-specified ID. this is used to determine if the
 * device has powered on and can respond to I2C read requests.
 ****/
stusb4500_status_t stusb4500_ready(stusb4500_device_t *dev)
{
  stusb4500_status_t stat;
  uint8_t device_id;

  if (NULL == dev)
    { return HAL_ERROR; }

  if (HAL_OK != (stat = stusb4500_i2c_read(dev, REG_DEVICE_ID, &device_id, 1U)))
    { return stat; }

  if (__STUSB4500_DEVICE_ID__ != device_id)
    { return HAL_ERROR; }

  return HAL_OK;
}

/****
 * repeatedly query the device in blocking mmode until it is ready.
 ****/
void stusb4500_wait_until_ready(stusb4500_device_t *dev)
{
  while (HAL_OK != stusb4500_ready(dev))
    { continue ; }
}

/****
 * assert and de-assert the hardware reset pin on the STUSB4500. after reset,
 * the device powers on and behaves according to NVM settings by default.
 * this routine can optionally block return until the device has verifiably
 * powered back on and is responding to I2C commands.
 ****/
void stusb4500_hard_reset(stusb4500_device_t *dev, stusb4500_reset_wait_t wait)
{
  // the reset pin on STUSB4500 is active high, so driving high temporarily
  // will reset the device
  HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, __GPIO_PIN_SET__);
  HAL_Delay(25);
  HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, __GPIO_PIN_CLR__);
  HAL_Delay(15);

  switch (wait) {
    case srwWaitReady:
      // first, wait for I2C registers to be initialized from NVM
      HAL_Delay(__STUSB4500_TLOAD_REG_INIT_MS__);
      // second, try querying the device
      stusb4500_wait_until_ready(dev);
      break;

    case srwDoNotWait:
    default:
      break;
  }
}

/****
 * resets the STUSB4500 by temporarily setting and then clearing the reset
 * register. this resets the internal Type-C and USB PD state machines and
 * causes electrical disconnect on both source and sink sides. while reset, all
 * pending interrupt alarms are cleared. this routine can optionally block
 * return until the device has verifiably powered back on and is responding to
 * I2C commands.
 ****/
void stusb4500_soft_reset(stusb4500_device_t *dev, stusb4500_reset_wait_t wait)
{
  uint8_t buff_src = SW_RST;
  stusb4500_status_t stat =
      stusb4500_i2c_write(dev, STUSB_GEN1S_RESET_CTRL_REG, &buff_src, 1);
  if (HAL_OK != stat)
    { return; }

  if (HAL_OK != (stat = stusb4500_clear_all_alerts(dev, suaDoNotUnmask)))
    { return; }

  HAL_Delay(__STUSB4500_SW_RESET_DEBOUNCE_MS__);

  // restore reset control register to listen only on hardware reset pin
  buff_src = No_SW_RST;
  stat = stusb4500_i2c_write(dev, STUSB_GEN1S_RESET_CTRL_REG, &buff_src, 1);
  if (HAL_OK != stat)
    { return; } // not superfluous, used for debugging

  switch (wait) {
    case srwWaitReady:
      // first, wait for I2C registers to be initialized from NVM
      HAL_Delay(__STUSB4500_TLOAD_REG_INIT_MS__);
      // second, try querying the device
      stusb4500_wait_until_ready(dev);
      break;

    case srwDoNotWait:
    default:
      break;
  }
}

void stusb4500_process_events(stusb4500_device_t *dev)
{
  uint8_t irq;
  uint8_t msg;
  uint8_t value;

  if (NULL == dev)
    { return; }

  stusb4500_usbpd_state_machine_t *usm = &(dev->usbpd_state_machine);
  while (0U != usm->irq_received) {
    --(usm->irq_received);
    //__NO_INTERRUPT(--(usm->irq_received));
    stusb4500_usbpd_pop_interrupt(dev, &irq);
  }

  if (0U != usm->attach_transition) {
    //--(usm->attach_transition);
    __NO_INTERRUPT(--(usm->attach_transition));

    if (VALUE_ATTACHED ==
        (dev->usbpd_status.cc_detection_status.d8 & STUSBMASK_ATTACHED_STATUS)) {
      ;
    }
    else {
      ;
    }
  }

  if (0U != usm->msg_received) {
    --(usm->msg_received);
    //__NO_INTERRUPT(--(usm->msg_received));

    while (HAL_ERROR != stusb4500_usbpd_pop_message(dev, &msg)) {
      switch (msg) {
        case __STUSB4500_USBPD_MESSAGE_TYPE_CTRL__:
          if (HAL_ERROR != stusb4500_usbpd_pop_message(dev, &value)) {
            ;
          }
          break;

        case __STUSB4500_USBPD_MESSAGE_TYPE_DATA__:
          if (HAL_ERROR != stusb4500_usbpd_pop_message(dev, &value)) {
            ;
          }
          break;

        default:
          break;
      }
    }
  }

  if (0U != usm->src_pdo_received) {
    //--(usm->src_pdo_received);
    __NO_INTERRUPT(--(usm->src_pdo_received));

    if (NULL != dev->source_capabilities_received) {
      dev->source_capabilities_received(dev);
    }
  }

  if (0U != usm->psrdy_received) {
    --(usm->psrdy_received);
    //__NO_INTERRUPT(--(usm->psrdy_received));
  }
}

void stusb4500_alert(stusb4500_device_t *dev)
{
#define READ_BUFF_MAX_SZ 40
  STUSB_GEN1S_ALERT_STATUS_RegTypeDef      alert_status;
  STUSB_GEN1S_ALERT_STATUS_MASK_RegTypeDef alert_mask;
  uint8_t read_buff[READ_BUFF_MAX_SZ];

  if (NULL == dev)
    { return; }

  stusb4500_status_t stat = stusb4500_i2c_read(dev, ALERT_STATUS_1, read_buff, 2U);
  if (HAL_OK != stat)
    { return; } // if ALRT interrupt works and I2C doesn't, something's very wrong

  alert_mask.d8   = read_buff[1];
  alert_status.d8 = read_buff[0] & ~(alert_mask.d8);

  stusb4500_usbpd_push_interrupt(dev, alert_status.d8);

  if (0U != alert_status.d8) {

    // bit 8
    dev->usbpd_status.hw_reset = read_buff[0] >> 7U;
    if (0U != dev->usbpd_status.hw_reset)
      { ++(dev->usbpd_state_machine.irq_hard_reset); }

    // bit 7
    if (0U != alert_status.b.CC_DETECTION_STATUS_AL) {
      stat = stusb4500_i2c_read(dev, PORT_STATUS_TRANS, read_buff, 2U);
      if (HAL_OK != stat)
        { return; }
      dev->usbpd_status.cc_detection_status.d8 = read_buff[1];
      if (0U != (read_buff[0] & STUSBMASK_ATTACH_STATUS_TRANS))
        { ++(dev->usbpd_state_machine.attach_transition); }
    }

    // bit 6
    if (0U != alert_status.b.MONITORING_STATUS_AL) {
      stat = stusb4500_i2c_read(dev, TYPEC_MONITORING_STATUS_0, read_buff, 2U);
      if (HAL_OK != stat)
        { return; }
      dev->usbpd_status.monitoring_status.d8 = read_buff[1];
    }

    // always read & update CC attachement status
    stat = stusb4500_i2c_read(dev, CC_STATUS, read_buff, 1U);
    if (HAL_OK != stat)
      { return; }
    dev->usbpd_status.cc_status.d8 = read_buff[0];

    // bit 5
    if (0U != alert_status.b.HW_FAULT_STATUS_AL) {
      stat = stusb4500_i2c_read(dev, CC_HW_FAULT_STATUS_0, read_buff, 2U);
      if (HAL_OK != stat)
        { return; }
      dev->usbpd_status.hw_fault_status.d8 = read_buff[1];
    }

    // bit 2
    if (0U != alert_status.b.PRT_STATUS_AL) {

      stusb4500_usbpd_message_header_t header;

      stat = stusb4500_i2c_read(dev, PRT_STATUS, read_buff, 1U);
      if (HAL_OK != stat)
        { return; }
      dev->usbpd_status.prt_status.d8 = read_buff[0];

      if (1U == dev->usbpd_status.prt_status.b.MSG_RECEIVED) {

        stat = stusb4500_i2c_read(dev, RX_HEADER, read_buff, 2U);
        if (HAL_OK != stat)
          { return; }

        header.d16 = __U16_LEND(read_buff);

        if (header.b.data_object_count > 0) {

          stusb4500_usbpd_push_message(dev,
              __STUSB4500_USBPD_MESSAGE_TYPE_DATA__, header.b.message_type);

          stat = stusb4500_i2c_read(dev, RX_BYTE_CNT, read_buff, 1U);
          if (HAL_OK != stat)
            { return; }
          uint8_t rx_byte_count = read_buff[0];
          if (rx_byte_count != header.b.data_object_count * 4U)
            { return; }

          switch (header.b.message_type) {
            case USBPD_DATAMSG_Source_Capabilities:

              stat = stusb4500_i2c_read(dev, RX_DATA_OBJ, read_buff, rx_byte_count);
              if (HAL_OK != stat)
                { return; }

              dev->usbpd_status.pdo_src_count = header.b.data_object_count;
              for (uint8_t i = 0, j = 0; i < header.b.data_object_count; ++i, j += 4) {
                dev->usbpd_status.pdo_src[i].d32 = __U32_LEND(&read_buff[j]);
              }

              ++(dev->usbpd_state_machine.src_pdo_received);
              break;

            case USBPD_DATAMSG_Request:
            case USBPD_DATAMSG_Sink_Capabilities:
            case USBPD_DATAMSG_Vendor_Defined:
            default :
              break;
          }
        }
        else {

          stusb4500_usbpd_push_message(dev,
              __STUSB4500_USBPD_MESSAGE_TYPE_CTRL__, header.b.message_type);

          switch (header.b.message_type) {
            case USBPD_CTRLMSG_GoodCRC:
                 ++(dev->usbpd_state_machine.msg_goodcrc);
                break;

            case USBPD_CTRLMSG_Accept:
                 ++(dev->usbpd_state_machine.msg_accept);
                break;

            case USBPD_CTRLMSG_Reject:
                 ++(dev->usbpd_state_machine.msg_reject);
                break;

            case USBPD_CTRLMSG_PS_RDY:
                 ++(dev->usbpd_state_machine.psrdy_received);
                break;

            case USBPD_CTRLMSG_Reserved1:
            case USBPD_CTRLMSG_Get_Source_Cap:
            case USBPD_CTRLMSG_Get_Sink_Cap:
            case USBPD_CTRLMSG_Wait:
            case USBPD_CTRLMSG_Soft_Reset:
            case USBPD_CTRLMSG_Not_Supported:
            case USBPD_CTRLMSG_Get_Source_Cap_Extended:
            case USBPD_CTRLMSG_Get_Status:
            case USBPD_CTRLMSG_FR_Swap:
            case USBPD_CTRLMSG_Get_PPS_Status:
            case USBPD_CTRLMSG_Get_Country_Codes:
            default:
              break;
          }
        }
      }
    }
  }

  return;
#undef READ_BUFF_MAX_SZ
}

void stusb4500_attach(stusb4500_device_t *dev)
{
  return;
}

/****
 * read the port and Type-C status registers to determine if a USB cable is
 * connected to the STUSB4500 device. if connected, the orientation is
 * determined and indicated by return value of the selected CC line.
 ****/
stusb4500_cable_connected_t stusb4500_cable_connected(stusb4500_device_t *dev)
{
  stusb4500_status_t status;
  uint8_t port_status;
  uint8_t type_c_status;

  if (NULL == dev)
    { return sccERROR; }

  if (HAL_OK != (status = stusb4500_i2c_read(dev, REG_PORT_STATUS, &port_status, 1U)))
    { return sccERROR; }

  if (VALUE_ATTACHED == (port_status & STUSBMASK_ATTACHED_STATUS))
  {
    if (HAL_OK != (status = stusb4500_i2c_read(dev, REG_TYPE_C_STATUS, &type_c_status, 1U)))
      { return sccERROR; }

    if (0U == (type_c_status & MASK_REVERSE))
      { return sccCC1Connected; }
    else
      { return sccCC2Connected; }
  }
  else
  {
    return sccNotConnected;
  }
}

stusb4500_status_t stusb4500_get_source_capabilities(stusb4500_device_t *dev)
{
  if (NULL == dev)
    { return HAL_ERROR; }

  stusb4500_status_t stat;

  if (HAL_OK != (stat = stusb4500_usbpd_cable_reset(dev)))
    { return stat; }

  return HAL_OK;
}

void stusb4500_set_source_capabilities_received(stusb4500_device_t *dev, stusb4500_event_callback_t callback)
{
  if ((NULL != dev) && (NULL != callback)) {
    dev->source_capabilities_received = callback;
  }
}

// -------------------------------------------------------- private functions --

/****
 * perform an I2C read on a STUSB4500 slave device.
 ****/
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

/****
 * perform an I2C write on a STUSB4500 slave device.
 ****/
static stusb4500_status_t stusb4500_i2c_write(stusb4500_device_t *dev,
    uint16_t mem_addr, uint8_t *buff_src, uint16_t buff_src_sz)
{
  stusb4500_status_t status;

  status = HAL_I2C_Mem_Write(
      dev->i2c_hal,
      __I2C_SLAVE_WRITE_ADDR(dev->i2c_slave_addr),
      mem_addr,
      __STUSB4500_I2C_MEM_ADDR_SIZE__,
      buff_src,
      buff_src_sz,
      __STUSB4500_I2C_WRITE_TIMEOUT_MS__);

  return status;
}

/****
 * clear all pending alerts by reading the status registers.
 ****/
static stusb4500_status_t stusb4500_clear_all_alerts(stusb4500_device_t *dev,
    stusb4500_unmask_alerts_t unmask)
{
  stusb4500_status_t stat;

  // clear alert status
  uint8_t alert_status[12];
  if (HAL_OK != (stat = stusb4500_i2c_read(dev, ALERT_STATUS_1, alert_status, 12U)))
    { return stat; }

  STUSB_GEN1S_ALERT_STATUS_MASK_RegTypeDef alert_mask;
  switch (unmask) {
    case suaUnmaskAlerts:
      // set interrupts to unmask
      alert_mask.d8 = 0xFF;
      //alert_mask.b.PHY_STATUS_AL_MASK          = 0U;
      alert_mask.b.PRT_STATUS_AL_MASK          = 0U;
      alert_mask.b.PD_TYPEC_STATUS_AL_MASK     = 0U;
      //alert_mask.b.HW_FAULT_STATUS_AL_MASK     = 0U;
      alert_mask.b.MONITORING_STATUS_AL_MASK   = 0U;
      alert_mask.b.CC_DETECTION_STATUS_AL_MASK = 0U;
      alert_mask.b.HARD_RESET_AL_MASK          = 0U;
      // unmask the above alarms
      if (HAL_OK != (stat = stusb4500_i2c_write(dev,
            ALERT_STATUS_MASK, &(alert_mask.d8), 1U)))
        { return stat; }
      break;

    case suaDoNotUnmask:
    default:
      break;
  }

  return HAL_OK;
}

static stusb4500_status_t stusb4500_read_port_status(stusb4500_device_t *dev)
{
#define BUFF_SZ 10
  uint8_t prt_buff[BUFF_SZ];

  stusb4500_status_t stat = stusb4500_i2c_read(dev, REG_PORT_STATUS, prt_buff, BUFF_SZ);
  if (HAL_OK != stat)
    { return stat; }

  dev->usbpd_status.cc_detection_status.d8 = prt_buff[1];
  dev->usbpd_status.cc_status.d8           = prt_buff[3];
  dev->usbpd_status.monitoring_status.d8   = prt_buff[3];
  dev->usbpd_status.hw_fault_status.d8     = prt_buff[6];

  return HAL_OK;
#undef BUFF_SZ
}

static stusb4500_status_t stusb4500_get_all_sink_pdo(stusb4500_device_t *dev)
{
#define BUFF_SZ __STUSB4500_NVM_SINK_PDO_COUNT__ * sizeof(USB_PD_SNK_PDO_TypeDef)
  uint8_t pdo_buff[BUFF_SZ];

  stusb4500_status_t stat = stusb4500_i2c_read(dev, DPM_SNK_PDO1, pdo_buff, BUFF_SZ);
  if (HAL_OK != stat)
    { return stat; }

  for (uint8_t i = 0, j = 0; i < __STUSB4500_NVM_SINK_PDO_COUNT__; ++i, j += 4) {
    dev->usbpd_status.pdo_snk[i].d32 = __U32_LEND(&pdo_buff[j]);
  }
  return HAL_OK;
#undef BUFF_SZ
}

static stusb4500_status_t stusb4500_set_num_sink_pdo(stusb4500_device_t *dev,
    uint8_t count)
{
  if ((count < 1) || (count > __STUSB4500_NVM_SINK_PDO_COUNT__))
    { return HAL_ERROR; }

  return stusb4500_i2c_write(dev, DPM_PDO_NUMB, &count, 1);
}

/****
 * send a USB power delivery reset message, forcing the source to send all of
 * its available PDOs.
 ****/
static stusb4500_status_t stusb4500_usbpd_cable_reset(stusb4500_device_t *dev)
{
#define USBPD_HEADER_SOFT_RESET 0x0D
#define USBPD_PD_COMMAND        0x26

  stusb4500_cable_connected_t conn = stusb4500_cable_connected(dev);

  // only continue if the cable is connected
  if (!__CABLE_CONNECTED(conn))
    { return HAL_ERROR; }

  stusb4500_status_t stat;

  // send PD message "soft reset" to source by setting TX header (0x51) to 0x0D,
  // and set PD command (0x1A) to 0x26.
  uint8_t soft_reset_data = USBPD_HEADER_SOFT_RESET;
  stat = stusb4500_i2c_write(dev, TX_HEADER, &soft_reset_data, 1U);
  if (HAL_OK != stat)
    { return stat; }

  uint8_t pd_command = USBPD_PD_COMMAND;
  stat = stusb4500_i2c_write(dev, STUSB_GEN1S_CMD_CTRL, &pd_command, 1U);
  if (HAL_OK != stat)
    { return stat; }

  return HAL_OK;

#undef USBPD_HEADER_SOFT_RESET
#undef USBPD_PD_COMMAND
}

static stusb4500_status_t stusb4500_usbpd_push_message(stusb4500_device_t *dev,
    uint8_t type, uint8_t value) {

  stusb4500_usbpd_state_machine_t *usm = &(dev->usbpd_state_machine);

  usm->msg[usm->msg_head] = type;
  ++(usm->msg_head);

  usm->msg[usm->msg_head] = value;
  ++(usm->msg_head);

  if (usm->msg_head == usm->msg_tail)
    { return HAL_ERROR; } // buffer overflow

  if (usm->msg_head >= __STUSB4500_USBPD_MESSAGE_QUEUE_SZ__)
    { usm->msg_head = 0U; }

  ++(usm->msg_received);

  return HAL_OK;
}

static stusb4500_status_t stusb4500_usbpd_pop_message(stusb4500_device_t *dev,
    uint8_t *type) {

  stusb4500_usbpd_state_machine_t *usm = &(dev->usbpd_state_machine);

  if (usm->msg_head == usm->msg_tail)
    { return HAL_ERROR; } // empty buffer

  *type = usm->msg[usm->msg_tail];
  usm->msg[usm->msg_tail] = 0U;

  ++(usm->msg_tail);
  if (usm->msg_tail >= __STUSB4500_USBPD_MESSAGE_QUEUE_SZ__)
    { usm->msg_tail = 0U; }

  return HAL_OK;
}

static stusb4500_status_t stusb4500_usbpd_push_interrupt(stusb4500_device_t *dev,
    uint8_t type) {

  stusb4500_usbpd_state_machine_t *usm = &(dev->usbpd_state_machine);

  usm->irq[usm->irq_head] = type;
  ++(usm->irq_head);

  if (usm->irq_head == usm->irq_tail)
    { return HAL_ERROR; } // buffer overflow

  if (usm->irq_head >= __STUSB4500_USBPD_INTERRUPT_QUEUE_SZ__)
    { usm->irq_head = 0U; }

  ++(usm->irq_received);

  return HAL_OK;
}

static stusb4500_status_t stusb4500_usbpd_pop_interrupt(stusb4500_device_t *dev,
    uint8_t *type) {

  stusb4500_usbpd_state_machine_t *usm = &(dev->usbpd_state_machine);

  if (usm->irq_head == usm->irq_tail)
    { return HAL_ERROR; } // empty buffer

  *type = usm->irq[usm->irq_tail];
  usm->irq[usm->irq_tail] = 0U;

  ++(usm->irq_tail);
  if (usm->irq_tail >= __STUSB4500_USBPD_INTERRUPT_QUEUE_SZ__)
    { usm->irq_tail = 0U; }

  return HAL_OK;
}

