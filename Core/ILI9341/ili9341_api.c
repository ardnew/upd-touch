/*
 * ili9341_api.c
 *
 *  Created on: Nov 28, 2019
 *      Author: andrew
 */

// ----------------------------------------------------------------- includes --

#include "ili9341_api.h"

#include <stdlib.h> // malloc()
#include <ctype.h>

// ---------------------------------------------------------- private defines --

#define __GPIO_PIN_CLR__    GPIO_PIN_RESET
#define __GPIO_PIN_SET__    GPIO_PIN_SET

#define __SPI_MAX_DELAY__   HAL_MAX_DELAY
#define __SPI_TX_BLOCK_SZ__ (1 * 1024) // 1 KiB -MAX- for HAL SPI transmit

// ----------------------------------------------------------- private macros --

#define __IS_SPI_SLAVE(s) (((s) > issNONE) && ((s) < issCOUNT))

#define __SLAVE_SELECT(d, s)  \
  if (__IS_SPI_SLAVE(s)) { ili9341_spi_slave_select((d), (s)); }

#define __SLAVE_RELEASE(d, s) \
  if (__IS_SPI_SLAVE(s)) { ili9341_spi_slave_release((d), (s)); }

#define __U16_MSBYTE(u) (uint8_t)(((uint16_t)(u) >> 8U) & 0xFF)
#define __U16_LSBYTE(u) (uint8_t)(((uint16_t)(u)      ) & 0xFF)

// ------------------------------------------------------------ private types --

/* nothing */

// ------------------------------------------------------- exported variables --

ili9341_color_t const ILI9341_BLACK   = (ili9341_color_t)0x0000;
ili9341_color_t const ILI9341_BLUE    = (ili9341_color_t)0x001F;
ili9341_color_t const ILI9341_RED     = (ili9341_color_t)0xF800;
ili9341_color_t const ILI9341_GREEN   = (ili9341_color_t)0x07E0;
ili9341_color_t const ILI9341_CYAN    = (ili9341_color_t)0x07FF;
ili9341_color_t const ILI9341_MAGENTA = (ili9341_color_t)0xF81F;
ili9341_color_t const ILI9341_YELLOW  = (ili9341_color_t)0xFFE0;
ili9341_color_t const ILI9341_WHITE   = (ili9341_color_t)0xFFFF;

// -------------------------------------------------------- private variables --

#if defined(__ILI9341_STATIC_MEM_ALLOC__)
static uint8_t __spi_tx_block__[__SPI_TX_BLOCK_SZ__]; // do not use, theoretically broken
#endif

// ---------------------------------------------- private function prototypes --

static void ili9341_reset(ili9341_device_t *dev);
static void ili9341_initialize(ili9341_device_t *dev);
static ili9341_two_dimension_t ili9341_screen_size(
    ili9341_screen_orientation_t orientation);
static uint8_t ili9341_screen_rotation(
    ili9341_screen_orientation_t orientation);

static uint32_t ili9341_alloc_spi_tx_block(ili9341_device_t *dev,
    uint32_t tx_block_sz, uint8_t **tx_block);
static void ili9341_free_spi_tx_block(ili9341_device_t *dev,
    uint8_t **tx_block);
static void ili9341_init_spi_tx_block(ili9341_device_t *dev,
    uint32_t tx_block_sz, uint8_t **tx_block);

static void ili9341_spi_tft_select(ili9341_device_t *dev);
static void ili9341_spi_tft_release(ili9341_device_t *dev);
static void ili9341_spi_touch_select(ili9341_device_t *dev);
static void ili9341_spi_touch_release(ili9341_device_t *dev);
static void ili9341_spi_slave_select(ili9341_device_t *dev,
    ili9341_spi_slave_t spi_slave);
static void ili9341_spi_slave_release(ili9341_device_t *dev,
    ili9341_spi_slave_t spi_slave);

static void ili9341_spi_write_command(ili9341_device_t *dev,
    ili9341_spi_slave_t spi_slave, uint8_t command);
static void ili9341_spi_write_data(ili9341_device_t *dev,
    ili9341_spi_slave_t spi_slave, uint16_t data_sz, uint8_t data[]);
static void ili9341_spi_write_command_data(ili9341_device_t *dev,
    ili9341_spi_slave_t spi_slave, uint8_t command, uint16_t data_sz, uint8_t data[]);

static void ili9341_spi_tft_set_address_rect(ili9341_device_t *dev,
    uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

// ------------------------------------------------------- exported functions --

ili9341_device_t *ili9341_device_new(

    SPI_HandleTypeDef *spi_hal,

    GPIO_TypeDef *reset_port,        uint16_t reset_pin,
    GPIO_TypeDef *tft_select_port,   uint16_t tft_select_pin,
    GPIO_TypeDef *data_command_port, uint16_t data_command_pin,

    ili9341_screen_orientation_t orientation,

    GPIO_TypeDef *touch_select_port, uint16_t touch_select_pin,
    GPIO_TypeDef *touch_irq_port,    uint16_t touch_irq_pin,

    ili9341_touch_support_t touch_support,
    uint16_t touch_coordinate_min_x,
    uint16_t touch_coordinate_min_y,
    uint16_t touch_coordinate_max_x,
    uint16_t touch_coordinate_max_y)
{
  ili9341_device_t *dev = NULL;

  if (NULL != spi_hal) {

    if ( (NULL != reset_port)        && IS_GPIO_PIN(reset_pin)         &&
         (NULL != tft_select_port)   && IS_GPIO_PIN(tft_select_pin)    &&
         (NULL != data_command_port) && IS_GPIO_PIN(data_command_pin)  &&
         (orientation > isoNONE)     && (orientation < isoCOUNT)       ) {

      // we must either NOT support the touch interface, OR we must have valid
      // touch interface parameters
      if ( itsSupported != touch_support ||
           ( (NULL != touch_select_port) && IS_GPIO_PIN(touch_select_pin) &&
             (NULL != touch_irq_port)    && IS_GPIO_PIN(touch_irq_pin)    &&
             (touch_coordinate_min_x < touch_coordinate_max_x)            &&
             (touch_coordinate_min_y < touch_coordinate_max_y)            )) {

        if (NULL != (dev = malloc(sizeof(ili9341_device_t)))) {

          dev->spi_hal              = spi_hal;

          dev->reset_port           = reset_port;
          dev->reset_pin            = reset_pin;
          dev->tft_select_port      = tft_select_port;
          dev->tft_select_pin       = tft_select_pin;
          dev->data_command_port    = data_command_port;
          dev->data_command_pin     = data_command_pin;

          dev->orientation          = orientation;
          dev->screen_size          = ili9341_screen_size(orientation);

          if (touch_support) {

            dev->touch_select_port    = touch_select_port;
            dev->touch_select_pin     = touch_select_pin;
            dev->touch_irq_port       = touch_irq_port;
            dev->touch_irq_pin        = touch_irq_pin;

            dev->touch_support        = touch_support;
            dev->touch_coordinate_min = (ili9341_two_dimension_t){
                {touch_coordinate_min_x},
                {touch_coordinate_min_y} };
            dev->touch_coordinate_max = (ili9341_two_dimension_t){
                {touch_coordinate_max_x},
                {touch_coordinate_max_y} };

            dev->touch_pressed        = ili9341_touch_pressed(dev);
            dev->touch_pressed_begin  = NULL;
            dev->touch_pressed_end    = NULL;

          } else {

            dev->touch_select_port    = NULL;
            dev->touch_select_pin     = 0;
            dev->touch_irq_port       = NULL;
            dev->touch_irq_pin        = 0;

            dev->touch_support        = touch_support;
            dev->touch_coordinate_min = (ili9341_two_dimension_t){ {0U}, {0U} };
            dev->touch_coordinate_max = (ili9341_two_dimension_t){ {0U}, {0U} };

            dev->touch_pressed        = itpNONE;
            dev->touch_pressed_begin  = NULL;
            dev->touch_pressed_end    = NULL;
          }

          ili9341_initialize(dev);
          ili9341_fill_screen(dev, ILI9341_BLACK);
        }
      }
    }
  }

  return dev;
}

void ili9341_fill_rect(ili9341_device_t *dev, ili9341_color_t color,
    uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  // adjust bounds to clip any area outside screen dimensions:
  //  1. rect origin beyond screen dimensions, nothing to draw
  if ((x >= dev->screen_size.width) || (y >= dev->screen_size.height))
    { return; }
  //  2. rect width beyond screen width, reduce rect width
  if ((x + w - 1) >= dev->screen_size.width)
    { w = dev->screen_size.width - x; }
  //  3. rect height beyond screen height, reduce rect height
  if ((y + h - 1) >= dev->screen_size.height)
    { h = dev->screen_size.height - y; }

  // 16-bit color, so need 2 bytes for each pixel being filled
  uint32_t num_pixels = w * h;
  uint32_t rect_sz    = 2 * num_pixels;

  // allocate largest block required to define rect
  uint8_t *block = NULL;
  uint32_t block_sz = ili9341_alloc_spi_tx_block(dev, rect_sz, &block);

  // failed to allocate a usable block of memory
  if (0 == block_sz || NULL == block)
    { return; }

  // fill entire block with ordered color data
  uint8_t msb = __U16_MSBYTE(color);
  uint8_t lsb = __U16_LSBYTE(color);
  for (uint16_t i = 0; i < block_sz; i += 2) {
    block[i + 0] = msb;
    block[i + 1] = lsb;
  }

  // select target region
  ili9341_spi_tft_set_address_rect(dev, x, y, (x + w - 1), (y + h - 1));
  ili9341_spi_tft_select(dev);

  // repeatedly send MIN(remaining-size, block-size) bytes of color data until
  // all rect bytes have been sent.
  HAL_GPIO_WritePin(dev->data_command_port, dev->data_command_pin, __GPIO_PIN_SET__);
  uint32_t curr_sz;
  while (rect_sz > 0) {
    curr_sz = rect_sz;
    if (curr_sz > block_sz)
      { curr_sz = block_sz; }
    HAL_SPI_Transmit(dev->spi_hal, block, curr_sz, __SPI_MAX_DELAY__);
    rect_sz -= curr_sz;
  }

  // cleanup, be sure to free the potentially-massive buffer
  ili9341_free_spi_tx_block(dev, &block);
  ili9341_spi_tft_release(dev);
}

void ili9341_fill_screen(ili9341_device_t *dev, ili9341_color_t color)
{
  ili9341_fill_rect(dev, color,
      0, 0, dev->screen_size.width, dev->screen_size.height);
}

void ili9341_draw_char(ili9341_device_t *dev, uint16_t x, uint16_t y,
    ili9341_font_t const *font, ili9341_color_t fg_color, ili9341_color_t bg_color,
    char ch)
{
  // 16-bit color, so need 2 bytes for each pixel being filled
  uint32_t num_pixels = font->width * font->height;
  uint32_t rect_sz    = 2 * num_pixels;

  uint8_t fg_msb = __U16_MSBYTE(fg_color);
  uint8_t fg_lsb = __U16_LSBYTE(fg_color);
  uint8_t bg_msb = __U16_MSBYTE(bg_color);
  uint8_t bg_lsb = __U16_LSBYTE(bg_color);
  uint8_t msb, lsb;

  // allocate largest block required to define rect
  uint8_t *block = NULL;
  uint32_t block_sz = ili9341_alloc_spi_tx_block(dev, rect_sz, &block);

  // failed to allocate a usable block of memory
  if (0 == block_sz || NULL == block)
    { return; }

  ili9341_fill_rect(dev, bg_color, x, y, font->width, font->height);

  // initialize the buffer with glyph from selected font
  uint8_t ch_index = glyph_index(ch);
  for (uint32_t yi = 0; yi < font->height; ++yi) {
    uint32_t gl = (uint32_t)font->glyph[ch_index * font->height + yi];
    for (uint32_t xi = 0; xi < font->width; ++xi) {
      if ((gl << xi) & 0x8000) {
        msb = fg_msb;
        lsb = fg_lsb;
      } else {
        msb = bg_msb;
        lsb = bg_lsb;
      }
      block[2 * (yi * font->width + xi) + 0] = msb;
      block[2 * (yi * font->width + xi) + 1] = lsb;
    }
  }

  // select target region
  ili9341_spi_tft_set_address_rect(dev, x, y, x + font->width - 1, y + font->height - 1);
  ili9341_spi_tft_select(dev);

  // transmit the character data in a single block transfer
  HAL_GPIO_WritePin(dev->data_command_port, dev->data_command_pin, __GPIO_PIN_SET__);
  HAL_SPI_Transmit(dev->spi_hal, block, block_sz, __SPI_MAX_DELAY__);

  // cleanup, be sure to free the transmit buffer
  ili9341_free_spi_tx_block(dev, &block);
  ili9341_spi_tft_release(dev);
}

void ili9341_draw_string(ili9341_device_t *dev, uint16_t x, uint16_t y,
    ili9341_font_t const *font, ili9341_color_t fg_color, ili9341_color_t bg_color,
    ili9341_word_wrap_t word_wrap, char str[])
{
  uint16_t curr_x = x;
  uint16_t curr_y = y;

  while ('\0' != *str) {
    if (iwwTruncate == word_wrap) {
      if ( (curr_x > dev->screen_size.width) ||
           (curr_y > dev->screen_size.height) )
        { break; }
    }
    else if (iwwCharWrap == word_wrap) {
      /* TODO */
      break;
    }
    else if (iwwWordWrap == word_wrap) {
      /* TODO */
      break;
    }

    ili9341_draw_char(dev, curr_x, curr_y, font, fg_color, bg_color, *str);

    curr_x += font->width;
    ++str;
  }
}

void ili9341_touch_interrupt(ili9341_device_t *dev)
{
  // read the new/incoming state of the touch screen
  ili9341_touch_pressed_t pressed = ili9341_touch_pressed(dev);

  // switch path based on existing/prior state of the touch screen
  switch (dev->touch_pressed) {
    case itpNotPressed:
      if (itpPressed == pressed) {
        // state change, start of press
        if (NULL != dev->touch_pressed_begin) {
          dev->touch_pressed_begin(dev);
        }
      }
      break;

    case itpPressed:
      if (itpNotPressed == pressed) {
        // state change, end of press
        if (NULL != dev->touch_pressed_end) {
          dev->touch_pressed_end(dev);
        }
      }
      break;

    default:
      break;
  }

  // update the internal state with current state of touch screen
  if (pressed != dev->touch_pressed) {
    dev->touch_pressed = pressed;
  }
}

ili9341_touch_pressed_t ili9341_touch_pressed(ili9341_device_t *dev)
{
  if (__GPIO_PIN_CLR__ == HAL_GPIO_ReadPin(dev->touch_irq_port, dev->touch_irq_pin))
    { return itpPressed; }
  else
    { return itpNotPressed; }
}

void ili9341_set_touch_pressed_begin(ili9341_device_t *dev, ili9341_touch_callback_t callback)
{
  if ((NULL != dev) && (NULL != callback)) {
    dev->touch_pressed_begin = callback;
  }
}

void ili9341_set_touch_pressed_end(ili9341_device_t *dev, ili9341_touch_callback_t callback)
{
  if ((NULL != dev) && (NULL != callback)) {
    dev->touch_pressed_end = callback;
  }
}

// -------------------------------------------------------- private functions --

static void ili9341_reset(ili9341_device_t *dev)
{
  // the reset pin on ILI9341 is active low, so driving low temporarily will
  // reset the device (also resets the touch screen peripheral)
  HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, __GPIO_PIN_CLR__);
  HAL_Delay(200);
  HAL_GPIO_WritePin(dev->reset_port, dev->reset_pin, __GPIO_PIN_SET__);

  // ensure both slave lines are open
  ili9341_spi_tft_release(dev);
  ili9341_spi_touch_release(dev);
}

static void ili9341_initialize(ili9341_device_t *dev)
{
  ili9341_reset(dev);
  ili9341_spi_tft_select(dev);

  // command list is based on https://github.com/martnak/STM32-ILI9341

  // SOFTWARE RESET
  ili9341_spi_write_command(dev, issNONE, 0x01);
  HAL_Delay(1000);

  // POWER CONTROL A
  ili9341_spi_write_command_data(dev, issNONE,
      0xCB, 5, (uint8_t[]){ 0x39, 0x2C, 0x00, 0x34, 0x02 });

  // POWER CONTROL B
  ili9341_spi_write_command_data(dev, issNONE,
      0xCF, 3, (uint8_t[]){ 0x00, 0xC1, 0x30 });

  // DRIVER TIMING CONTROL A
  ili9341_spi_write_command_data(dev, issNONE,
      0xE8, 3, (uint8_t[]){ 0x85, 0x00, 0x78 });

  // DRIVER TIMING CONTROL B
  ili9341_spi_write_command_data(dev, issNONE,
      0xEA, 2, (uint8_t[]){ 0x00, 0x00 });

  // POWER ON SEQUENCE CONTROL
  ili9341_spi_write_command_data(dev, issNONE,
      0xED, 4, (uint8_t[]){ 0x64, 0x03, 0x12, 0x81 });

  // PUMP RATIO CONTROL
  ili9341_spi_write_command_data(dev, issNONE,
      0xF7, 1, (uint8_t[]){ 0x20 });

  // POWER CONTROL,VRH[5:0]
  ili9341_spi_write_command_data(dev, issNONE,
      0xC0, 1, (uint8_t[]){ 0x23 });

  // POWER CONTROL,SAP[2:0];BT[3:0]
  ili9341_spi_write_command_data(dev, issNONE,
      0xC1, 1, (uint8_t[]){ 0x10 });

  // VCM CONTROL
  ili9341_spi_write_command_data(dev, issNONE,
      0xC5, 2, (uint8_t[]){ 0x3E, 0x28 });

  // VCM CONTROL 2
  ili9341_spi_write_command_data(dev, issNONE,
      0xC7, 1, (uint8_t[]){ 0x86 });

  // MEMORY ACCESS CONTROL
  ili9341_spi_write_command_data(dev, issNONE,
      0x36, 1, (uint8_t[]){ 0x48 });

  // PIXEL FORMAT
  ili9341_spi_write_command_data(dev, issNONE,
      0x3A, 1, (uint8_t[]){ 0x55 });

  // FRAME RATIO CONTROL, STANDARD RGB COLOR
  ili9341_spi_write_command_data(dev, issNONE,
      0xB1, 2, (uint8_t[]){ 0x00, 0x18 });

  // DISPLAY FUNCTION CONTROL
  ili9341_spi_write_command_data(dev, issNONE,
      0xB6, 3, (uint8_t[]){ 0x08, 0x82, 0x27 });

  // 3GAMMA FUNCTION DISABLE
  ili9341_spi_write_command_data(dev, issNONE,
      0xF2, 1, (uint8_t[]){ 0x00 });

  // GAMMA CURVE SELECTED
  ili9341_spi_write_command_data(dev, issNONE,
      0x26, 1, (uint8_t[]){ 0x01 });

  // POSITIVE GAMMA CORRECTION
  ili9341_spi_write_command_data(dev, issNONE,
      0xE0, 15, (uint8_t[]){ 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1,
                             0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00 });

  // NEGATIVE GAMMA CORRECTION
  ili9341_spi_write_command_data(dev, issNONE,
      0xE1, 15, (uint8_t[]){ 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,
                             0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F });

  // EXIT SLEEP
  ili9341_spi_write_command(dev, issNONE, 0x11);
  HAL_Delay(120);

  // TURN ON DISPLAY
  ili9341_spi_write_command(dev, issNONE, 0x29);

  // MADCTL
  ili9341_spi_write_command_data(dev, issNONE,
      0x36, 1, (uint8_t[]){ ili9341_screen_rotation(dev->orientation) });

  ili9341_spi_tft_release(dev);
}

static ili9341_two_dimension_t ili9341_screen_size(
    ili9341_screen_orientation_t orientation)
{
  switch (orientation) {
    default:
    case isoDown:
      return (ili9341_two_dimension_t){ { .width = 240U }, { .height = 320U } };
    case isoRight:
      return (ili9341_two_dimension_t){ { .width = 320U }, { .height = 240U } };
    case isoUp:
      return (ili9341_two_dimension_t){ { .width = 240U }, { .height = 320U } };
    case isoLeft:
      return (ili9341_two_dimension_t){ { .width = 320U }, { .height = 240U } };
  }
}

static uint8_t ili9341_screen_rotation(
    ili9341_screen_orientation_t orientation)
{
  switch (orientation) {
    default:
    case isoDown:
      return 0x40 | 0x08;
    case isoRight:
      return 0x40 | 0x80 | 0x20 | 0x08;
    case isoUp:
      return 0x80 | 0x08;
    case isoLeft:
      return 0x20 | 0x08;
  }
}

static uint32_t ili9341_alloc_spi_tx_block(ili9341_device_t *dev,
    uint32_t tx_block_sz, uint8_t **tx_block)
{
  if (tx_block_sz > __SPI_TX_BLOCK_SZ__)
    { tx_block_sz = __SPI_TX_BLOCK_SZ__; }

#if defined(__ILI9341_STATIC_MEM_ALLOC__)
  *tx_block = __spi_tx_block__;
#else
  *tx_block = malloc(tx_block_sz);
#endif

  return tx_block_sz;
}

static void ili9341_free_spi_tx_block(ili9341_device_t *dev,
    uint8_t **tx_block)
{
#if defined(__ILI9341_STATIC_MEM_ALLOC__)
  ; /* nothing */
#else
  free(*tx_block);
#endif
}

static void ili9341_init_spi_tx_block(ili9341_device_t *dev,
    uint32_t tx_block_sz, uint8_t **tx_block)
{
  if (NULL != *tx_block) {
    for (size_t i = 0U; i < tx_block_sz; ++i)
      { (*tx_block)[i] = 0U; }
  }
}

static void ili9341_spi_tft_select(ili9341_device_t *dev)
{
  // clear bit indicates the TFT is -active- slave SPI device
  HAL_GPIO_WritePin(dev->tft_select_port, dev->tft_select_pin, __GPIO_PIN_CLR__);
}

static void ili9341_spi_tft_release(ili9341_device_t *dev)
{
  // set bit indicates the TFT is -inactive- slave SPI device
  HAL_GPIO_WritePin(dev->tft_select_port, dev->tft_select_pin, __GPIO_PIN_SET__);
}

static void ili9341_spi_touch_select(ili9341_device_t *dev)
{
  // clear bit indicates the touch screen is -active- slave SPI device
  HAL_GPIO_WritePin(dev->touch_select_port, dev->touch_select_pin, __GPIO_PIN_CLR__);
}

static void ili9341_spi_touch_release(ili9341_device_t *dev)
{
  // set bit indicates the touch screen is -inactive- slave SPI device
  HAL_GPIO_WritePin(dev->touch_select_port, dev->touch_select_pin, __GPIO_PIN_SET__);
}

static void ili9341_spi_slave_select(ili9341_device_t *dev,
    ili9341_spi_slave_t spi_slave)
{
  switch (spi_slave) {
    case issDisplayTFT:  ili9341_spi_tft_select(dev);   break;
    case issTouchScreen: ili9341_spi_touch_select(dev); break;
    default: break;
  }
}

static void ili9341_spi_slave_release(ili9341_device_t *dev,
    ili9341_spi_slave_t spi_slave)
{
  switch (spi_slave) {

    case issDisplayTFT:  ili9341_spi_tft_release(dev);   break;
    case issTouchScreen: ili9341_spi_touch_release(dev); break;
    default: break;
  }
}

static void ili9341_spi_write_command(ili9341_device_t *dev,
    ili9341_spi_slave_t spi_slave, uint8_t command)
{
  __SLAVE_SELECT(dev, spi_slave);

  HAL_GPIO_WritePin(dev->data_command_port, dev->data_command_pin, __GPIO_PIN_CLR__);
  HAL_SPI_Transmit(dev->spi_hal, &command, sizeof(command), __SPI_MAX_DELAY__);

  __SLAVE_RELEASE(dev, spi_slave);
}

static void ili9341_spi_write_data(ili9341_device_t *dev,
    ili9341_spi_slave_t spi_slave, uint16_t data_sz, uint8_t data[])
{
  __SLAVE_SELECT(dev, spi_slave);

  HAL_GPIO_WritePin(dev->data_command_port, dev->data_command_pin, __GPIO_PIN_SET__);
  HAL_SPI_Transmit(dev->spi_hal, data, data_sz, __SPI_MAX_DELAY__);

  __SLAVE_RELEASE(dev, spi_slave);
}

static void ili9341_spi_write_command_data(ili9341_device_t *dev,
    ili9341_spi_slave_t spi_slave, uint8_t command, uint16_t data_sz, uint8_t data[])
{
  __SLAVE_SELECT(dev, spi_slave);

  ili9341_spi_write_command(dev, issNONE, command);
  ili9341_spi_write_data(dev, issNONE, data_sz, data);

  __SLAVE_RELEASE(dev, spi_slave);
}

static void ili9341_spi_tft_set_address_rect(ili9341_device_t *dev,
    uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  ili9341_spi_tft_select(dev);

  // column address set
  ili9341_spi_write_command_data(dev, issNONE,
      0x2A, 4, (uint8_t[]){ (x0 >> 8) & 0xFF, x0 & 0xFF,
                            (x1 >> 8) & 0xFF, x1 & 0xFF });

  // row address set
  ili9341_spi_write_command_data(dev, issNONE,
      0x2B, 4, (uint8_t[]){ (y0 >> 8) & 0xFF, y0 & 0xFF,
                            (y1 >> 8) & 0xFF, y1 & 0xFF });

  // write to RAM
  ili9341_spi_write_command(dev, issNONE, 0x2C);

  ili9341_spi_tft_release(dev);
}
