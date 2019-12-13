/*
 * ili9341_api.h
 *
 *  Created on: Nov 28, 2019
 *      Author: andrew
 */

#ifndef __ILI9341_API_H
#define __ILI9341_API_H

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

#include "ili9341_font.h"

// ------------------------------------------------------------------ defines --

// whether memory should be allocated as-needed for the SPI block transfers, or
// if they should share a common pool allocated at compile-time. static
// allocation may be faster, but it's a waste of RAM if you aren't frequently
// doing full-screen refreshes.
//#define __ILI9341_STATIC_MEM_ALLOC__

// ------------------------------------------------------------------- macros --

#define __ILI9341_COLOR565(r, g, b) \
    (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

// ----------------------------------------------------------- exported types --

typedef struct ili9341_device ili9341_device_t;

typedef struct
{
  union
  {
    uint16_t x;
    uint16_t width;
  };
  union
  {
    uint16_t y;
    uint16_t height;
  };
}
ili9341_two_dimension_t;

typedef uint16_t ili9341_color_t;

typedef enum
{
  // orientation is based on position of board pins when looking at the screen
  isoNONE                     = -1,
  isoDown,   isoPortrait      = isoDown,  // = 0
  isoRight,  isoLandscape     = isoRight, // = 1
  isoUp,     isoPortraitFlip  = isoUp,    // = 2
  isoLeft,   isoLandscapeFlip = isoLeft,  // = 3
  isoCOUNT                                // = 4
}
ili9341_screen_orientation_t;

typedef enum
{
  itsNONE = -1,
  itsNotSupported, // = 0
  itsSupported,    // = 1
  itsCOUNT         // = 2
}
ili9341_touch_support_t;

typedef enum
{
  itpNONE = -1,
  itpNotPressed, // = 0
  itpPressed,    // = 1
  itpCOUNT       // = 2
}
ili9341_touch_pressed_t;

typedef enum
{
  itnNONE = -1,
  itnNotNormalized, // = 0
  itnNormalized,    // = 1
  itnCOUNT          // = 2
}
ili9341_touch_normalize_t;

typedef enum
{
  iwwNONE = -1,
  iwwTruncate, // = 0
  iwwCharWrap, // = 1
  iwwWordWrap, // = 2
  iwwCOUNT     // = 3
}
ili9341_word_wrap_t;

typedef enum
{
  issNONE = -1,
  issDisplayTFT,  // = 0
  issTouchScreen, // = 1
  issCOUNT        // = 2
}
ili9341_spi_slave_t;

typedef void (*ili9341_touch_callback_t)(ili9341_device_t *, uint16_t, uint16_t);

typedef HAL_StatusTypeDef ili9341_status_t;

struct ili9341_device
{
  SPI_HandleTypeDef *spi_hal;

  GPIO_TypeDef *reset_port;
  uint16_t      reset_pin;
  GPIO_TypeDef *tft_select_port;
  uint16_t      tft_select_pin;
  GPIO_TypeDef *data_command_port;
  uint16_t      data_command_pin;

  ili9341_screen_orientation_t orientation;
  ili9341_two_dimension_t      screen_size;

  GPIO_TypeDef *touch_select_port;
  uint16_t      touch_select_pin;
  GPIO_TypeDef *touch_irq_port;
  uint16_t      touch_irq_pin;

  ili9341_touch_support_t   touch_support;
  ili9341_touch_normalize_t touch_normalize;
  ili9341_two_dimension_t   touch_coordinate;
  ili9341_two_dimension_t   touch_coordinate_min;
  ili9341_two_dimension_t   touch_coordinate_max;

  ili9341_touch_pressed_t  touch_pressed;
  ili9341_touch_callback_t touch_pressed_begin;
  ili9341_touch_callback_t touch_pressed_end;
};

// ------------------------------------------------------- exported variables --

extern ili9341_color_t const ILI9341_BLACK;
extern ili9341_color_t const ILI9341_BLUE;
extern ili9341_color_t const ILI9341_RED;
extern ili9341_color_t const ILI9341_GREEN;
extern ili9341_color_t const ILI9341_CYAN;
extern ili9341_color_t const ILI9341_MAGENTA;
extern ili9341_color_t const ILI9341_YELLOW;
extern ili9341_color_t const ILI9341_WHITE;

// ------------------------------------------------------- exported functions --

ili9341_device_t *ili9341_device_new(

    SPI_HandleTypeDef *spi_hal,

    GPIO_TypeDef *reset_port,        uint16_t reset_pin,
    GPIO_TypeDef *tft_select_port,   uint16_t tft_select_pin,
    GPIO_TypeDef *data_command_port, uint16_t data_command_pin,

    ili9341_screen_orientation_t orientation,

    GPIO_TypeDef *touch_select_port, uint16_t touch_select_pin,
    GPIO_TypeDef *touch_irq_port,    uint16_t touch_irq_pin,

    ili9341_touch_support_t   touch_support,
    ili9341_touch_normalize_t touch_normalize,
    uint16_t touch_coordinate_min_x,
    uint16_t touch_coordinate_min_y,
    uint16_t touch_coordinate_max_x,
    uint16_t touch_coordinate_max_y);

void ili9341_fill_rect(ili9341_device_t *dev, ili9341_color_t color,
    uint16_t x, uint16_t y, uint16_t w, uint16_t h);
void ili9341_fill_screen(ili9341_device_t *dev, ili9341_color_t color);
void ili9341_draw_char(ili9341_device_t *dev, uint16_t x, uint16_t y,
    ili9341_font_t const *font, ili9341_color_t fg_color, ili9341_color_t bg_color,
    char ch);
void ili9341_draw_string(ili9341_device_t *dev, uint16_t x, uint16_t y,
    ili9341_font_t const *font, ili9341_color_t fg_color, ili9341_color_t bg_color,
    ili9341_word_wrap_t word_wrap, char str[]);

void ili9341_touch_interrupt(ili9341_device_t *dev);
ili9341_touch_pressed_t ili9341_touch_pressed(ili9341_device_t *dev);

void ili9341_set_touch_pressed_begin(ili9341_device_t *dev, ili9341_touch_callback_t callback);
void ili9341_set_touch_pressed_end(ili9341_device_t *dev, ili9341_touch_callback_t callback);

ili9341_touch_pressed_t ili9341_touch_coordinate(ili9341_device_t *dev,
    uint16_t *x_pos, uint16_t *y_pos);

#ifdef __cplusplus
}
#endif

#endif /* __ILI9341_API_H */
