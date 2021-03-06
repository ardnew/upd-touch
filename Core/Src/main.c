/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stusb4500_api.h"
#include "ili9341_api.h"

#include <stdio.h> // snprintf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
stusb4500_device_t *usbpd;
ili9341_device_t *screen;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void draw_pdo_src_all(ili9341_device_t *scr, stusb4500_device_t *usb)
{
#define Y_START 20

  if ((NULL != scr) && (NULL != usb)) {

    char pdo_str[32];
    float volts;
    float amps;
    uint16_t watts;

    for (uint8_t i = 0; i < usb->usbpd_status.pdo_src_count; ++i) {

      volts = (float)(usb->usbpd_status.pdo_src[i].fix.Voltage) / 20.0F;
      amps  = (float)(usb->usbpd_status.pdo_src[i].fix.Max_Operating_Current) / 100.0F;
      watts = (uint16_t)(volts * amps);

      snprintf(pdo_str, 32, "(%u) %4.1fV %4.1fV %2uW", i + 1, volts, amps, watts);

      ili9341_draw_string(scr, 2, Y_START + 20 * i, &ili9341_font_11x18, ILI9341_WHITE, ILI9341_BLACK, iwwTruncate, pdo_str);
    }
  }

#undef Y_START
}

void draw_pdo_snk_all(ili9341_device_t *scr, stusb4500_device_t *usb)
{
#define Y_START (scr->screen_size.height / 2U)

  if ((NULL != scr) && (NULL != usb)) {

    char pdo_str[32];
    float volts;
    float volts_lo, volts_hi; // variable supply
    float amps;
    uint16_t watts;

    for (uint8_t i = 0; i < usb->usbpd_status.pdo_snk_count; ++i) {

      switch (usb->usbpd_status.pdo_snk[i].fix.Fixed_Supply) {

        case ssfFixed:
          volts = (float)(usb->usbpd_status.pdo_snk[i].fix.Voltage) / 20.0F;
          amps  = (float)(usb->usbpd_status.pdo_snk[i].fix.Operational_Current) / 100.0F;
          watts = (uint16_t)(volts * amps);
          snprintf(pdo_str, 32, "(%c) %4.1fV %4.1fA %2uW", 'F', volts, amps, watts);
          break;

        case ssfVariable:
          volts_lo = (float)(usb->usbpd_status.pdo_snk[i].var.Min_Voltage) / 20.0F;
          volts_hi = (float)(usb->usbpd_status.pdo_snk[i].var.Max_Voltage) / 20.0F;
          amps     = (float)(usb->usbpd_status.pdo_snk[i].var.Operating_Current) / 100.0F;
          snprintf(pdo_str, 32, "(%c) %4.1fV-%4.1fV %2uA", 'V', volts_lo, volts_hi, (uint16_t)amps);
          break;

        case ssfBattery:
          volts_lo = (float)(usb->usbpd_status.pdo_snk[i].bat.Min_Voltage) / 20.0F;
          volts_hi = (float)(usb->usbpd_status.pdo_snk[i].bat.Max_Voltage) / 20.0F;
          watts    = (uint16_t)(usb->usbpd_status.pdo_snk[i].bat.Operating_Power) / 4.0F;
          snprintf(pdo_str, 32, "(%c) %4.1fV-%4.1fA %2uW", 'B', volts_lo, volts_hi, watts);
          break;

        default:
          break;
      }
      ili9341_draw_string(scr, 2, Y_START + 20 * i, &ili9341_font_11x18, ILI9341_WHITE, ILI9341_BLACK, iwwTruncate, pdo_str);
    }
  }

#undef Y_START
}

void screen_touch_begin(ili9341_device_t *dev)
{
  ; /* nothing */
}

void screen_touch_end(ili9341_device_t *dev)
{
  static uint32_t mode = 0;
  if (NULL != usbpd) {
    //stusb4500_get_source_capabilities(usbpd);

    stusb4500_status_t status;

    switch(mode % 5) {
      case 0:
        status = stusb4500_set_power(usbpd,  5000, 3000);
        break;
      case 1:
        status = stusb4500_set_power(usbpd,  9000, 3000);
        break;
      case 2:
        status = stusb4500_set_power(usbpd, 12000, 3000);
        break;
      case 3:
        status = stusb4500_set_power(usbpd, 15000, 3000);
        break;
      case 4:
        status = stusb4500_set_power(usbpd, 20000, 4500);
        break;
    }
    ++mode;

    if (HAL_OK == status)
      { draw_pdo_snk_all(dev, usbpd); }
  }
}

void source_capabilities_received(stusb4500_device_t *dev)
{
  draw_pdo_src_all(screen, dev);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */


  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  usbpd = stusb4500_device_new(
      &hi2c1,
      __STUSB4500_I2C_SLAVE_BASE_ADDR__,
      USBPD_RESET_GPIO_Port, USBPD_RESET_Pin);

  stusb4500_set_source_capabilities_received(usbpd, source_capabilities_received);

  stusb4500_device_init(usbpd);

  screen = ili9341_device_new(
      &hspi1,
      TFT_RESET_GPIO_Port, TFT_RESET_Pin,
      TFT_CS_GPIO_Port,    TFT_CS_Pin,
      TFT_DC_GPIO_Port,    TFT_DC_Pin,
      isoPortrait,
      TOUCH_CS_GPIO_Port,  TOUCH_CS_Pin,
      TOUCH_IRQ_GPIO_Port, TOUCH_IRQ_Pin,
      itsSupported,
      1500, 3276, 31000, 30110);

  ili9341_set_touch_pressed_begin(screen, screen_touch_begin);
  ili9341_set_touch_pressed_end(screen, screen_touch_end);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    stusb4500_process_events(usbpd);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
  /* EXTI0_1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
  /* EXTI4_15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
