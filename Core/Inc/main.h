/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TFT_DC_Pin GPIO_PIN_9
#define TFT_DC_GPIO_Port GPIOB
#define SYS_RESET_Pin GPIO_PIN_2
#define SYS_RESET_GPIO_Port GPIOF
#define TFT_SCK_Pin GPIO_PIN_1
#define TFT_SCK_GPIO_Port GPIOA
#define STLINK_TX_Pin GPIO_PIN_2
#define STLINK_TX_GPIO_Port GPIOA
#define STLINK_RX_Pin GPIO_PIN_3
#define STLINK_RX_GPIO_Port GPIOA
#define TFT_MISO_Pin GPIO_PIN_6
#define TFT_MISO_GPIO_Port GPIOA
#define TFT_MOSI_Pin GPIO_PIN_7
#define TFT_MOSI_GPIO_Port GPIOA
#define TFT_RESET_Pin GPIO_PIN_0
#define TFT_RESET_GPIO_Port GPIOB
#define TOUCH_CS_Pin GPIO_PIN_2
#define TOUCH_CS_GPIO_Port GPIOB
#define USBPD_RESET_Pin GPIO_PIN_8
#define USBPD_RESET_GPIO_Port GPIOA
#define USBPD_SCL_Pin GPIO_PIN_9
#define USBPD_SCL_GPIO_Port GPIOA
#define SYS_LED_GREEN_Pin GPIO_PIN_6
#define SYS_LED_GREEN_GPIO_Port GPIOC
#define USBPD_SDA_Pin GPIO_PIN_10
#define USBPD_SDA_GPIO_Port GPIOA
#define SYS_SWDIO_Pin GPIO_PIN_13
#define SYS_SWDIO_GPIO_Port GPIOA
#define SYS_SWCLK_Pin GPIO_PIN_14
#define SYS_SWCLK_GPIO_Port GPIOA
#define USBPD_IRQ_Pin GPIO_PIN_4
#define USBPD_IRQ_GPIO_Port GPIOB
#define USBPD_IRQ_EXTI_IRQn EXTI4_15_IRQn
#define TFT_CS_Pin GPIO_PIN_5
#define TFT_CS_GPIO_Port GPIOB
#define GP_TX_Pin GPIO_PIN_6
#define GP_TX_GPIO_Port GPIOB
#define GP_RX_Pin GPIO_PIN_7
#define GP_RX_GPIO_Port GPIOB
#define TOUCH_IRQ_Pin GPIO_PIN_8
#define TOUCH_IRQ_GPIO_Port GPIOB
#define TOUCH_IRQ_EXTI_IRQn EXTI4_15_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
