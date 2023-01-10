/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

#include "stm32g0xx_ll_spi.h"
#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_dma.h"

#include "stm32g0xx_ll_exti.h"

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

void usart2_SendAnswer_DMA(uint8_t length, uint8_t* ptr);
uint8_t WIZ_recvudp (uint8_t *data );

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MEM_WP_Pin GPIO_PIN_15
#define MEM_WP_GPIO_Port GPIOC
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define ETH_INTx_Pin GPIO_PIN_11
#define ETH_INTx_GPIO_Port GPIOA
#define ETH_RESET_Pin GPIO_PIN_12
#define ETH_RESET_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
/* Protcol defines -----------------------------------------------------------*/

#define Start_Byte          0x02
#define Stop_Byte           0x03
#define Magic_Command       0x22
#define Locate_Commnad      0x23
#define Command_POS         2

/* IO defines -----------------------------------------------------------*/

#define Wiz_deSelect        HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port,SPI1_NSS_Pin,GPIO_PIN_SET)
#define Wiz_Select          HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port,SPI1_NSS_Pin,GPIO_PIN_RESET)

/* Wiznet defines -----------------------------------------------------------*/

#define SOCKET_UDP          0
#define SOCKET_TCP          1  

/* Network defines -----------------------------------------------------------*/

#define IPSIZE				4
#define SUBNETSIZE			4
#define	GATEWAYSIZE			4
#define MACSIZE				6
#define DEFAULT_SOURCEPORT	0xAFC8//45000
#define	DEFAULT_DESTPORT	0xABE1//44001



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
