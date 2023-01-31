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

void usart2_SendAnswer_DMA(uint8_t length, uint8_t *ptr);
void HAL_TIM14_IRQHandlerCallBack (void);
uint8_t Frame_check (uint8_t *data,uint8_t len);
uint8_t WIZ_recvudp (uint8_t *data );
void WIZ_sendudp (uint8_t sn,uint8_t *data, uint16_t len);
void WIZ_defaultNetworkConfig (void);
void WIZ_linkCheck (void);
void WIZ_basicConfig (void);
void WIZ_networkConfig (void);

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
#define SOCKET_CONFIG		1

/* Network defines -----------------------------------------------------------*/

#define STATIC_FIRST_MAC	0x53
#define STATIC_SECOND_MAC	0x52
#define STATIC_THIRD_MAC	0x48 
#define SOCKET0_DEFAULT_SOURCEPORT	0xAFC8//45000
#define	SOCKET0_DEFAULT_DESTPORT	0xABE1//44001
#define SOCKET1_DEFAULT_SOURCEPORT	0xB02C//45100
#define	SOCKET1_DEFAULT_DESTPORT	0xB090//45200

/* Memory defines -----------------------------------------------------------*/

/*
|-------------------------------------|
|	PAGE0:MANUFATURE DATA 0			  |
|-------------------------------------|
|	PAGE1:MANUFATURE DATA 1			  |
|-------------------------------------|
|	PAGE2:IP ADDRESS				  |
|-------------------------------------|
|	PAGE3:MAC ADDRESS				  |
|-------------------------------------|
|	PAGE4:SUBNET MASK				  |
|-------------------------------------|
|	PAGE5:SOCKET0 DESTINATION IP	  |
|-------------------------------------|
|	PAGE6:SOCKET1 DESTINATION IP	  |
|-------------------------------------|
|	PAGE7:GATEWAY					  |
|-------------------------------------|
|	PAGE8:SOCKET0 SOURCE PORT		  |
|-------------------------------------|
|	PAGE9:SOCKET0 DESTINATION PORT	  |
|-------------------------------------|
|	PAGE10:SOCKET1 SOURCE PORT		  |
|-------------------------------------|
|	PAGE11:SOCKET1 DESTINATION PORT	  |
|-------------------------------------|

*/

#define MANUFACTURE_DATA0_PAGE					0
#define MANUFACTURE_DATA1_PAGE					1
#define IP_ADDRESS_PAGE							2
#define MAC_ADDRESS_PAGE						3
#define SUBNET_MASK_PAGE						4
#define SOCKET0_DESTINATION_IP_ADDRESS_PAGE		5
#define SOCKET1_DESTINATION_IP_ADDRESS_PAGE		6
#define GATEWAY_IP_ADDRESS_PAGE					7
#define SOCKET0_SOURCE_PORT_PAGE				8
#define SOCKET0_DESTINATION_PORT_PAGE			9
#define SOCKET1_SOURCE_PORT_PAGE				10
#define SOCKET1_DESTINATION_PORT_PAGE			11


#define MANUFACTURE_DADA_PROGRAM_BYTE	(MANUFACTURE_DATA0_PAGE*8)
#define MANUFACTURE_DADA__VERSION_BYTE	((MANUFACTURE_DATA0_PAGE*8)+1)	
#define SOCKET0_SOURCE_PORT_BYTE		(SOCKET0_SOURCE_PORT_PAGE*8)
#define SOCKET0_DESTINATION_PORT_BYTE	(SOCKET0_DESTINATION_PORT_PAGE*8)
#define SOCKET1_SOURCE_PORT_BYTE		(SOCKET1_SOURCE_PORT_PAGE*8)
#define SOCKET1_DESTINATION_PORT_BYTE	(SOCKET1_DESTINATION_PORT_PAGE*8)



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
