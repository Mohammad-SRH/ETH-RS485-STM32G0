/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "w5500.h"
#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t GW[4];
	uint8_t MAC[6];
	uint8_t SUBNET[4];
	uint8_t IPadd[4];
	uint8_t DEST_IPADD[4];
	uint16_t SOURCE_PORT;
	uint16_t DEST_PORT;
}WIZNetworksetting_t;


const uint8_t 	defaultgateway[]={192,168,1,1};
const uint8_t	defaultmac[]={0x22,0x33,0x44,0x55,0x66,0x77};
const uint8_t	defaultsubnet[]={255,255,255,0};
const uint8_t	defaultip[]={192,168,1,200};
const uint8_t	defaultdestip[]={192,168,1,255};
const int8_t    waitForPHY[]={"Waiting For PHY...\r\n"};
const int8_t	PHYReady[]={"PHY On.\r\n"};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Global variables ---------------------------------------------------------*/
WIZNetworksetting_t g_WIZNetworkSetting;
volatile uint8_t g_usart2RxcommBuf[64];
volatile uint8_t g_usart2_DataRDY = 0;
volatile uint8_t g_UDPdataRDY = 0;
volatile uint8_t g_usart2_DmaData[64];
uint8_t g_udpData[64];
volatile uint16_t g_temp;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t udp_dataSize = 0;
	int8_t strbuff[64];
	
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
	LL_SPI_SetRxFIFOThreshold(SPI1,LL_SPI_RX_FIFO_TH_QUARTER);
	LL_SPI_Enable(SPI1);	//Enable SPI1
	while( LL_SPI_IsEnabled(SPI1) == 0);	//Wait To Enable SPI
	 
	LL_USART_Enable(USART2);
	while( LL_USART_IsEnabled(USART2) == 0); //Wait To Enable USART
	 
	usart2_SendAnswer_DMA(strlen(waitForPHY),waitForPHY);
	while(!(getPHYCFGR() & PHYCFGR_LNK_ON));   //Wait for PHY On
	usart2_SendAnswer_DMA(strlen(PHYReady),PHYReady);
	
	
	
	setSn_RXBUF_SIZE(SOCKET_UDP,8);
	setSn_TXBUF_SIZE(SOCKET_UDP,8);
	setSn_MR(SOCKET_UDP,Sn_MR_UDP);  //Set UDP
	g_temp = getSn_MR(SOCKET_UDP);
	setSn_IR(SOCKET_UDP,0x1F);
	setSn_CR(SOCKET_UDP,Sn_CR_OPEN);
	while((getSn_SR(SOCKET_UDP) != 0x22));
	
	setGAR((memcpy(g_WIZNetworkSetting.GW,defaultgateway,GATEWAYSIZE)));
	setSHAR(memcpy(g_WIZNetworkSetting.MAC,defaultmac,MACSIZE));
	setSUBR(memcpy(g_WIZNetworkSetting.SUBNET,defaultsubnet,SUBNETSIZE));
	setSIPR(memcpy(g_WIZNetworkSetting.IPadd,defaultip,IPSIZE));
	setSn_DIPR(SOCKET_UDP,memcpy(g_WIZNetworkSetting.DEST_IPADD,defaultdestip,IPSIZE));
	setSn_PORT(SOCKET_UDP,DEFAULT_SOURCEPORT);
	setSn_DPORT(SOCKET_UDP,DEFAULT_DESTPORT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  udp_dataSize = WIZ_recvudp(g_udpData);
	  if(g_UDPdataRDY ==1){
		usart2_SendAnswer_DMA(strlen(g_udpData),g_udpData);
		while(g_usart2_DataRDY == 0);
		usart2_SendAnswer_DMA(strlen(g_usart2RxcommBuf),g_usart2RxcommBuf);
		g_usart2_DataRDY = 0;
		g_UDPdataRDY = 0;
	  }
		
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_2);
}

/* USER CODE BEGIN 4 */
/**
  * @brief System Clock Configuration
  * @retval None
  */
void HAL_UART_IRQHandlerCallBack(void){
	
	uint8_t data;
	static 	uint8_t usart2RxBuf[64];
	static uint8_t usart2RxBuf_Counter=0;
	static uint8_t rxisr_frameSize=0;
	uint32_t isrflags   = LL_USART_ReadReg(USART2,ISR);
	uint32_t errorflags = 0x00U;
	uint8_t i=0;

	
  /* If no error occurs */
	data = LL_USART_ReceiveData8 (USART2);
	errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
	if(errorflags == RESET)
	{
		switch(data){
		case Start_Byte :
			usart2RxBuf_Counter=0;
			usart2RxBuf[usart2RxBuf_Counter] = data;
			usart2RxBuf_Counter++; 
		break;
		case Stop_Byte :
			usart2RxBuf[usart2RxBuf_Counter] = data;
			usart2RxBuf_Counter++;
			rxisr_frameSize=usart2RxBuf_Counter;
			usart2RxBuf_Counter=0;
			for(i=0;i<rxisr_frameSize;i++){
				g_usart2RxcommBuf[i]=usart2RxBuf[i];    
			}
			usart2RxBuf_Counter = 0;
			g_usart2_DataRDY = 1;
		break;
		default :
			usart2RxBuf[usart2RxBuf_Counter] = data;
			usart2RxBuf_Counter++;          
		break;
		}
	}
	LL_USART_ClearFlag_ORE(USART2);	
	LL_USART_ClearFlag_PE(USART2);	
	LL_USART_ClearFlag_FE(USART2);	
	LL_USART_ClearFlag_NE(USART2);			
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void usart2_SendAnswer_DMA (uint8_t length, uint8_t* ptr){

	LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_1); //DISABLE channel to be able to change size
	LL_DMA_SetMemoryAddress(DMA1,LL_DMA_CHANNEL_1,(uint32_t) ptr); //memory address
	LL_DMA_SetPeriphAddress(DMA1,LL_DMA_CHANNEL_1,LL_USART_DMA_GetRegAddr(USART2,LL_USART_DMA_REG_DATA_TRANSMIT));// USART TRANSSMIT register address
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1,length);		// set counter
	LL_DMA_ClearFlag_GI2(DMA1);	//clearerr Channel 2 DMA Flags
	LL_USART_ClearFlag_TC(USART2);     // Clear tc flag
	LL_USART_EnableDMAReq_TX (USART2); // ENABLE USART DMA TRanssmit Req 
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);// enable channel 2
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
uint8_t WIZ_recvudp (uint8_t *data ){
	
uint16_t ptr = 0;
uint32_t addrsel = 0;
uint8_t header_size = 8;
uint8_t ctrlSize = 0;
uint8_t header[8];
uint16_t datasize = 0;

	
//	g_temp = getSn_IR(SOCKET_UDP);

    if((getSn_IR(SOCKET_UDP) & Sn_IR_RECV ) && (getSn_IMR(SOCKET_UDP) & Sn_IR_RECV ) )
	{
        //LL_USART_TransmitData8(USART2,0xff);
		setSn_IR(SOCKET_UDP , Sn_IR_RECV );
        while((getSn_IMR(SOCKET_UDP) & Sn_IR_RECV) == 0);
        ptr = getSn_RX_RD(SOCKET_UDP);
        ctrlSize = getSn_RXBUF_SIZE(SOCKET_UDP);
        addrsel = ((uint32_t)ptr << 8) + (WIZCHIP_RXBUF_BLOCK(SOCKET_UDP) << 3);
        WIZCHIP_READ_BUF(addrsel, header, header_size);
        ptr += header_size;
        datasize = (header[6] << 8);
        datasize += header[7];
        addrsel = ((uint32_t)ptr << 8) + (WIZCHIP_RXBUF_BLOCK(SOCKET_UDP) << 3);
        WIZCHIP_READ_BUF(addrsel, data, datasize); 
        ptr += datasize;
        setSn_RX_RD(SOCKET_UDP,ptr);
        setSn_CR(SOCKET_UDP,Sn_CR_RECV );
        
        g_UDPdataRDY = 1;
		
//        return datasize;          
    }
	return datasize;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
