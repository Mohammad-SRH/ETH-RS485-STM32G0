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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "w5500.h"
#include "string.h"
#include "stdio.h"
#include "MemoryI2C.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t gateway[4];
	uint8_t mac[6];
	uint8_t subnet[4];
	uint8_t ipadd[4];
	uint8_t socket0_Destipadd[4];
	uint8_t socket1_Destipadd[4];
	uint16_t socket0_SourcePort;
	uint16_t socket0_DestPort;
	uint16_t socket1_SourcePort;
	uint16_t socket1_DestPort;
	
}WIZNetworksetting_t;


const uint8_t 	defaultgateway[]={192,168,1,1};
//const uint8_t	defaultmac[]={0x22,0x33,0x44,0x55,0x66,0x77};
const uint8_t	defaultsubnet[]={255,255,255,0};
const uint8_t	defaultip[]={192,168,1,200};
const uint8_t	defaultdestip[]={192,168,1,255};
const int8_t    waitForPHY[]={"Waiting For PHY On...\r\n"};
const int8_t	PHYReady[]={"PHY On.\r\n"};
const int8_t	config[]={"config Send To Server.\r\n"};

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
volatile uint8_t g_UDPconfigRDY = 0;
volatile uint8_t g_usart2_DmaData[64];
volatile uint8_t g_udpData[64];
volatile uint8_t g_timeOutCounter = 0;
volatile uint8_t g_UDP_Commbuf[64];
volatile uint8_t g_frameCheckFlag = 0;
volatile uint8_t g_rxisr_frameSize = 0;
volatile uint8_t g_flag =0 ;
		uint8_t g_temp[6];

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
	uint8_t frameSize = 0;

	
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
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
	
	LL_SPI_SetRxFIFOThreshold(SPI1,LL_SPI_RX_FIFO_TH_QUARTER);
	LL_SPI_Enable(SPI1);	//Enable SPI1
	while( LL_SPI_IsEnabled(SPI1) == 0);	//Wait To Enable SPI
	 
	LL_USART_EnableIT_RXNE(USART2);
	while( LL_USART_IsEnabled(USART2) == 0); //Wait To Enable USART
	
	/*Reset Wiznet */
	HAL_GPIO_WritePin(ETH_RESET_GPIO_Port,ETH_RESET_Pin,GPIO_PIN_RESET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(ETH_RESET_GPIO_Port,ETH_RESET_Pin,GPIO_PIN_SET);
	
	usart2_SendAnswer_DMA(strlen(waitForPHY),waitForPHY);
	while(!(getPHYCFGR() & PHYCFGR_LNK_ON));   //Wait for PHY On
	usart2_SendAnswer_DMA(strlen(PHYReady),PHYReady);
	
	WIZ_basicConfig();	//Config Socket & Mode wiznet
	
	HAL_GPIO_WritePin(MEM_WP_GPIO_Port,MEM_WP_Pin,GPIO_PIN_RESET);
	HAL_Delay(5);
	
	memWriteByte(MANUFACTURE_DADA_PROGRAM_BYTE,0xff);
//	uint16_t i =0;
//	for(i=0;i<1024;i++){
//			memWriteByte(i,0xff);
//	}
	if(memReadByte(MANUFACTURE_DADA_PROGRAM_BYTE) == 1){	//Read Last Config From Memory
		memReadArray(IP_ADDRESS_PAGE,g_WIZNetworkSetting.ipadd,4);
		memReadArray(MAC_ADDRESS_PAGE,g_WIZNetworkSetting.mac,6);
		memReadArray(SUBNET_MASK_PAGE,g_WIZNetworkSetting.subnet,4);
		memReadArray(GATEWAY_IP_ADDRESS_PAGE,g_WIZNetworkSetting.gateway,4);
		//Socket0 Read Setting
		memReadArray(SOCKET0_DESTINATION_IP_ADDRESS_PAGE,g_WIZNetworkSetting.socket0_Destipadd,4);
		g_WIZNetworkSetting.socket0_SourcePort = memReadHalfWord(SOCKET0_SOURCE_PORT_BYTE);
		g_WIZNetworkSetting.socket0_DestPort = memReadHalfWord(SOCKET0_DESTINATION_PORT_BYTE);	
		//Socket1 Read Setting
		memReadArray(SOCKET1_DESTINATION_IP_ADDRESS_PAGE,g_WIZNetworkSetting.socket1_Destipadd,4);
		g_WIZNetworkSetting.socket1_SourcePort = memReadHalfWord(SOCKET1_SOURCE_PORT_BYTE);
		g_WIZNetworkSetting.socket1_DestPort = memReadHalfWord(SOCKET1_DESTINATION_PORT_BYTE);	
		
		WIZ_networkConfig();
	}
	else{
		WIZ_defaultNetworkConfig();
	}
	
	HAL_Delay(5);	
	HAL_GPIO_WritePin(MEM_WP_GPIO_Port,MEM_WP_Pin,GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)   
	
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  WIZ_linkCheck();
	  udp_dataSize = WIZ_recvudp(g_udpData);
	  if(g_UDPdataRDY == 1){
		g_UDPdataRDY = 0;
		frameSize = Frame_check(g_udpData,udp_dataSize);
		if(g_frameCheckFlag == 1){
			g_frameCheckFlag = 0;
			usart2_SendAnswer_DMA(frameSize,g_UDP_Commbuf);
			__HAL_TIM_CLEAR_FLAG(&htim14,TIM_FLAG_UPDATE);
			HAL_TIM_Base_Start_IT(&htim14);
			while((g_usart2_DataRDY == 0) && (g_timeOutCounter < 10));
			g_timeOutCounter = 0;
			if(g_usart2_DataRDY == 1){
				WIZ_sendudp(SOCKET_UDP,g_usart2RxcommBuf,g_rxisr_frameSize);
				g_usart2_DataRDY = 0;
			}
			__HAL_TIM_CLEAR_FLAG(&htim14,TIM_FLAG_UPDATE);
			HAL_TIM_Base_Stop_IT(&htim14);
		}
	  }
	  else if(g_UDPconfigRDY == 1){
			WIZ_sendudp(SOCKET_CONFIG,config,strlen(config));
			g_UDPconfigRDY = 0;
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
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
	uint32_t isrflags   = LL_USART_ReadReg(USART2,ISR);
	uint32_t errorflags = 0x00U;
	volatile	uint8_t i=0;

	
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
			g_rxisr_frameSize=usart2RxBuf_Counter;
			memcpy(g_usart2RxcommBuf,usart2RxBuf,g_rxisr_frameSize);
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
	LL_DMA_ClearFlag_GI1(DMA1);	//clearerr Channel 2 DMA Flags
	LL_USART_ClearFlag_TC(USART2);     // Clear tc flag
	LL_USART_EnableDMAReq_TX (USART2); // ENABLE USART DMA TRanssmit Req
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);// enable channel 1	

	
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


    if((getSn_IR(SOCKET_UDP) & Sn_IR_RECV ) != 0 )
	{		
		setSn_IR(SOCKET_UDP , Sn_IR_RECV );
        ptr = getSn_RX_RD(SOCKET_UDP);
        ctrlSize = getSn_RX_RSR(SOCKET_UDP);
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
    }
	else if((getSn_IR(SOCKET_CONFIG) & Sn_IR_RECV ) != 0 ){
	
		setSn_IR(SOCKET_CONFIG , Sn_IR_RECV );
        ptr = getSn_RX_RD(SOCKET_CONFIG);
        ctrlSize = getSn_RX_RSR(SOCKET_CONFIG);
        addrsel = ((uint32_t)ptr << 8) + (WIZCHIP_RXBUF_BLOCK(SOCKET_CONFIG) << 3);
        WIZCHIP_READ_BUF(addrsel, header, header_size);
        ptr += header_size;
        datasize = (header[6] << 8);
        datasize += header[7];
		addrsel = ((uint32_t)ptr << 8) + (WIZCHIP_RXBUF_BLOCK(SOCKET_CONFIG) << 3);
		WIZCHIP_READ_BUF(addrsel, data, datasize); 
		ptr += datasize;
		setSn_RX_RD(SOCKET_CONFIG,ptr);
		setSn_CR(SOCKET_CONFIG,Sn_CR_RECV );
		g_UDPconfigRDY = 1 ;
	}

	return datasize;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void HAL_TIM14_IRQHandlerCallBack(void){
	
	g_timeOutCounter++;
	if(g_timeOutCounter >= 200)g_timeOutCounter = 200;
	__HAL_TIM_CLEAR_FLAG(&htim14,TIM_FLAG_UPDATE);
	
	
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
uint8_t Frame_check (uint8_t *data,uint8_t len){

uint8_t i=0;
uint8_t UDP_Packet_size=0;
static uint8_t UDP_Buff_Counter = 0; 
    
    for(i=0;i<len;i++){
        if(data[i] == Start_Byte){
            UDP_Buff_Counter=0;
            g_UDP_Commbuf[UDP_Buff_Counter]=data[i];
            UDP_Buff_Counter++;
        }
        else if( (UDP_Buff_Counter > 0) && (data[i] != Stop_Byte) ){
            g_UDP_Commbuf[UDP_Buff_Counter]=data[i];
            UDP_Buff_Counter++; 
        }
        else if((UDP_Buff_Counter > 0) && (data[i] == Stop_Byte)){
            g_UDP_Commbuf[UDP_Buff_Counter]=data[i];
            UDP_Buff_Counter++;
            UDP_Packet_size = UDP_Buff_Counter;
            UDP_Buff_Counter = 0;
			g_frameCheckFlag = 1;
//            switch(g_UDP_Commbuf[Command_POS]){
//                case Magic_Command:
////                        Make_Magicframe();
//                break;
//                case Locate_Commnad:
////                        Locate_device();
//                break;
//                default:
////                        g_frameCheckFlag = 1;
////                break;
//                        
//            }              
        }                    
    }
    return UDP_Packet_size;           
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void WIZ_sendudp (uint8_t sn,uint8_t *data, uint16_t len){
	
	uint16_t ptr = 0;
	uint32_t addrsel = 0;
	uint8_t ctrlSize = 0;
	uint16_t freesize = 0;

	if(len == 0)  return;
	freesize = getSn_TX_FSR(sn);
	if(freesize < len){
		g_flag = 100;
		len = freesize;	
	}
	setSn_IR(sn , Sn_IR_SENDOK );
	ptr = getSn_TX_WR(sn);	
	//M20140501 : implict type casting -> explict type casting
	//addrsel = (ptr << 8) + (WIZCHIP_TXBUF_BLOCK(sn) << 3);
	addrsel = ((uint32_t)ptr << 8) + (WIZCHIP_TXBUF_BLOCK(sn) << 3);
	//
	WIZCHIP_WRITE_BUF(addrsel,data, len);

	ptr += len;
	setSn_TX_WR(sn,ptr);
	setSn_CR(sn,Sn_CR_SEND );
	while(!(getSn_IR(sn) & (Sn_IR_SENDOK) )){
		if((getSn_IR(sn)) & (Sn_IR_TIMEOUT)){
			setSn_IR(sn,(Sn_IR_SENDOK | Sn_IR_TIMEOUT));
			break;
		}
	}
	setSn_IR(sn , Sn_IR_SENDOK );
}
/**
  * @brief System Clock Configuration
  * @retval None
  */

void WIZ_defaultNetworkConfig (void){
	
	uint32_t UID[3];
	uint32_t buff = 0;
	uint8_t	defaultmac[6];
	
	UID[0] = HAL_GetUIDw0();
	UID[1] = HAL_GetUIDw1();
	UID[2] = HAL_GetUIDw2();
	
	buff = (UID[0] ^ UID[1]);
	buff = (buff ^ UID[2]);
	buff &= (0xffffff);
	
	defaultmac[0]= STATIC_FIRST_MAC;
	defaultmac[1]= STATIC_SECOND_MAC;
	defaultmac[2]= STATIC_THIRD_MAC;
	defaultmac[3]= (buff >> 16);
	defaultmac[4]= (buff >> 8);
	defaultmac[5]= (buff);
	
	/*
	1)Gateway Address
	2)MAC Address
	3)Subnet Mak
	4)Wiznet IP Address
	5)Socke0 Destination IP Address
	6)Socke0 Source PORT
	7)Socke0 Destination Port
	8)Socke1 Destination IP Address
	9)Socke1 Source PORT
	10)Socke1 Destination Port
	*/	
	setGAR((memcpy(g_WIZNetworkSetting.gateway,defaultgateway,strlen(defaultgateway))));	
	setSHAR(memcpy(g_WIZNetworkSetting.mac,defaultmac,strlen(defaultmac)));
	setSUBR(memcpy(g_WIZNetworkSetting.subnet,defaultsubnet,strlen(defaultsubnet)));
	setSIPR(memcpy(g_WIZNetworkSetting.ipadd,defaultip,strlen(defaultip)));
	//Config Socket0 
	setSn_DIPR(SOCKET_UDP,memcpy(g_WIZNetworkSetting.socket0_Destipadd,defaultdestip,strlen(defaultdestip)));
	setSn_PORT(SOCKET_UDP,SOCKET0_DEFAULT_SOURCEPORT);
	setSn_DPORT(SOCKET_UDP,SOCKET0_DEFAULT_DESTPORT);
	//Config Socket1 
	setSn_DIPR(SOCKET_CONFIG,memcpy(g_WIZNetworkSetting.socket1_Destipadd,defaultdestip,strlen(defaultdestip)));
	setSn_PORT(SOCKET_CONFIG,SOCKET1_DEFAULT_SOURCEPORT);
	setSn_DPORT(SOCKET_CONFIG,SOCKET1_DEFAULT_DESTPORT);
	
	memWriteArray(IP_ADDRESS_PAGE,g_WIZNetworkSetting.ipadd,4);
	memWriteArray(MAC_ADDRESS_PAGE,g_WIZNetworkSetting.mac,6);
	memWriteArray(SUBNET_MASK_PAGE,g_WIZNetworkSetting.subnet,4);
	memWriteArray(GATEWAY_IP_ADDRESS_PAGE,g_WIZNetworkSetting.gateway,4);
	//Save Socket0 Info
	memWriteArray(SOCKET0_DESTINATION_IP_ADDRESS_PAGE,g_WIZNetworkSetting.socket0_Destipadd,4);
	memWriteHalfWord(SOCKET0_SOURCE_PORT_BYTE,SOCKET0_DEFAULT_SOURCEPORT);
	memWriteHalfWord(SOCKET0_DESTINATION_PORT_BYTE,SOCKET0_DEFAULT_DESTPORT);
	//Save Socket1 Info
	memWriteArray(SOCKET1_DESTINATION_IP_ADDRESS_PAGE,g_WIZNetworkSetting.socket1_Destipadd,4);
	memWriteHalfWord(SOCKET1_SOURCE_PORT_BYTE,SOCKET1_DEFAULT_SOURCEPORT);
	memWriteHalfWord(SOCKET1_DESTINATION_PORT_BYTE,SOCKET1_DEFAULT_DESTPORT);
	
	memWriteByte(MANUFACTURE_DADA_PROGRAM_BYTE,1);
	
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void WIZ_linkCheck (void){
 
	if((getPHYCFGR() & PHYCFGR_LNK_ON ) == 0){		
		usart2_SendAnswer_DMA(strlen(waitForPHY),waitForPHY);
        while ((getPHYCFGR() & PHYCFGR_LNK_ON) == 0);   //Wait for PHY On
		/*Reset Wiznet */
		HAL_GPIO_WritePin(ETH_RESET_GPIO_Port,ETH_RESET_Pin,GPIO_PIN_RESET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(ETH_RESET_GPIO_Port,ETH_RESET_Pin,GPIO_PIN_SET);
        while ((getPHYCFGR() & PHYCFGR_LNK_ON) == 0);
		usart2_SendAnswer_DMA(strlen(PHYReady),PHYReady);
		WIZ_networkConfig();
    }

}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void WIZ_basicConfig (void){
		
	setMR(0x00);	//Common Register INT set to null
	setIMR(0x00);	//Common Register INT Mask set to null
	setRTR(0x01f4);		//set Retry Time-value to 50ms
	setRCR(0x0002);		//set Retry Count to 2 
	
	setSn_RXBUF_SIZE(SOCKET_UDP,0x02);	//set Socket0 RXbuff
	setSn_TXBUF_SIZE(SOCKET_UDP,0x02);	//set Socket0 TXbuff
	setSn_RXBUF_SIZE(SOCKET_CONFIG,0x02);	//set Socket1 RXbuff
	setSn_TXBUF_SIZE(SOCKET_CONFIG,0x02);	//set Socket1 TXbuff
	
//	setSIMR(0x02);
	
	setSn_MR(SOCKET_UDP,Sn_MR_UDP);  //Set UDP Socket 0
	setSn_CR(SOCKET_UDP,Sn_CR_OPEN);	//Open Socket 0
	while((getSn_SR(SOCKET_UDP) != 0x22));	//wait to socket0 open	
	
	setSn_MR(SOCKET_CONFIG,Sn_MR_UDP);  //Set UDP Socket 1
	setSn_CR(SOCKET_CONFIG,Sn_CR_OPEN);	//Open Socket 1
	while((getSn_SR(SOCKET_CONFIG) != 0x22));	//wait to socket1 open
	
	
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void WIZ_networkConfig (void){
	//Config Wiznet Network Info
	setGAR(g_WIZNetworkSetting.gateway);	
	setSHAR(g_WIZNetworkSetting.mac);
	setSUBR(g_WIZNetworkSetting.subnet);
	setSIPR(g_WIZNetworkSetting.ipadd);
	//Config SOCKET0 Info
	setSn_DIPR(SOCKET_UDP,g_WIZNetworkSetting.socket0_Destipadd);
	setSn_PORT(SOCKET_UDP,g_WIZNetworkSetting.socket0_SourcePort);
	setSn_DPORT(SOCKET_UDP,g_WIZNetworkSetting.socket0_DestPort);
	//Config SOCKET1 Info
	setSn_DIPR(SOCKET_CONFIG,g_WIZNetworkSetting.socket1_Destipadd);
	setSn_PORT(SOCKET_CONFIG,g_WIZNetworkSetting.socket0_SourcePort);
	setSn_DPORT(SOCKET_CONFIG,g_WIZNetworkSetting.socket0_DestPort);
	
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
