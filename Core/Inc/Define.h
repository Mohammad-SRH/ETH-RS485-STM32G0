


#define RX_BUFFER_SIZE      64 
#define UDP_BUFFER_SIZE     64
//---------------------------------------------------------//

#define Start_Byte          0x02
#define Stop_Byte           0x03
#define Magic_Command       0x22
#define Locate_Commnad      0x23
#define Command_POS         2
                                       

//---------------------------------------------------------//
#define Wiz_deSelect        HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port,SPI1_NSS_Pin,GPIO_PIN_SET)
#define Wiz_Select          HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port,SPI1_NSS_Pin,GPIO_PIN_RESET)


//#define OUT1_POS             1
//#define OUT1_DIR_PORT      PORTC
//#define OUT1_DIR_DDR       DDRC
//#define OUT1_ON            OUT1_DIR_PORT  |= (1 << OUT1_POS)
//#define OUT1_OFF           OUT1_DIR_PORT &= ~(1 << OUT1_POS)
//#define OUT1_TOGGLE        OUT1_DIR_PORT ^= ~(1 << OUT1_POS)

//#define OUT2_POS             0
//#define OUT2_DIR_PORT      PORTC
//#define OUT2_DIR_DDR       DDRC
//#define OUT2_ON            OUT2_DIR_PORT  |= (1 << OUT2_POS)
//#define OUT2_OFF           OUT2_DIR_PORT &= ~(1 << OUT2_POS)
//#define OUT2_TOGGLE        OUT2_DIR_PORT ^= ~(1 << OUT2_POS) 

                         
 

#define SOCKET_UDP          0
#define SOCKET_TCP          1                          


//#define Random_upper_Num    255
//#define Random_lower_Num    0                

//#define PACK_FIRST          0x80
//#define PACK_COMPLETED      0x00                               
//#define PACK_REMAINED       0x01