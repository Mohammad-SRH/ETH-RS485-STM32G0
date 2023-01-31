#ifndef MEMORY_H
#define MEMORY_H

#include "gpio.h"
#include "main.h"
#include "i2c.h"

#define MEM_DEVICE_ADD 0XA0
#define MEM_DEVICE_WRITE_ADD  0XA0
#define MEM_DEVICE_READ_ADD   0XA1

//eeprom unsigned char ee_zone1_missedPulse=2;
#define EE_ZONE1_MISSEDPULSE_BYTEADD	0
//eeprom unsigned char ee_zone2_missedPulse=2;
#define EE_ZONE2_MISSEDPULSE_BYTEADD	1

//eeprom unsigned char ee_ctrl_zone1_enable=0;
#define EE_CTRL1_ZONE1_ENABLE_BYTEADD	2

//eeprom unsigned char ee_ctrl_zone2_enable=0;
#define EE_CTRL1_ZONE2_ENABLE_BYTEADD	3


//eeprom unsigned int  ee_zone1_threshold=0;
#define EE_ZONE1_THR_HWORDADD	  4
//eeprom unsigned int  ee_zone2_threshold=0;
#define EE_ZONE2_THR_HWORDADD	  6
//eeprom unsigned int ee_shock_power=50;
#define EE_SHOCK_POWER_HWORDADD	8





uint8_t memReadByte(uint8_t byteAdd);
void memWriteByte(uint8_t byteAdd,uint8_t data);

uint16_t memReadHalfWord(uint8_t byteAdd);
void memWriteHalfWord(uint8_t halfWordAdd, uint16_t data);

void memReadPage (uint8_t pageAdd , uint8_t *retByteArray);
void memWritePage (uint8_t pageAdd , uint8_t *data);

void memWriteArray (uint8_t pageAdd , uint8_t *data , uint8_t len);
void memReadArray (uint8_t pageAdd , uint8_t *retByteArray , uint8_t len);

#endif /*MEMORY_H */
