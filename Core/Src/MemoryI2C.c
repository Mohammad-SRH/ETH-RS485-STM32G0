
#include "MemoryI2C.h"
#include "string.h"


//void memReadByte(uint8_t byteAdd,uint8_t *dataBuf ){
//   	HAL_I2C_Mem_Read(&hi2c1,MEM_DEVICE_ADD,byteAdd,I2C_MEMADD_SIZE_8BIT,dataBuf,8,1);
//}

//void memWriteByte(uint8_t byteAdd,uint8_t *dataBuf){
//		HAL_I2C_Mem_Write(&hi2c1,MEM_DEVICE_ADD,byteAdd,I2C_MEMADD_SIZE_8BIT,dataBuf,8,1);
//}

uint8_t memReadByte(uint8_t byteAdd){
	
	uint8_t retVal;
	HAL_I2C_Mem_Read(&hi2c1,MEM_DEVICE_ADD,byteAdd,I2C_MEMADD_SIZE_8BIT,&retVal,1,1);

	return retVal;
}
void memWriteByte(uint8_t byteAdd,uint8_t data){
	
	HAL_I2C_Mem_Write(&hi2c1,MEM_DEVICE_ADD,byteAdd,I2C_MEMADD_SIZE_8BIT,&data,1,6);
	HAL_Delay(5);

}

uint16_t memReadHalfWord(uint8_t byteAdd){
	uint8_t retByteArray[2];
	uint16_t retValHalfWord;

	HAL_I2C_Mem_Read(&hi2c1,MEM_DEVICE_ADD,byteAdd,I2C_MEMADD_SIZE_8BIT,retByteArray,2,2);
	retValHalfWord = retByteArray[1];
	retValHalfWord <<= 8;
	retValHalfWord |= retByteArray[0];
	return retValHalfWord;
}
void memWriteHalfWord(uint8_t halfWordAdd, uint16_t data){
	uint8_t retByteArray[2];
	retByteArray[0] = data; // lowBYte
	retByteArray[1] = data>>8;
	HAL_I2C_Mem_Write(&hi2c1,MEM_DEVICE_ADD,halfWordAdd,I2C_MEMADD_SIZE_8BIT,retByteArray,2,6);
	HAL_Delay(5);
}


void memReadPage (uint8_t pageAdd , uint8_t *retByteArray){
	
	uint8_t Address = 0;
	uint8_t dataSize = 0;
	dataSize = (sizeof(retByteArray));
	Address = (pageAdd * 8);
	
	HAL_I2C_Mem_Read(&hi2c1,MEM_DEVICE_ADD,Address,I2C_MEMADD_SIZE_8BIT,retByteArray,dataSize,2);

}

void memWritePage (uint8_t pageAdd , uint8_t *data){
	
	uint8_t Address = 0;
	uint8_t dataSize = 0;
	dataSize = (sizeof(*data));
	Address = (pageAdd * 8);
	HAL_I2C_Mem_Write(&hi2c1,MEM_DEVICE_ADD,Address,I2C_MEMADD_SIZE_8BIT,data,dataSize,6);
	HAL_Delay(5);
	
}

void memReadArray (uint8_t pageAdd , uint8_t *retByteArray , uint8_t NumberofByte){
	
	uint8_t Address = 0;
	Address = (pageAdd * 8);
	HAL_I2C_Mem_Read(&hi2c1,MEM_DEVICE_ADD,Address,I2C_MEMADD_SIZE_8BIT,retByteArray,NumberofByte,6);	
}

void memWriteArray (uint8_t pageAdd , uint8_t *data , uint8_t NumberofByte){
	
	uint8_t Address = 0;
	Address = (pageAdd * 8);
	HAL_I2C_Mem_Write(&hi2c1,MEM_DEVICE_ADD,Address,I2C_MEMADD_SIZE_8BIT,data,NumberofByte,6);
	
}


