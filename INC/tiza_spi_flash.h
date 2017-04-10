#ifndef __TIZA_SPI_FLASH_H
#define __TIZA_SPI_FLASH_H

#include "tiza_include.h"

	#ifdef SPI_FLASH_GLOBAL
		#define EXTERN_SPI_FLASH
	#else
		#define EXTERN_SPI_FLASH extern
	#endif

// 一个SECTOR大小为4096
#define FLASH_SECTOR_SIZE	4096


EXTERN_SPI_FLASH uint8 FlashInit(void);
EXTERN_SPI_FLASH uint8 FlashWrite(uint32 WriteAddr,uint8* pBuffer,uint16 NumByteToWrite);	//不会检查所写扇区是否擦除
EXTERN_SPI_FLASH uint8 FlashRead(uint32 ReadAddr, uint8* pBuffer, uint16 NumByteToRead);   //读取flash
EXTERN_SPI_FLASH uint8 FlashEraseChip(void);    	  		//整片擦除，等待时间长
EXTERN_SPI_FLASH uint8 FlashEraseSector(uint16 SectorNum);	//扇区擦除，擦除一个扇区的最少时间:150ms

EXTERN_SPI_FLASH uint8 FlashPowerDown(void);        	//进入掉电模式
EXTERN_SPI_FLASH uint8 FlashWakeUp(void);				//唤醒


#endif
