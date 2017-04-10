#define FLASH_GLOBAL

#include "tiza_flash.h"
 

#define IS_F407_FLASH_SECTOR(SECTOR) (((SECTOR) == FLASH_Sector_0)   || ((SECTOR) == FLASH_Sector_1)   ||\
                                 ((SECTOR) == FLASH_Sector_2)   || ((SECTOR) == FLASH_Sector_3)   ||\
                                 ((SECTOR) == FLASH_Sector_4)   || ((SECTOR) == FLASH_Sector_5)   ||\
                                 ((SECTOR) == FLASH_Sector_6)   || ((SECTOR) == FLASH_Sector_7)   ||\
                                 ((SECTOR) == FLASH_Sector_8)   || ((SECTOR) == FLASH_Sector_9)   ||\
                                 ((SECTOR) == FLASH_Sector_10)  || ((SECTOR) == FLASH_Sector_11))

#define IS_FORBIDDEN_FLASH_SECTOR(SECTOR) (((SECTOR) == FLASH_Sector_0)   || ((SECTOR) == FLASH_Sector_1) ||\
                                 ((SECTOR) == FLASH_Sector_5)   || ((SECTOR) == FLASH_Sector_6)   ||\
                                 ((SECTOR) == FLASH_Sector_7))

                                 

//读取指定地址的字(32位数据) 
//faddr:读地址 
//返回值:对应数据.
static uint32 STMFLASH_ReadWord(uint32 faddr)
{
	return *(uint32*)faddr; 
}  

//读取指定地址的字节(8位数据) 
//faddr:读地址 
//返回值:对应数据.

static uint8 STMFLASH_ReadByte(uint32 faddr)
{
	return *(uint8*)faddr; 
}  

//获取某个地址所在的flash扇区
//addr:flash地址
//返回值:0~11,即addr所在的扇区
static uint16 STMFLASH_GetFlashSector(uint32 addr)
{
	if(addr < ADDR_FLASH_SECTOR_1)return FLASH_Sector_0;
	else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_Sector_1;
	else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_Sector_2;
	else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_Sector_3;
	else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_Sector_4;
	else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_Sector_5;
	else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_Sector_6;
	else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_Sector_7;
	else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_Sector_8;
	else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_Sector_9;
	else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_Sector_10; 
	return FLASH_Sector_11;	
}

//从指定地址开始写入指定长度的数据
//特别注意:因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
//         写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
//         写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
//         没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写. 
//该函数对OTP区域也有效!可以用来写OTP区!
//OTP区域地址范围:0X1FFF7800~0X1FFF7A0F

// 0 : success    1 : failed
uint8 CpuFlashWrite(uint32 Addr, uint8 *Data, uint16 Len)
{ 
  	FLASH_Status status = FLASH_COMPLETE;
	uint32 addrx = 0;
	uint32 endaddr = 0;	
	bool flag = TRUE;

	if(STMFLASH_GetFlashSector(Addr) == FLASH_Sector_0 || STMFLASH_GetFlashSector(Addr + Len -1) == FLASH_Sector_0)
		return 1;
	
	if(Addr < STM32_FLASH_BASE || (Addr + Len) >= (STM32_FLASH_BASE + STM32_FLASH_SIZE))
		return 1;	//非法地址
		
	FLASH_Unlock();									//解锁 
	FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
 		
	addrx = Addr;				//写入的起始地址
	endaddr = Addr + Len;	//写入的结束地址

	if(addrx < 0X1FFF0000)			//只有主存储区,才需要执行擦除操作!!
	{
		while(addrx < endaddr)		//扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if(STMFLASH_ReadWord(addrx) != 0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
			{   
				status = FLASH_EraseSector(STMFLASH_GetFlashSector(addrx), VoltageRange_3);//VCC=2.7~3.6V之间!!
				if(status != FLASH_COMPLETE)
				{
					flag = FALSE;
					break;	//发生错误了
				}
			}
			else
			{
				addrx+=4;
			}
		} 
	}
	
	if(status == FLASH_COMPLETE)
	{
		while(Addr < endaddr)//写数据
		{
			if(FLASH_ProgramByte(Addr, *Data) != FLASH_COMPLETE)//写入数据
			{ 
				break;	//写入异常
			}
			
			Addr++;
			Data++;
		} 
	}
	
	FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁

	if(flag == TRUE)
	{
		return 0;
	}
	else
	{
		return 1;
	}
} 

// 0 : success    1 : failed
uint8 CpuFlashRead(uint32 Addr, uint8 *Data, uint16 Len)
{
	uint32 i;

	if(STMFLASH_GetFlashSector(Addr) == FLASH_Sector_0 || STMFLASH_GetFlashSector(Addr + Len -1) == FLASH_Sector_0)
		return 1;

	if(Addr < STM32_FLASH_BASE || (Addr + Len) >= (STM32_FLASH_BASE + STM32_FLASH_SIZE))
		return 1;	//非法地址
	
	for(i=0; i < Len; i++)
	{
		Data[i] = STMFLASH_ReadByte(Addr);	//读取1个字节.
		Addr++;		//偏移1个字节.	
	}

	return 0;
}

// 0 : success    1 : failed
uint8 CpuFlashEraseSector(uint16 Sector)
{
	FLASH_Status status = FLASH_COMPLETE;

	if(!IS_F407_FLASH_SECTOR(Sector))
		return 1;

	if(IS_FORBIDDEN_FLASH_SECTOR(Sector))
		return 1;

	FLASH_Unlock();									//解锁 
	FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
	
	status = FLASH_EraseSector(Sector, VoltageRange_3);//VCC=2.7~3.6V之间!!

	FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁

	if(status != FLASH_COMPLETE)
		return 1;	//发生错误了

	return 0;
}

/*
// 0 : success    1 : failed
uint8 FlashEraseAll(void)
{
	FLASH_Status status = FLASH_COMPLETE;

	FLASH_Unlock();									//解锁 
	FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间,必须禁止数据缓存
	
	status = FLASH_EraseAllSectors(VoltageRange_3);

	FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束,开启数据缓存
	FLASH_Lock();//上锁

	if(status == FLASH_COMPLETE)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
*/

