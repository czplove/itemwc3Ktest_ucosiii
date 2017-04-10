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

                                 

//��ȡָ����ַ����(32λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.
static uint32 STMFLASH_ReadWord(uint32 faddr)
{
	return *(uint32*)faddr; 
}  

//��ȡָ����ַ���ֽ�(8λ����) 
//faddr:����ַ 
//����ֵ:��Ӧ����.

static uint8 STMFLASH_ReadByte(uint32 faddr)
{
	return *(uint8*)faddr; 
}  

//��ȡĳ����ַ���ڵ�flash����
//addr:flash��ַ
//����ֵ:0~11,��addr���ڵ�����
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

//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ر�ע��:��ΪSTM32F4������ʵ��̫��,û�취���ر�����������,���Ա�����
//         д��ַ�����0XFF,��ô���Ȳ������������Ҳ�������������.����
//         д��0XFF�ĵ�ַ,�����������������ݶ�ʧ.����д֮ǰȷ��������
//         û����Ҫ����,��������������Ȳ�����,Ȼ����������д. 
//�ú�����OTP����Ҳ��Ч!��������дOTP��!
//OTP�����ַ��Χ:0X1FFF7800~0X1FFF7A0F

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
		return 1;	//�Ƿ���ַ
		
	FLASH_Unlock();									//���� 
	FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���
 		
	addrx = Addr;				//д�����ʼ��ַ
	endaddr = Addr + Len;	//д��Ľ�����ַ

	if(addrx < 0X1FFF0000)			//ֻ�����洢��,����Ҫִ�в�������!!
	{
		while(addrx < endaddr)		//ɨ��һ���ϰ�.(�Է�FFFFFFFF�ĵط�,�Ȳ���)
		{
			if(STMFLASH_ReadWord(addrx) != 0XFFFFFFFF)//�з�0XFFFFFFFF�ĵط�,Ҫ�����������
			{   
				status = FLASH_EraseSector(STMFLASH_GetFlashSector(addrx), VoltageRange_3);//VCC=2.7~3.6V֮��!!
				if(status != FLASH_COMPLETE)
				{
					flag = FALSE;
					break;	//����������
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
		while(Addr < endaddr)//д����
		{
			if(FLASH_ProgramByte(Addr, *Data) != FLASH_COMPLETE)//д������
			{ 
				break;	//д���쳣
			}
			
			Addr++;
			Data++;
		} 
	}
	
	FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
	FLASH_Lock();//����

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
		return 1;	//�Ƿ���ַ
	
	for(i=0; i < Len; i++)
	{
		Data[i] = STMFLASH_ReadByte(Addr);	//��ȡ1���ֽ�.
		Addr++;		//ƫ��1���ֽ�.	
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

	FLASH_Unlock();									//���� 
	FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���
	
	status = FLASH_EraseSector(Sector, VoltageRange_3);//VCC=2.7~3.6V֮��!!

	FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
	FLASH_Lock();//����

	if(status != FLASH_COMPLETE)
		return 1;	//����������

	return 0;
}

/*
// 0 : success    1 : failed
uint8 FlashEraseAll(void)
{
	FLASH_Status status = FLASH_COMPLETE;

	FLASH_Unlock();									//���� 
	FLASH_DataCacheCmd(DISABLE);//FLASH�����ڼ�,�����ֹ���ݻ���
	
	status = FLASH_EraseAllSectors(VoltageRange_3);

	FLASH_DataCacheCmd(ENABLE);	//FLASH��������,�������ݻ���
	FLASH_Lock();//����

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

