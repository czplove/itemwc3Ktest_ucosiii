#define TEST_TASKS_GLOBAL

#include "includes.h"
#include "test_tasks.h"

#include "tiza_can.h"
#include "tiza_rs485.h"
#include "tiza_wd.h"
#include "tiza_gps.h"
#include "tiza_selfdef.h"
#include "tiza_io.h"
#include "tiza_rtc.h"
#include "tiza_flash.h"
#include "tiza_spi_flash.h"
#include "tiza_adc.h"
#include "tiza_gen.h"
#include "tiza_sd.h"
#include "tiza_hw_timer.h"
#include "tiza_power.h"

#define TEST_RTC							0
#define TEST_NSS							0
#define TEST_GREEN						0


#define TEST_TASKS_DEBUG
#define USE_HW_TIMER_CLEAR_WATCHDOG

typedef struct{
	uint32 id;
	uint8  data[8]; 
	uint8  flag;
}CAN_MSG;
#define STANDARD_CAN	1
#define EXPAND_CAN 		2
CAN_MSG Can1RxFifo;
CAN_MSG Can2RxFifo;


void HardTest(void);

void UsartLocal232DeInit(void)///Gprs�������ŷ���ʼ��
{
	GPIO_InitTypeDef GPIO_InitStructure;
///����232	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	USART_Cmd(USART1, DISABLE);  							//ʹ�ܴ���1 
	USART_ClearFlag(USART1, USART_FLAG_TC);	
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//��������ж�
	
///gps	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	USART_Cmd(UART4, DISABLE);  							//ʹ�ܴ���1 
	USART_ClearFlag(UART4, USART_FLAG_TC);	
	USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);//��������ж�
	
}


static void printhex(uint8 *data, uint8 len)
{
	uint8 i;

	for(i=0; i < len; i++)
	{
		printf("%02X", data[i]);
	}
}
/**weichai**/
void CanRecMessage(uint8 port,uint32 CanId,uint8* Dat)
{
	int i;
	
	printf("can port= %u can id = 0x%08X\r\n", port, CanId);
	printf("recv data: ");
	if(port == 1){//CAN1
		Can1RxFifo.id = 1;
		for(i=0; i < 8; i++){
			Can1RxFifo.data[i] = (uint8)*Dat;
			printf("%02X", (uint8)*Dat);
			Dat++;
		}
		printf("\r\n");
		Can1RxFifo.flag = 1;
	}
	else if(port == 2){//CAN2
		Can2RxFifo.id = 1;
		for(i=0; i < 8; i++){
			Can2RxFifo.data[i] = (uint8)*Dat;
			printf("%02X", (uint8)*Dat);
			Dat++;
		}
		printf("\r\n");
		Can2RxFifo.flag = 1;	
	}
}


 void DealDebugSend(){
	 
 }
uint8 STFIFOUsed(CAN_MSG* canmsg){
	if(canmsg->flag){
		canmsg->flag = 0;
		return 1;
	}
	else{
		return 0;
	}
}
void ReadSTFIFO(CAN_MSG* source, CAN_MSG* target){
	target = source;
}
void CanTask(void* p_arg){
  OS_ERR err;
	uint8 TestData[200];
	CAN_MSG TestCanmsg;
	
	while(1){
		OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
		
			#if 1			//TEST_CAN > 0
			printf("\r\n����CANģ��:");
			DealDebugSend();
		//	InitSTFIFO(&Can1RxFifo,CanRx1Msg,sizeof(CAN_MSG),32);
		//	InitSTFIFO(&Can2RxFifo,CanRx2Msg,sizeof(CAN_MSG),32);
			printf("\r\n------1M������,��չ֡:");
			if(CanOpenPort(1,1000)==0)
			{
				printf("\r\n----CAN�˿�1�򿪳ɹ�");
			}
			else
			{
				printf("\r\n----CAN�˿�1��ʧ��");
			}
			DealDebugSend();
			if(CanOpenPort(2,1000)==0)
			{
				printf("\r\n----CAN�˿�2�򿪳ɹ�");
			}
			else
			{
				printf("\r\n----CAN�˿�2��ʧ��");
			}
			DealDebugSend();
			TestData[0] = 0x02;
			TestData[1] = 0x10;
			TestData[2] = 0x61;
			if(CanSendMessage(1,0x18DA00F1,EXPAND_CAN,TestData,8) == 0)
			{
				printf("\r\n----CANͨ��1���ݷ��ͳɹ�,ID:0x%x",0x18DA00F1);
			}
			else
			{
				printf("\r\n----CANͨ��1���ݷ���ʧ��.");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can2RxFifo))
			{
				ReadSTFIFO(&Can2RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN2���յ�����,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN2��������ʧ��");
			}
			memset(TestData,0x06,10);
			if(CanSendMessage(2,0x18000024,EXPAND_CAN,TestData,8) == 0)
			{
				printf("\r\n----CANͨ��2���ݷ��ͳɹ�,ID:0x%x",0x18000024);
			}
			else
			{
				printf("\r\n----CANͨ��2���ݷ���ʧ��");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can1RxFifo))
			{
				ReadSTFIFO(&Can1RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN1���յ�����,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN1��������ʧ��");
			}
			printf("\r\n------1M������,��׼֡:");
			if(CanOpenPort(1,1000)==0)
			{
				printf("\r\n----CAN�˿�1�򿪳ɹ�");
			}
			else
			{
				printf("\r\n----CAN�˿�1��ʧ��");
			}
			DealDebugSend();
			if(CanOpenPort(2,1000)==0)
			{
				printf("\r\n----CAN�˿�2�򿪳ɹ�");
			}
			else
			{
				printf("\r\n----CAN�˿�2��ʧ��");
			}
			DealDebugSend();
			TestData[0] = 0x02;
			TestData[1] = 0x10;
			TestData[2] = 0x61;
			if(CanSendMessage(1,0x7e0,STANDARD_CAN,TestData,8) == 0)
			{
				printf("\r\n----CANͨ��1���ݷ��ͳɹ�,ID:0x%x",0x7e0);
			}
			else
			{
				printf("\r\n----CANͨ��1���ݷ���ʧ��.");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can2RxFifo))
			{
				ReadSTFIFO(&Can2RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN2���յ�����,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN2��������ʧ��");
			}
			memset(TestData,0x06,10);
			if(CanSendMessage(2,0x7e1,STANDARD_CAN,TestData,8) == 0)
			{
				printf("\r\n----CANͨ��2���ݷ��ͳɹ�,ID:0x%x",0x7e1);
			}
			else
			{
				printf("\r\n----CANͨ��2���ݷ���ʧ��");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can1RxFifo))
			{
				ReadSTFIFO(&Can1RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN1���յ�����,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN1��������ʧ��");
			}
			
			printf("\r\n------500K������,��չ֡:");
			if(CanOpenPort(1,500)==0)
			{
				printf("\r\n----CAN�˿�1�򿪳ɹ�");
			}
			else
			{
				printf("\r\n----CAN�˿�1��ʧ��");
			}
			DealDebugSend();
			if(CanOpenPort(2,500)==0)
			{
				printf("\r\n----CAN�˿�2�򿪳ɹ�");
			}
			else
			{
				printf("\r\n----CAN�˿�2��ʧ��");
			}
			DealDebugSend();
			TestData[0] = 0x02;
			TestData[1] = 0x10;
			TestData[2] = 0x61;
			if(CanSendMessage(1,0x18DA00F2,EXPAND_CAN,TestData,8) == 0)
			{
				printf("\r\n----CANͨ��1���ݷ��ͳɹ�,ID:0x%x",0x18DA00F2);
			}
			else
			{
				printf("\r\n----CANͨ��1���ݷ���ʧ��.");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can2RxFifo))
			{
				ReadSTFIFO(&Can2RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN2���յ�����,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\n----CAN2��������ʧ��");
			}
			memset(TestData,0x06,10);
			if(CanSendMessage(2,0x18000023,EXPAND_CAN,TestData,8) == 0)
			{
				printf("\r\n----CANͨ��2���ݷ��ͳɹ�,ID:0x%x",0x18000023);
			}
			else
			{
				printf("\r\n----CANͨ��2���ݷ���ʧ��");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can1RxFifo))
			{
				ReadSTFIFO(&Can1RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN1���յ�����,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN1��������ʧ��");
			}
			printf("\r\n------500K������,��׼֡:");
			if(CanOpenPort(1,500)==0)
			{
				printf("\r\n----CAN�˿�1�򿪳ɹ�");
			}
			else
			{
				printf("\r\n----CAN�˿�1��ʧ��");
			}
			DealDebugSend();
			if(CanOpenPort(2,500)==0)
			{
				printf("\r\n----CAN�˿�2�򿪳ɹ�");
			}
			else
			{
				printf("\r\n----CAN�˿�2��ʧ��");
			}
			DealDebugSend();
			TestData[0] = 0x02;
			TestData[1] = 0x10;
			TestData[2] = 0x61;
			if(CanSendMessage(1,0x7e1,STANDARD_CAN,TestData,8) == 0)
			{
				printf("\r\n----CANͨ��1���ݷ��ͳɹ�,ID:0x%x",0x7e1);
			}
			else
			{
				printf("\r\n----CANͨ��1���ݷ���ʧ��.");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can2RxFifo))
			{
				ReadSTFIFO(&Can2RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN2���յ�����,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN2��������ʧ��");
			}
			memset(TestData,0x06,10);
			if(CanSendMessage(2,0x7e1,STANDARD_CAN,TestData,8) == 0)
			{
				printf("\r\n----CANͨ��2���ݷ��ͳɹ�,ID:0x%x",0x7e1);
			}
			else
			{
				printf("\r\n----CANͨ��2���ݷ���ʧ��");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can1RxFifo))
			{
				ReadSTFIFO(&Can1RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN1���յ�����,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN1��������ʧ��");
			}
			
			printf("\r\n------250K������,��չ֡:");
			if(CanOpenPort(1,250)==0)
			{
				printf("\r\n----CAN�˿�1�򿪳ɹ�");
			}
			else
			{
				printf("\r\n----CAN�˿�1��ʧ��");
			}
			DealDebugSend();
			if(CanOpenPort(2,250)==0)
			{
				printf("\r\n----CAN�˿�2�򿪳ɹ�");
			}
			else
			{
				printf("\r\n----CAN�˿�2��ʧ��");
			}
			DealDebugSend();
			TestData[0] = 0x02;
			TestData[1] = 0x10;
			TestData[2] = 0x61;
			if(CanSendMessage(1,0x18DA00F3,EXPAND_CAN,TestData,8) == 0)
			{
				printf("\r\n----CANͨ��1���ݷ��ͳɹ�,ID:0x%x",0x18DA00F3);
			}
			else
			{
				printf("\r\n----CANͨ��1���ݷ���ʧ��.");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can2RxFifo))
			{
				ReadSTFIFO(&Can2RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN2���յ�����,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN2��������ʧ��");
			}
			memset(TestData,0x06,10);
			if(CanSendMessage(2,0x18000022,EXPAND_CAN,TestData,8) == 0)
			{
				printf("\r\n----CANͨ��2���ݷ��ͳɹ�,ID:0x%x",0x18000022);
			}
			else
			{
				printf("\r\n----CANͨ��2���ݷ���ʧ��");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can1RxFifo))
			{
				ReadSTFIFO(&Can1RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN1���յ�����,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN1��������ʧ��");
			}
			printf("\r\n------250K������,��׼֡:");
			if(CanOpenPort(1,250)==0)
			{
				printf("\r\n----CAN�˿�1�򿪳ɹ�");
			}
			else
			{
				printf("\r\n----CAN�˿�1��ʧ��");
			}
			DealDebugSend();
			if(CanOpenPort(2,250)==0)
			{
				printf("\r\n----CAN�˿�2�򿪳ɹ�");
			}
			else
			{
				printf("\r\n----CAN�˿�2��ʧ��");
			}
			DealDebugSend();
			TestData[0] = 0x02;
			TestData[1] = 0x10;
			TestData[2] = 0x61;
			if(CanSendMessage(1,0x7e0,STANDARD_CAN,TestData,8) == 0)
			{
				printf("\r\n----CANͨ��1���ݷ��ͳɹ�,ID:0x%x",0x7e0);
			}
			else
			{
				printf("\r\n----CANͨ��1���ݷ���ʧ��.");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can2RxFifo))
			{
				ReadSTFIFO(&Can2RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN2���յ�����,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN2��������ʧ��");
			}
			memset(TestData,0x06,10);
			if(CanSendMessage(2,0x7e1,STANDARD_CAN,TestData,8) == 0)
			{
				printf("\r\n----CANͨ��2���ݷ��ͳɹ�,ID:0x%x",0x7e1);
			}
			else
			{
				printf("\r\n----CANͨ��2���ݷ���ʧ��");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can1RxFifo))
			{
				ReadSTFIFO(&Can1RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN1���յ�����,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN1��������ʧ��");
			}
			printf("\r\nCANģ��������!\n");
			DealDebugSend();
		#endif
	}
}

#if 0
void GpsTask(void* p_arg)
{
  OS_ERR err;
	uint8 res;
	NSS_INFO TestNssInfo;
	uint8 count = 0;
	
	
	printf("\r\n����GPS����:");
 /*	DealDebugSend();*/
	if(NssInit()==0)
	{
		printf("\r\n----GPS������ʼ���ɹ�");
	}
	else
	{
		printf("\r\n----GPS������ʼ��ʧ��");
	} 
	
	OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err);
//	//PrintRtc();
	DealDebugSend();
	while(1)
	{
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
//		ClearWatchdog();
		res = GetCurrentPosition(&TestNssInfo);
		if((res == 0)&&(TestNssInfo.Status == 0))
		{
			printf("\r\n----GPS������λ��Ϣ�������.");
			printf("\r\n����:%o%o%o%o%o\n;",TestNssInfo.Longitude[0],TestNssInfo.Longitude[1],TestNssInfo.Longitude[2],TestNssInfo.Longitude[3],TestNssInfo.Longitude[4]);
			printf("\r\nγ��:%o%o%o%o\n",TestNssInfo.Latitude[0],TestNssInfo.Latitude[1],TestNssInfo.Latitude[2],TestNssInfo.Latitude[3]);
			printf("\r\n����:%o%o%o%o\n",TestNssInfo.Altitude[0],TestNssInfo.Altitude[1],TestNssInfo.Altitude[2],TestNssInfo.Altitude[3]);
			printf("\r\n�ٶ�:%o%o%o\n",TestNssInfo.Speed[0],TestNssInfo.Speed[1],TestNssInfo.Speed[2]);
			printf("\r\n����:%o%o%o\n",TestNssInfo.Direction[0],TestNssInfo.Direction[1],TestNssInfo.Direction[2]);
			printf("\r\nʱ��:%d-%d-%d %d:%d:%d\n",TestNssInfo.Time.year,TestNssInfo.Time.month,TestNssInfo.Time.day,TestNssInfo.Time.hour,TestNssInfo.Time.minute,TestNssInfo.Time.second);
			break;
		}
 	}
//	while(1){ 		
 	printf("\r\n��ǰʱ��:");
//	//PrintRtc();
	DealDebugSend();
	res = GetNssAntStatus();
	if(res == 0)
	{
		printf("\r\n----GPS���������޹���");
	}
	else if(res == 1)
	{
		printf("\r\n----GPS�������߶�·");
	}
	else if(res == 2)
	{
		printf("\r\n----GPS�������߶�·");
	}
	printf("\r\n-----�Ͽ�GPS����,����OK\n");
	DealDebugSend();
	OSTimeDlyHMSM(0,0,30,100,OS_OPT_TIME_HMSM_STRICT,&err);
//	GetCurrentPosition(&TestNssInfo);
//	//WaitInputOK();
	res = GetNssAntStatus();
	if(res == 0)
	{
		printf("\r\n----GPS���������޹���");
	}
	else if(res == 1)
	{
		printf("\r\n----GPS�������߶�·");
	}
	else if(res == 2)
	{
		printf("\r\n----GPS�������߶�·");
	}
	printf("\r\nGPS�����������!\n");
	DealDebugSend();
//	} 
}

#else
void GpsTask(void* p_arg)
{
	OS_ERR err;
	NSS_INFO nss_info;
	uint8 count = 0;

	NssInit();

	LedDisp(LED1, TRUE);
	OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err);
	LedDisp(LED1, FALSE);
   	
	while (1)
	{
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);

		if(GetCurrentPosition(&nss_info) == 0)
		{
			if(nss_info.Status == 0)
			{
				LedDisp(LED1, TRUE);
				
				printf("\r\n��λ��Ч\r\n");
				printf("PlanetNum = %u\r\n", nss_info.PlanetNum);
				
				printf("Altitude = ");
				printhex(nss_info.Altitude, 4);
				printf("\r\n");
				
				printf("Speed = ");
				printhex(nss_info.Speed, 3);
				printf("\r\n");
				
				printf("Lat =");
				printhex(nss_info.Latitude, 4);
				printf(", Long =");
				printhex(nss_info.Longitude, 5);
				printf("\r\n");
				
				printf("Direction = ");
				printhex(nss_info.Direction, 3);
				printf("\r\n");
			}
			else
			{
				LedDisp(LED1, FALSE);
				printf("��λ��Ч\r\n");
			}
			printf("GPS Time: %02u/%02u/%02u - %02u:%02u:%02u \r\n", nss_info.Time.year, nss_info.Time.month, nss_info.Time.day, nss_info.Time.hour, nss_info.Time.minute, nss_info.Time.second);
		}

		if(count % 30 == 0 && GetNssAntStatus())
		{
			printf("GPS ANT cut\r\n");
			
			count = 0;
		}

		count ++;
//		printf("gps task running\r\n");
	}
}
#endif


void RtcTask(void* p_arg)
{
	uint8  res;
	uint32 len;
	OS_ERR err;
	RTC_ST time;
	RTC_ST time_r;

/* 	if(RtcInit() == 1)
	{
		printf("RtcInit failed...\r\n");
		return;
	}

	time.year = 16;
	time.month = 8;
	time.day = 19;
	time.hour = 23;
	time.minute =59;
	time.second = 50;

	if(RtcSetTime(&time) == 1)
	{

		printf("ClockSetTime failed...\r\n");
	}
//	RtcSetAlarm(1);
	while (1)
  {
		OSTimeDlyHMSM(0, 0, 5, 0, OS_OPT_TIME_HMSM_STRICT, &err);

		if(RtcGetTime(&time_r) == 1)
		{
			printf("ClockGetTime failed...\r\n");
		}
		printf("RTC Time: %02u/%02u/%02u - %02u:%02u:%02u \r\n", time_r.year, time_r.month, time_r.day, time_r.hour, time_r.minute, time_r.second);
		//printf("RTC task is running...\r\n");
		break;
	} */
	
	
	
#if 1
		OSTimeDlyHMSM(0, 0, 5, 0, OS_OPT_TIME_HMSM_STRICT, &err);
		HardTest();
#endif
	
}



void HardTest(void)
{
	OS_ERR err;
	uint8 TestData[200];
	uint8 res,res1;
	uint16 i;
	uint32 len;
	RTC_ST TestRtcTime;
	NSS_INFO TestNssInfo;
	CAN_MSG TestCanmsg;
	SD_STATUS SdStatus;
	ip_st SerIP;
	RTC_ST rtc;
	uint16 Time;

	uint8 Gsmtestdata[] = {0x7E,0x01,0x00,0x00,0x29,0x00,0x00,0x00,0x00,0x00,0x00,0x54,0x49,0x5A,0x41,0x20,0x20,0x20,0x00,0x47,0x47,0x58,0x2D,0x48,0x31,0x30,0x00,0x54,0x49,0x5A,0x41,0x30,0x30,0x30,0x30,0x30,0x31,0x41,0x31,0x30,0x30,0x30,0x30,0x35,0x33,0x30,0x30,0x33,0x35,0x42,0x32,0x00,0x35,0x7E};
	
//	SysTickConfig(_TICK_1MS);
	
AAA:
//	WdtInit(2000);
//	InitFIFO(&GenMSerRxFifo,GenMSerRx,GEN_MSER_MAX);
//	InitFIFO(&GenDSerRxFifo,GenDSerRx,GEN_DSER_MAX);
	
//	RS232Init();
//		UsartLocal232DeInit();//���ڷ���ʼ��
//		uart_init(115200); 	//���ڳ�ʼ��
	printf("\r\n---------------��ʼ����--------------");	
	GetDeviceSerial(TestData,12);
	printf("\r\n----�豸���к�:%m\n",TestData,12);
	DealDebugSend();
	
#if TEST_RTC > 0
	printf("\r\n����RTC:");
	DealDebugSend();
	if(RtcInit()==0)
	{
		printf("\r\n----RTC��ʼ���ɹ�");
	}
	else
	{
		printf("\r\n----RTC��ʼ��ʧ��");
	}
	TestRtcTime.second = 36;
	TestRtcTime.minute = 15;
	TestRtcTime.hour = 10;
	TestRtcTime.day = 29;
	TestRtcTime.month = 10;
	TestRtcTime.year = 16;
	if(RtcSetTime(&TestRtcTime) == 0)
	{
		printf("\r\n----ʱ�����óɹ�,16��10��29��10:15:36");
	}
	else
	{
		printf("\r\n----ʱ������ʧ��,16��10��29��10:15:36");
	}
	DealDebugSend();
	if(RtcGetTime(&TestRtcTime) == 0)
	{
		printf("\r\n----ʱ���ȡ�ɹ�,ʱ��Ϊ:%d��%d��%d��%d:%d:%d",TestRtcTime.year,TestRtcTime.month,TestRtcTime.day,TestRtcTime.hour,TestRtcTime.minute,TestRtcTime.second);
	}
	else
	{
		printf("\r\n----ʱ���ȡʧ��");
	}
	printf("\r\nRTC�������!\n");
	DealDebugSend();
#endif
	
	

#if TEST_NSS > 0
	printf("\r\n����GPS����:");
	DealDebugSend();
	if(NssInit()==0)
	{
		printf("\r\n----GPS������ʼ���ɹ�");
	}
	else
	{
		printf("\r\n----GPS������ʼ��ʧ��");
	}
	//PrintRtc();
	DealDebugSend();
	OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err);
	while(1)
	{
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
		ClearWatchdog();
		res = GetCurrentPosition(&TestNssInfo);
		if((res == 0)&&(TestNssInfo.Status == 0))
		{
			printf("\r\n----GPS������λ��Ϣ�������.");
			printf("\r\n����:%o%o%o%o%o\n;",TestNssInfo.Longitude[0],TestNssInfo.Longitude[1],TestNssInfo.Longitude[2],TestNssInfo.Longitude[3],TestNssInfo.Longitude[4]);
			printf("\r\nγ��:%o%o%o%o\n",TestNssInfo.Latitude[0],TestNssInfo.Latitude[1],TestNssInfo.Latitude[2],TestNssInfo.Latitude[3]);
			printf("\r\n����:%o%o%o%o\n",TestNssInfo.Altitude[0],TestNssInfo.Altitude[1],TestNssInfo.Altitude[2],TestNssInfo.Altitude[3]);
			printf("\r\n�ٶ�:%o%o%o\n",TestNssInfo.Speed[0],TestNssInfo.Speed[1],TestNssInfo.Speed[2]);
			printf("\r\n����:%o%o%o\n",TestNssInfo.Direction[0],TestNssInfo.Direction[1],TestNssInfo.Direction[2]);
			printf("\r\nʱ��:%d-%d-%d %d:%d:%d\n",TestNssInfo.Time.year,TestNssInfo.Time.month,TestNssInfo.Time.day,TestNssInfo.Time.hour,TestNssInfo.Time.minute,TestNssInfo.Time.second);
			break;
		}
	}
	printf("\r\n��ǰʱ��:");
	//PrintRtc();
	DealDebugSend();
	res = GetNssAntStatus();
	if(res == 0)
	{
		printf("\r\n----GPS���������޹���");
	}
	else if(res == 1)
	{
		printf("\r\n----GPS�������߶�·");
	}
	else if(res == 2)
	{
		printf("\r\n----GPS�������߶�·");
	}
	printf("\r\n-----�Ͽ�GPS����,����OK\n");
	DealDebugSend();
	OSTimeDlyHMSM(0,0,10,100,OS_OPT_TIME_HMSM_STRICT,&err);
//	GetCurrentPosition(&TestNssInfo);
	//WaitInputOK();
	res = GetNssAntStatus();
	if(res == 0)
	{
		printf("\r\n----GPS���������޹���");
	}
	else if(res == 1)
	{
		printf("\r\n----GPS�������߶�·");
	}
	else if(res == 2)
	{
		printf("\r\n----GPS�������߶�·");
	}
	printf("\r\nGPS�����������!\n");
	DealDebugSend();
#endif

#if TEST_GREEN > 0
	printf("\r\n����ʡ��ģʽ");
	res = GetExtPowerStatus();
	if(res == 0)
	{
		printf("\r\n---�ⲿ��Դû��\n");
	}
	else
	{
		printf("\r\n---�ⲿ��Դ�е�\n");
	}
	printf("\r\n---�Ͽ��ⲿ��Դ,��OK\n");
	DealDebugSend();
	//WaitInputOK();
	res = GetExtPowerStatus();
	if(res == 0)
	{
		printf("\r\n---�ⲿ��Դû��\n");
	}
	else
	{
		printf("\r\n---�ⲿ��Դ�е�\n");
	}
	DealDebugSend();
	printf("\r\n---�Ͽ������Դ.\n");
	ClearWatchdog();
	
	ClearWatchdog();
	len = 0;
	while(1)
	{
		OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
		printf("\r\n---��������");
		OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
		DealDebugSend();
		ClearWatchdog();
		RtcSetAlarm(1);
		SystemPowerDown();
		CpuPowerDown();
		
		ClearWatchdog();
		OSTimeDlyHMSM(0, 0, 3, 100, OS_OPT_TIME_HMSM_STRICT, &err);
//		IoInit();
//		RS232Init();			//�ڶ��γ�ʼ��֮ǰҪ�ȷ���ʼ��
		//RtcInit();
		//PrintRtc();
		printf("\r\n---���߻��� len=%d",len);
		DealDebugSend();
		ClearWatchdog();
		if(len > 30)
		{
			SystemWakeup();
			goto AAA;
		}
		len ++;
		
	}
	
#endif



	printf("\r\n---------------���Խ���--------------\n");
	DealDebugSend();
	while(1);
}





























//uint8 testdata[] = {0x26, 0x80, 0x61, 0x71, 0x66, 0x26, 0xaa, 0x00, 0x02, 0x55, 0xaa, 0x62};
uint8 testdata[] = {0x26, 0x80, 0x61, 0x71, 0x66, 0x16, 0xaa, 0x00, 0x02, 0x55, 0xaa, 0x62};

uint8 testdata2[1500];
// Ϋ���Լ�ʵ��
uint8 GenRecData(uint8 IPnum, uint8 *data, uint16 len)
{
	uint16 i;
	
	printf("gsm recv data: ");
	for(i=0; i < len; i++)
	{
		printf("%02X ", data[i]);
	}

	printf("\r\n");
	
    return 0;
}
void RS485RecData(uint8 *data, uint16 len)
{
	#ifdef TEST_TASKS_DEBUG
	printf("%02X", data[0]);
	#endif
}
void WatchDogTask(void* p_arg)
{
    OS_ERR err;

	#ifdef USE_HW_TIMER_CLEAR_WATCHDOG
	return;
	#endif
	
	//�ⲿӲ�����Ź�����
	//EWdtInit();

	//�ڲ��������Ź�����
	WdtInit(2000);	// 2��
	
	while (1)
    {
		//�ⲿӲ�����Ź�����
		//ClearEWatchdog();
		
		#ifdef TEST_TASKS_DEBUG
		//printf("hw watch dog feed\r\n");
		#endif
		
		// ����1.6�벻��Ӳ�����Ź���ϵͳ��λ, ����1�룬����ݲ�������
		//OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);

		//�ڲ��������Ź�����
		ClearWatchdog();
		
		#ifdef TEST_TASKS_DEBUG
		//printf("independent watch dog feed\r\n");
		#endif

		OSTimeDlyHMSM(0, 0, 0, 500, OS_OPT_TIME_HMSM_STRICT, &err);

		//printf("Watch dog feed\r\n");
	}
}

// test code end
void TimerFunc(void)
{
	OS_ERR err;
	//static bool flag = FALSE;

	#ifdef USE_HW_TIMER_CLEAR_WATCHDOG
	//�ⲿӲ�����Ź�����
	//ClearEWatchdog();

	//�ڲ��������Ź�����
	ClearWatchdog();

	//printf("feed watchdog\r\n");
/*
	if(flag)
	{
		LedDisp(LED2, TRUE);
	}
	else
	{
		LedDisp(LED2, FALSE);
	}
	flag = !flag;
	*/
	#else
	printf("hw timer called =%u ...\r\n", OSTimeGet(&err));
	#endif
}
void HwTimerTask(void* p_arg)
{
	OS_ERR err;

	#ifdef USE_HW_TIMER_CLEAR_WATCHDOG
	//�ⲿӲ�����Ź�����
	//EWdtInit();

	//�ڲ��������Ź�����
	WdtInit(2000);	// 2��

	OpenTimer(500);

	return; 
	#endif

	while (1)
    {
		OSTimeDlyHMSM(0, 0, 5, 0, OS_OPT_TIME_HMSM_STRICT, &err);

		printf("hw timer started...\r\n");
		
		OpenTimer(100);
		
		OSTimeDlyHMSM(0, 0, 5, 0, OS_OPT_TIME_HMSM_STRICT, &err);

		CloseTimer();
		printf("hw timer stopped...\r\n");
		
		//printf("hw timer task is running...\r\n");
	}
}



























