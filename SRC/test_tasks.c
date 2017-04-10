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

void UsartLocal232DeInit(void)///Gprs串口引脚反初始化
{
	GPIO_InitTypeDef GPIO_InitStructure;
///本地232	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	USART_Cmd(USART1, DISABLE);  							//使能串口1 
	USART_ClearFlag(USART1, USART_FLAG_TC);	
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//开启相关中断
	
///gps	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	USART_Cmd(UART4, DISABLE);  							//使能串口1 
	USART_ClearFlag(UART4, USART_FLAG_TC);	
	USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);//开启相关中断
	
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
			printf("\r\n测试CAN模块:");
			DealDebugSend();
		//	InitSTFIFO(&Can1RxFifo,CanRx1Msg,sizeof(CAN_MSG),32);
		//	InitSTFIFO(&Can2RxFifo,CanRx2Msg,sizeof(CAN_MSG),32);
			printf("\r\n------1M波特率,扩展帧:");
			if(CanOpenPort(1,1000)==0)
			{
				printf("\r\n----CAN端口1打开成功");
			}
			else
			{
				printf("\r\n----CAN端口1打开失败");
			}
			DealDebugSend();
			if(CanOpenPort(2,1000)==0)
			{
				printf("\r\n----CAN端口2打开成功");
			}
			else
			{
				printf("\r\n----CAN端口2打开失败");
			}
			DealDebugSend();
			TestData[0] = 0x02;
			TestData[1] = 0x10;
			TestData[2] = 0x61;
			if(CanSendMessage(1,0x18DA00F1,EXPAND_CAN,TestData,8) == 0)
			{
				printf("\r\n----CAN通道1数据发送成功,ID:0x%x",0x18DA00F1);
			}
			else
			{
				printf("\r\n----CAN通道1数据发送失败.");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can2RxFifo))
			{
				ReadSTFIFO(&Can2RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN2接收到数据,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN2接收数据失败");
			}
			memset(TestData,0x06,10);
			if(CanSendMessage(2,0x18000024,EXPAND_CAN,TestData,8) == 0)
			{
				printf("\r\n----CAN通道2数据发送成功,ID:0x%x",0x18000024);
			}
			else
			{
				printf("\r\n----CAN通道2数据发送失败");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can1RxFifo))
			{
				ReadSTFIFO(&Can1RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN1接收到数据,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN1接收数据失败");
			}
			printf("\r\n------1M波特率,标准帧:");
			if(CanOpenPort(1,1000)==0)
			{
				printf("\r\n----CAN端口1打开成功");
			}
			else
			{
				printf("\r\n----CAN端口1打开失败");
			}
			DealDebugSend();
			if(CanOpenPort(2,1000)==0)
			{
				printf("\r\n----CAN端口2打开成功");
			}
			else
			{
				printf("\r\n----CAN端口2打开失败");
			}
			DealDebugSend();
			TestData[0] = 0x02;
			TestData[1] = 0x10;
			TestData[2] = 0x61;
			if(CanSendMessage(1,0x7e0,STANDARD_CAN,TestData,8) == 0)
			{
				printf("\r\n----CAN通道1数据发送成功,ID:0x%x",0x7e0);
			}
			else
			{
				printf("\r\n----CAN通道1数据发送失败.");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can2RxFifo))
			{
				ReadSTFIFO(&Can2RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN2接收到数据,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN2接收数据失败");
			}
			memset(TestData,0x06,10);
			if(CanSendMessage(2,0x7e1,STANDARD_CAN,TestData,8) == 0)
			{
				printf("\r\n----CAN通道2数据发送成功,ID:0x%x",0x7e1);
			}
			else
			{
				printf("\r\n----CAN通道2数据发送失败");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can1RxFifo))
			{
				ReadSTFIFO(&Can1RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN1接收到数据,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN1接收数据失败");
			}
			
			printf("\r\n------500K波特率,扩展帧:");
			if(CanOpenPort(1,500)==0)
			{
				printf("\r\n----CAN端口1打开成功");
			}
			else
			{
				printf("\r\n----CAN端口1打开失败");
			}
			DealDebugSend();
			if(CanOpenPort(2,500)==0)
			{
				printf("\r\n----CAN端口2打开成功");
			}
			else
			{
				printf("\r\n----CAN端口2打开失败");
			}
			DealDebugSend();
			TestData[0] = 0x02;
			TestData[1] = 0x10;
			TestData[2] = 0x61;
			if(CanSendMessage(1,0x18DA00F2,EXPAND_CAN,TestData,8) == 0)
			{
				printf("\r\n----CAN通道1数据发送成功,ID:0x%x",0x18DA00F2);
			}
			else
			{
				printf("\r\n----CAN通道1数据发送失败.");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can2RxFifo))
			{
				ReadSTFIFO(&Can2RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN2接收到数据,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\n----CAN2接收数据失败");
			}
			memset(TestData,0x06,10);
			if(CanSendMessage(2,0x18000023,EXPAND_CAN,TestData,8) == 0)
			{
				printf("\r\n----CAN通道2数据发送成功,ID:0x%x",0x18000023);
			}
			else
			{
				printf("\r\n----CAN通道2数据发送失败");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can1RxFifo))
			{
				ReadSTFIFO(&Can1RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN1接收到数据,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN1接收数据失败");
			}
			printf("\r\n------500K波特率,标准帧:");
			if(CanOpenPort(1,500)==0)
			{
				printf("\r\n----CAN端口1打开成功");
			}
			else
			{
				printf("\r\n----CAN端口1打开失败");
			}
			DealDebugSend();
			if(CanOpenPort(2,500)==0)
			{
				printf("\r\n----CAN端口2打开成功");
			}
			else
			{
				printf("\r\n----CAN端口2打开失败");
			}
			DealDebugSend();
			TestData[0] = 0x02;
			TestData[1] = 0x10;
			TestData[2] = 0x61;
			if(CanSendMessage(1,0x7e1,STANDARD_CAN,TestData,8) == 0)
			{
				printf("\r\n----CAN通道1数据发送成功,ID:0x%x",0x7e1);
			}
			else
			{
				printf("\r\n----CAN通道1数据发送失败.");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can2RxFifo))
			{
				ReadSTFIFO(&Can2RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN2接收到数据,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN2接收数据失败");
			}
			memset(TestData,0x06,10);
			if(CanSendMessage(2,0x7e1,STANDARD_CAN,TestData,8) == 0)
			{
				printf("\r\n----CAN通道2数据发送成功,ID:0x%x",0x7e1);
			}
			else
			{
				printf("\r\n----CAN通道2数据发送失败");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can1RxFifo))
			{
				ReadSTFIFO(&Can1RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN1接收到数据,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN1接收数据失败");
			}
			
			printf("\r\n------250K波特率,扩展帧:");
			if(CanOpenPort(1,250)==0)
			{
				printf("\r\n----CAN端口1打开成功");
			}
			else
			{
				printf("\r\n----CAN端口1打开失败");
			}
			DealDebugSend();
			if(CanOpenPort(2,250)==0)
			{
				printf("\r\n----CAN端口2打开成功");
			}
			else
			{
				printf("\r\n----CAN端口2打开失败");
			}
			DealDebugSend();
			TestData[0] = 0x02;
			TestData[1] = 0x10;
			TestData[2] = 0x61;
			if(CanSendMessage(1,0x18DA00F3,EXPAND_CAN,TestData,8) == 0)
			{
				printf("\r\n----CAN通道1数据发送成功,ID:0x%x",0x18DA00F3);
			}
			else
			{
				printf("\r\n----CAN通道1数据发送失败.");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can2RxFifo))
			{
				ReadSTFIFO(&Can2RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN2接收到数据,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN2接收数据失败");
			}
			memset(TestData,0x06,10);
			if(CanSendMessage(2,0x18000022,EXPAND_CAN,TestData,8) == 0)
			{
				printf("\r\n----CAN通道2数据发送成功,ID:0x%x",0x18000022);
			}
			else
			{
				printf("\r\n----CAN通道2数据发送失败");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can1RxFifo))
			{
				ReadSTFIFO(&Can1RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN1接收到数据,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN1接收数据失败");
			}
			printf("\r\n------250K波特率,标准帧:");
			if(CanOpenPort(1,250)==0)
			{
				printf("\r\n----CAN端口1打开成功");
			}
			else
			{
				printf("\r\n----CAN端口1打开失败");
			}
			DealDebugSend();
			if(CanOpenPort(2,250)==0)
			{
				printf("\r\n----CAN端口2打开成功");
			}
			else
			{
				printf("\r\n----CAN端口2打开失败");
			}
			DealDebugSend();
			TestData[0] = 0x02;
			TestData[1] = 0x10;
			TestData[2] = 0x61;
			if(CanSendMessage(1,0x7e0,STANDARD_CAN,TestData,8) == 0)
			{
				printf("\r\n----CAN通道1数据发送成功,ID:0x%x",0x7e0);
			}
			else
			{
				printf("\r\n----CAN通道1数据发送失败.");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can2RxFifo))
			{
				ReadSTFIFO(&Can2RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN2接收到数据,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN2接收数据失败");
			}
			memset(TestData,0x06,10);
			if(CanSendMessage(2,0x7e1,STANDARD_CAN,TestData,8) == 0)
			{
				printf("\r\n----CAN通道2数据发送成功,ID:0x%x",0x7e1);
			}
			else
			{
				printf("\r\n----CAN通道2数据发送失败");
			}
			DealDebugSend();
			if(STFIFOUsed(&Can1RxFifo))
			{
				ReadSTFIFO(&Can1RxFifo,(void*)&TestCanmsg);
				printf("\r\n----CAN1接收到数据,ID:0x%x",TestCanmsg.id);
			}
			else
			{
				printf("\r\n----CAN1接收数据失败");
			}
			printf("\r\nCAN模块测试完成!\n");
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
	
	
	printf("\r\n测试GPS北斗:");
 /*	DealDebugSend();*/
	if(NssInit()==0)
	{
		printf("\r\n----GPS北斗初始化成功");
	}
	else
	{
		printf("\r\n----GPS北斗初始化失败");
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
			printf("\r\n----GPS北斗定位信息更新完成.");
			printf("\r\n经度:%o%o%o%o%o\n;",TestNssInfo.Longitude[0],TestNssInfo.Longitude[1],TestNssInfo.Longitude[2],TestNssInfo.Longitude[3],TestNssInfo.Longitude[4]);
			printf("\r\n纬度:%o%o%o%o\n",TestNssInfo.Latitude[0],TestNssInfo.Latitude[1],TestNssInfo.Latitude[2],TestNssInfo.Latitude[3]);
			printf("\r\n海拔:%o%o%o%o\n",TestNssInfo.Altitude[0],TestNssInfo.Altitude[1],TestNssInfo.Altitude[2],TestNssInfo.Altitude[3]);
			printf("\r\n速度:%o%o%o\n",TestNssInfo.Speed[0],TestNssInfo.Speed[1],TestNssInfo.Speed[2]);
			printf("\r\n方向:%o%o%o\n",TestNssInfo.Direction[0],TestNssInfo.Direction[1],TestNssInfo.Direction[2]);
			printf("\r\n时间:%d-%d-%d %d:%d:%d\n",TestNssInfo.Time.year,TestNssInfo.Time.month,TestNssInfo.Time.day,TestNssInfo.Time.hour,TestNssInfo.Time.minute,TestNssInfo.Time.second);
			break;
		}
 	}
//	while(1){ 		
 	printf("\r\n当前时间:");
//	//PrintRtc();
	DealDebugSend();
	res = GetNssAntStatus();
	if(res == 0)
	{
		printf("\r\n----GPS北斗天线无故障");
	}
	else if(res == 1)
	{
		printf("\r\n----GPS北斗天线断路");
	}
	else if(res == 2)
	{
		printf("\r\n----GPS北斗天线短路");
	}
	printf("\r\n-----断开GPS天线,输入OK\n");
	DealDebugSend();
	OSTimeDlyHMSM(0,0,30,100,OS_OPT_TIME_HMSM_STRICT,&err);
//	GetCurrentPosition(&TestNssInfo);
//	//WaitInputOK();
	res = GetNssAntStatus();
	if(res == 0)
	{
		printf("\r\n----GPS北斗天线无故障");
	}
	else if(res == 1)
	{
		printf("\r\n----GPS北斗天线断路");
	}
	else if(res == 2)
	{
		printf("\r\n----GPS北斗天线短路");
	}
	printf("\r\nGPS北斗测试完成!\n");
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
				
				printf("\r\n定位有效\r\n");
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
				printf("定位无效\r\n");
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
//		UsartLocal232DeInit();//串口反初始化
//		uart_init(115200); 	//串口初始化
	printf("\r\n---------------开始测试--------------");	
	GetDeviceSerial(TestData,12);
	printf("\r\n----设备序列号:%m\n",TestData,12);
	DealDebugSend();
	
#if TEST_RTC > 0
	printf("\r\n测试RTC:");
	DealDebugSend();
	if(RtcInit()==0)
	{
		printf("\r\n----RTC初始化成功");
	}
	else
	{
		printf("\r\n----RTC初始化失败");
	}
	TestRtcTime.second = 36;
	TestRtcTime.minute = 15;
	TestRtcTime.hour = 10;
	TestRtcTime.day = 29;
	TestRtcTime.month = 10;
	TestRtcTime.year = 16;
	if(RtcSetTime(&TestRtcTime) == 0)
	{
		printf("\r\n----时间设置成功,16年10月29号10:15:36");
	}
	else
	{
		printf("\r\n----时间设置失败,16年10月29号10:15:36");
	}
	DealDebugSend();
	if(RtcGetTime(&TestRtcTime) == 0)
	{
		printf("\r\n----时间读取成功,时间为:%d年%d月%d日%d:%d:%d",TestRtcTime.year,TestRtcTime.month,TestRtcTime.day,TestRtcTime.hour,TestRtcTime.minute,TestRtcTime.second);
	}
	else
	{
		printf("\r\n----时间读取失败");
	}
	printf("\r\nRTC测试完成!\n");
	DealDebugSend();
#endif
	
	

#if TEST_NSS > 0
	printf("\r\n测试GPS北斗:");
	DealDebugSend();
	if(NssInit()==0)
	{
		printf("\r\n----GPS北斗初始化成功");
	}
	else
	{
		printf("\r\n----GPS北斗初始化失败");
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
			printf("\r\n----GPS北斗定位信息更新完成.");
			printf("\r\n经度:%o%o%o%o%o\n;",TestNssInfo.Longitude[0],TestNssInfo.Longitude[1],TestNssInfo.Longitude[2],TestNssInfo.Longitude[3],TestNssInfo.Longitude[4]);
			printf("\r\n纬度:%o%o%o%o\n",TestNssInfo.Latitude[0],TestNssInfo.Latitude[1],TestNssInfo.Latitude[2],TestNssInfo.Latitude[3]);
			printf("\r\n海拔:%o%o%o%o\n",TestNssInfo.Altitude[0],TestNssInfo.Altitude[1],TestNssInfo.Altitude[2],TestNssInfo.Altitude[3]);
			printf("\r\n速度:%o%o%o\n",TestNssInfo.Speed[0],TestNssInfo.Speed[1],TestNssInfo.Speed[2]);
			printf("\r\n方向:%o%o%o\n",TestNssInfo.Direction[0],TestNssInfo.Direction[1],TestNssInfo.Direction[2]);
			printf("\r\n时间:%d-%d-%d %d:%d:%d\n",TestNssInfo.Time.year,TestNssInfo.Time.month,TestNssInfo.Time.day,TestNssInfo.Time.hour,TestNssInfo.Time.minute,TestNssInfo.Time.second);
			break;
		}
	}
	printf("\r\n当前时间:");
	//PrintRtc();
	DealDebugSend();
	res = GetNssAntStatus();
	if(res == 0)
	{
		printf("\r\n----GPS北斗天线无故障");
	}
	else if(res == 1)
	{
		printf("\r\n----GPS北斗天线断路");
	}
	else if(res == 2)
	{
		printf("\r\n----GPS北斗天线短路");
	}
	printf("\r\n-----断开GPS天线,输入OK\n");
	DealDebugSend();
	OSTimeDlyHMSM(0,0,10,100,OS_OPT_TIME_HMSM_STRICT,&err);
//	GetCurrentPosition(&TestNssInfo);
	//WaitInputOK();
	res = GetNssAntStatus();
	if(res == 0)
	{
		printf("\r\n----GPS北斗天线无故障");
	}
	else if(res == 1)
	{
		printf("\r\n----GPS北斗天线断路");
	}
	else if(res == 2)
	{
		printf("\r\n----GPS北斗天线短路");
	}
	printf("\r\nGPS北斗测试完成!\n");
	DealDebugSend();
#endif

#if TEST_GREEN > 0
	printf("\r\n测试省电模式");
	res = GetExtPowerStatus();
	if(res == 0)
	{
		printf("\r\n---外部电源没电\n");
	}
	else
	{
		printf("\r\n---外部电源有电\n");
	}
	printf("\r\n---断开外部电源,按OK\n");
	DealDebugSend();
	//WaitInputOK();
	res = GetExtPowerStatus();
	if(res == 0)
	{
		printf("\r\n---外部电源没电\n");
	}
	else
	{
		printf("\r\n---外部电源有电\n");
	}
	DealDebugSend();
	printf("\r\n---断开外设电源.\n");
	ClearWatchdog();
	
	ClearWatchdog();
	len = 0;
	while(1)
	{
		OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
		printf("\r\n---进入休眠");
		OSTimeDlyHMSM(0, 0, 0, 100, OS_OPT_TIME_HMSM_STRICT, &err);
		DealDebugSend();
		ClearWatchdog();
		RtcSetAlarm(1);
		SystemPowerDown();
		CpuPowerDown();
		
		ClearWatchdog();
		OSTimeDlyHMSM(0, 0, 3, 100, OS_OPT_TIME_HMSM_STRICT, &err);
//		IoInit();
//		RS232Init();			//第二次初始化之前要先反初始化
		//RtcInit();
		//PrintRtc();
		printf("\r\n---休眠唤醒 len=%d",len);
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



	printf("\r\n---------------测试结束--------------\n");
	DealDebugSend();
	while(1);
}





























//uint8 testdata[] = {0x26, 0x80, 0x61, 0x71, 0x66, 0x26, 0xaa, 0x00, 0x02, 0x55, 0xaa, 0x62};
uint8 testdata[] = {0x26, 0x80, 0x61, 0x71, 0x66, 0x16, 0xaa, 0x00, 0x02, 0x55, 0xaa, 0x62};

uint8 testdata2[1500];
// 潍柴自己实现
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
	
	//外部硬件看门狗测试
	//EWdtInit();

	//内部独立看门狗测试
	WdtInit(2000);	// 2秒
	
	while (1)
    {
		//外部硬件看门狗测试
		//ClearEWatchdog();
		
		#ifdef TEST_TASKS_DEBUG
		//printf("hw watch dog feed\r\n");
		#endif
		
		// 超过1.6秒不清硬件看门狗，系统复位, 建议1秒，或根据测试来定
		//OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);

		//内部独立看门狗测试
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
	//外部硬件看门狗测试
	//ClearEWatchdog();

	//内部独立看门狗测试
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
	//外部硬件看门狗测试
	//EWdtInit();

	//内部独立看门狗测试
	WdtInit(2000);	// 2秒

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



























