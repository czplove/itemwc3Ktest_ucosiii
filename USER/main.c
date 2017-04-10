
#include "delay.h"
#include "usart.h"
#include "includes.h"

#include "tiza_io.h"
#include "test_tasks.h"
#include "tiza_misc.h"

#include "tiza_hw_timer.h"
#include "tiza_spi_flash.h"


//�������ȼ�
#define START_TASK_PRIO		3
//�����ջ��С	
#define START_STK_SIZE 		256
//������ƿ�
OS_TCB StartTaskTCB;
//�����ջ	
CPU_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *p_arg);

//�������ȼ�
#define TASK1_TASK_PRIO		4
//�����ջ��С	
#define TASK1_STK_SIZE 		256
//������ƿ�
OS_TCB Task1_TaskTCB;
//�����ջ	
CPU_STK TASK1_TASK_STK[TASK1_STK_SIZE];
void task1_task(void *p_arg);

//�������ȼ�
#define TASK2_TASK_PRIO		4
//�����ջ��С	
#define TASK2_STK_SIZE 		256
//������ƿ�
OS_TCB Task2_TaskTCB;
//�����ջ	
CPU_STK TASK2_TASK_STK[TASK2_STK_SIZE];
//������
void task2_task(void *p_arg);

//�������ȼ�
#define TASK3_TASK_PRIO		4
//�����ջ��С	
#define TASK3_STK_SIZE 		256
//������ƿ�
OS_TCB Task3_TaskTCB;
//�����ջ	
CPU_STK TASK3_TASK_STK[TASK3_STK_SIZE];
//������
void task3_task(void *p_arg);

//�������ȼ�
#define TASK4_TASK_PRIO		4
//�����ջ��С	
#define TASK4_STK_SIZE 		256
//������ƿ�
OS_TCB Task4_TaskTCB;
//�����ջ	
CPU_STK TASK4_TASK_STK[TASK4_STK_SIZE];
//������
void task4_task(void *p_arg);

//�������ȼ�
#define TASK5_TASK_PRIO		4
//�����ջ��С	
#define TASK5_STK_SIZE 		256
//������ƿ�
OS_TCB Task5_TaskTCB;
//�����ջ	
CPU_STK TASK5_TASK_STK[TASK5_STK_SIZE];
//������
void task5_task(void *p_arg);

//�������ȼ�
#define TASK6_TASK_PRIO		4
//�����ջ��С	
#define TASK6_STK_SIZE 		256
//������ƿ�
OS_TCB Task6_TaskTCB;
//�����ջ	
CPU_STK TASK6_TASK_STK[TASK6_STK_SIZE];
//������
void task6_task(void *p_arg);

//�������ȼ�
#define TASK7_TASK_PRIO		4
//�����ջ��С	
#define TASK7_STK_SIZE 		256
//������ƿ�
OS_TCB Task7_TaskTCB;
//�����ջ	
CPU_STK TASK7_TASK_STK[TASK7_STK_SIZE];
//������
void task7_task(void *p_arg);

//�������ȼ�
#define TASK8_TASK_PRIO		4
//�����ջ��С	
#define TASK8_STK_SIZE 		256
//������ƿ�
OS_TCB Task8_TaskTCB;
//�����ջ	
CPU_STK TASK8_TASK_STK[TASK8_STK_SIZE];
//������
void task8_task(void *p_arg);

//�������ȼ�
#define TASK9_TASK_PRIO		4
//�����ջ��С	
#define TASK9_STK_SIZE 		256
//������ƿ�
OS_TCB Task9_TaskTCB;
//�����ջ	
CPU_STK TASK9_TASK_STK[TASK9_STK_SIZE];
//������
void task9_task(void *p_arg);

//�������ȼ�
#define TASK10_TASK_PRIO		4
//�����ջ��С	
#define TASK10_STK_SIZE 		256
//������ƿ�
OS_TCB Task10_TaskTCB;
//�����ջ	
CPU_STK TASK10_TASK_STK[TASK10_STK_SIZE];
//������
void task10_task(void *p_arg);

//�������ȼ�
#define TASK11_TASK_PRIO		4
//�����ջ��С	
#define TASK11_STK_SIZE 		256
//������ƿ�
OS_TCB Task11_TaskTCB;
//�����ջ	
CPU_STK TASK11_TASK_STK[TASK11_STK_SIZE];
//������
void task11_task(void *p_arg);

//�������ȼ�
#define TASK12_TASK_PRIO		4
//�����ջ��С	
#define TASK12_STK_SIZE 		256
//������ƿ�
OS_TCB Task12_TaskTCB;
//�����ջ	
CPU_STK TASK12_TASK_STK[TASK12_STK_SIZE];
//������
void task12_task(void *p_arg);

//�������ȼ�
#define TASK13_TASK_PRIO		4
//�����ջ��С	
#define TASK13_STK_SIZE 		256
//������ƿ�
OS_TCB Task13_TaskTCB;
//�����ջ	
CPU_STK TASK13_TASK_STK[TASK13_STK_SIZE];
//������
void task13_task(void *p_arg);

//�������ȼ�
#define TASK14_TASK_PRIO		4
//�����ջ��С	
#define TASK14_STK_SIZE 		256
//������ƿ�
OS_TCB Task14_TaskTCB;
//�����ջ	
CPU_STK TASK14_TASK_STK[TASK14_STK_SIZE];
//������
void task14_task(void *p_arg);


extern void GsmUsartInit(u32 uart_bpr,uint8 data_bits,uint8 stop_bits,uint8 parity_check);


// read device id
extern uint8 device_id_data[];
extern uint8 device_sn_data[];

extern uint8 testdata[];

void ReadDeviceId(void)
{
	uint8 device_id_data2[5];
	uint16 i;

	FlashInit();

	FlashRead(40960, device_id_data2, 5);
	
	printf("Device id= ");
	for(i=0; i < 5; i++)
	{
		printf("%02X", device_id_data2[i]);
	}
	printf("\r\n");
	
	testdata[1] = device_id_data2[0];
	testdata[2] = device_id_data2[1];
	testdata[3] = device_id_data2[2];
	testdata[4] = device_id_data2[3];
	testdata[5] = device_id_data2[4];
}

void ReadDeviceSn(void)
{
	uint8 sn_data[13],i;

	GetDeviceSerial(sn_data, 12);
	sn_data[12] = '\0';
	printf("Device SN: %s\r\n", sn_data);
}


//������
int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	uint32 count=0;
	
	delay_init(168);  	//ʱ�ӳ�ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�жϷ�������
	#ifndef CREATE_LIB
	uart_init(115200); 	//���ڳ�ʼ��
	#endif
	IoInit();         //IO��ʼ��	

	delay_ms(100);
	printf("##### Device stated... #####\r\n");


	ReadDeviceSn();

	#ifdef UART3_BRIDDGE_UART1
	if(GenInit() != 0)
	{
		printf("GenInit failed\r\n");
    }

	GPIO_SetBits(GPIOD, GPIO_Pin_8);
	delay_ms(1000);
	GPIO_ResetBits(GPIOC, GPIO_Pin_0);
	delay_ms(1000);
	GPIO_SetBits(GPIOC, GPIO_Pin_0);
	delay_ms(3000);
	#endif

	ReadDeviceId();
	
	OSInit(&err);		//��ʼ��UCOSIII
	OS_CRITICAL_ENTER();//�����ٽ���			 
	//������ʼ����
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//������ƿ�
				 (CPU_CHAR	* )"start task", 		//��������
                 (OS_TASK_PTR )start_task, 			//������
                 (void		* )0,					//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK   * )&START_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,					//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,					//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,					//�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);				//��Ÿú�������ʱ�ķ���ֵ
	OS_CRITICAL_EXIT();	//�˳��ٽ���	 
	OSStart(&err);      //����UCOSIII
}

bool need_write_device_id = FALSE;
bool need_write_device_sn = FALSE;
void task14_task(void *p_arg)
{
	OS_ERR err;
	uint32 device_sn_addr;
	uint8 i;
	bool write_ok = TRUE;

	device_sn_addr = 0x08003FF0;
	
	while(1)
	{
		OSTimeDlyHMSM(0, 0, 1, 0, OS_OPT_TIME_HMSM_STRICT, &err);

		if(need_write_device_id)
		{
			FlashEraseSector(10);

			FlashWrite(40960, device_id_data, 5);

			need_write_device_id = FALSE;

			printf("save device id success, please reboot\r\n");
		}

		if(need_write_device_sn)
		{
			if((*((uint8*)device_sn_addr)) == 0xFF)
			{
				for(i=0; i < 12; i++)
				{
					if(FLASH_ProgramByte(device_sn_addr, device_sn_data[i]) != FLASH_COMPLETE)
					{
						write_ok = FALSE;
						break;
					}

					device_sn_addr++;
				}

				if(write_ok)
				{
					printf("save device sn success\r\n");
				}
				else
				{
					printf("save device sn failed\r\n");
				}
			}
			else
			{
				printf("please erase boot flash sector\r\n");
			}

			need_write_device_sn = FALSE;
		}
	}
}

//��ʼ������
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	
	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//ͳ������                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//���ʹ���˲����жϹر�ʱ��
    CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //��ʹ��ʱ��Ƭ��ת��ʱ��
	 //ʹ��ʱ��Ƭ��ת���ȹ���,ʱ��Ƭ����Ϊ1��ϵͳʱ�ӽ��ģ���1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif		
	
	OS_CRITICAL_ENTER();	//�����ٽ���
	
	#if 0
	//����TASK2����
	OSTaskCreate((OS_TCB 	* )&Task2_TaskTCB,		
				 (CPU_CHAR	* )"task2 task", 		
                 (OS_TASK_PTR )GpsTask, 			
                 (void		* )0,					
                 (OS_PRIO	  )TASK2_TASK_PRIO,     	
                 (CPU_STK   * )&TASK2_TASK_STK[0],	
                 (CPU_STK_SIZE)TASK2_STK_SIZE/10,	
                 (CPU_STK_SIZE)TASK2_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )3,	//3��ʱ��Ƭ����3*5=15ms					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
	#endif

	#if 1
    //����TASK3����
	OSTaskCreate((OS_TCB 	* )&Task3_TaskTCB,		
				 (CPU_CHAR	* )"task3 task", 		
                 (OS_TASK_PTR )CanTask, 			
                 (void		* )0,					
                 (OS_PRIO	  )TASK3_TASK_PRIO,     	
                 (CPU_STK   * )&TASK3_TASK_STK[0],	
                 (CPU_STK_SIZE)TASK3_STK_SIZE/10,	
                 (CPU_STK_SIZE)TASK3_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )3,	//3��ʱ��Ƭ����3*5=15ms					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);
	#endif

	#if 1
	//����TASK4����
	OSTaskCreate((OS_TCB	* )&Task4_TaskTCB,		
				 (CPU_CHAR	* )"task4 task",		
				 (OS_TASK_PTR )WatchDogTask, 			
				 (void		* )0,					
				 (OS_PRIO	  )TASK4_TASK_PRIO, 		
				 (CPU_STK	* )&TASK4_TASK_STK[0],	
				 (CPU_STK_SIZE)TASK4_STK_SIZE/10,	
				 (CPU_STK_SIZE)TASK4_STK_SIZE,		
				 (OS_MSG_QTY  )0,					
				 (OS_TICK	  )3,	//3��ʱ��Ƭ����3*5=15ms 				
				 (void		* )0,				
				 (OS_OPT	  )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
				 (OS_ERR	* )&err);
	#endif

	#if 0
	//����TASK5����
	OSTaskCreate((OS_TCB	* )&Task5_TaskTCB,		
				 (CPU_CHAR	* )"task5 task",		
				 (OS_TASK_PTR )Uart485Task,			
				 (void		* )0,					
				 (OS_PRIO	  )TASK5_TASK_PRIO, 		
				 (CPU_STK	* )&TASK5_TASK_STK[0],	
				 (CPU_STK_SIZE)TASK5_STK_SIZE/10,	
				 (CPU_STK_SIZE)TASK5_STK_SIZE,		
				 (OS_MSG_QTY  )0,					
				 (OS_TICK	  )3,	//3��ʱ��Ƭ����3*5=15ms 				
				 (void		* )0,				
				 (OS_OPT	  )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
				 (OS_ERR	* )&err);
	#endif

	#if 0
	//����TASK6����
	OSTaskCreate((OS_TCB	* )&Task6_TaskTCB,		
				 (CPU_CHAR	* )"task6 task",		
				 (OS_TASK_PTR )FlashTask,			
				 (void		* )0,					
				 (OS_PRIO	  )TASK6_TASK_PRIO, 		
				 (CPU_STK	* )&TASK6_TASK_STK[0],	
				 (CPU_STK_SIZE)TASK6_STK_SIZE/10,	
				 (CPU_STK_SIZE)TASK6_STK_SIZE,		
				 (OS_MSG_QTY  )0,					
				 (OS_TICK	  )3,	//3��ʱ��Ƭ����3*5=15ms 				
				 (void		* )0,				
				 (OS_OPT	  )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
				 (OS_ERR	* )&err);
	#endif

	#if 0
	//����TASK7����
	OSTaskCreate((OS_TCB	* )&Task7_TaskTCB,		
				 (CPU_CHAR	* )"task7 task",		
				 (OS_TASK_PTR )IoTask,			
				 (void		* )0,					
				 (OS_PRIO	  )TASK7_TASK_PRIO, 		
				 (CPU_STK	* )&TASK7_TASK_STK[0],	
				 (CPU_STK_SIZE)TASK7_STK_SIZE/10,	
				 (CPU_STK_SIZE)TASK7_STK_SIZE,		
				 (OS_MSG_QTY  )0,					
				 (OS_TICK	  )3,	//3��ʱ��Ƭ����3*5=15ms 				
				 (void		* )0,				
				 (OS_OPT	  )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
				 (OS_ERR	* )&err);
	#endif

	#if 0
	//����TASK8����
	OSTaskCreate((OS_TCB	* )&Task8_TaskTCB,		
				 (CPU_CHAR	* )"task8 task",		
				 (OS_TASK_PTR )RtcTask,			
				 (void		* )0,					
				 (OS_PRIO	  )TASK8_TASK_PRIO, 		
				 (CPU_STK	* )&TASK8_TASK_STK[0],	
				 (CPU_STK_SIZE)TASK8_STK_SIZE/10,	
				 (CPU_STK_SIZE)TASK8_STK_SIZE,		
				 (OS_MSG_QTY  )0,					
				 (OS_TICK	  )3,	//3��ʱ��Ƭ����3*5=15ms 				
				 (void		* )0,				
				 (OS_OPT	  )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
				 (OS_ERR	* )&err);
	#endif

	#if 0
	//����TASK9����
	OSTaskCreate((OS_TCB	* )&Task9_TaskTCB,		
				 (CPU_CHAR	* )"task9 task",		
				 (OS_TASK_PTR )SpiFlashTask,			
				 (void		* )0,					
				 (OS_PRIO	  )TASK9_TASK_PRIO, 		
				 (CPU_STK	* )&TASK9_TASK_STK[0],	
				 (CPU_STK_SIZE)TASK9_STK_SIZE/10,	
				 (CPU_STK_SIZE)TASK9_STK_SIZE,		
				 (OS_MSG_QTY  )0,					
				 (OS_TICK	  )3,	//3��ʱ��Ƭ����3*5=15ms 				
				 (void		* )0,				
				 (OS_OPT	  )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
				 (OS_ERR	* )&err);
	#endif

	#if 0
	//����TASK10����
	OSTaskCreate((OS_TCB	* )&Task10_TaskTCB,		
				 (CPU_CHAR	* )"task10 task",		
				 (OS_TASK_PTR )AdcTask,			
				 (void		* )0,					
				 (OS_PRIO	  )TASK10_TASK_PRIO, 		
				 (CPU_STK	* )&TASK10_TASK_STK[0],	
				 (CPU_STK_SIZE)TASK10_STK_SIZE/10,	
				 (CPU_STK_SIZE)TASK10_STK_SIZE,		
				 (OS_MSG_QTY  )0,					
				 (OS_TICK	  )3,	//3��ʱ��Ƭ����3*5=15ms 				
				 (void		* )0,				
				 (OS_OPT	  )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
				 (OS_ERR	* )&err);
	#endif

	#if 0	//gen task no block
	//����TASK11����
	OSTaskCreate((OS_TCB	* )&Task11_TaskTCB,		
				 (CPU_CHAR	* )"task11 task",		
				 (OS_TASK_PTR )GenTask,			
				 (void		* )0,					
				 (OS_PRIO	  )TASK11_TASK_PRIO, 		
				 (CPU_STK	* )&TASK11_TASK_STK[0],	
				 (CPU_STK_SIZE)TASK11_STK_SIZE/10,	
				 (CPU_STK_SIZE)TASK11_STK_SIZE,		
				 (OS_MSG_QTY  )0,					
				 (OS_TICK	  )3,	//3��ʱ��Ƭ����3*5=15ms 				
				 (void		* )0,				
				 (OS_OPT	  )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
				 (OS_ERR	* )&err);
	#endif

	#if 0
	//����TASK11����
	OSTaskCreate((OS_TCB	* )&Task12_TaskTCB,		
				 (CPU_CHAR	* )"task12 task",		
				 (OS_TASK_PTR )SdCardTask,			
				 (void		* )0,					
				 (OS_PRIO	  )TASK12_TASK_PRIO, 		
				 (CPU_STK	* )&TASK12_TASK_STK[0],	
				 (CPU_STK_SIZE)TASK12_STK_SIZE/10,	
				 (CPU_STK_SIZE)TASK12_STK_SIZE,		
				 (OS_MSG_QTY  )0,					
				 (OS_TICK	  )3,	//3��ʱ��Ƭ����3*5=15ms 				
				 (void		* )0,				
				 (OS_OPT	  )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
				 (OS_ERR	* )&err);
	#endif

	#if 1
	//����TASK11����
	OSTaskCreate((OS_TCB	* )&Task13_TaskTCB,		
				 (CPU_CHAR	* )"task13 task",		
				 (OS_TASK_PTR )HwTimerTask,			
				 (void		* )0,					
				 (OS_PRIO	  )TASK13_TASK_PRIO, 		
				 (CPU_STK	* )&TASK13_TASK_STK[0],	
				 (CPU_STK_SIZE)TASK13_STK_SIZE/10,	
				 (CPU_STK_SIZE)TASK13_STK_SIZE,		
				 (OS_MSG_QTY  )0,					
				 (OS_TICK	  )3,	//3��ʱ��Ƭ����3*5=15ms 				
				 (void		* )0,				
				 (OS_OPT	  )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
				 (OS_ERR	* )&err);
	#endif

	#if 1
	//����TASK11����
	OSTaskCreate((OS_TCB	* )&Task14_TaskTCB,		
				 (CPU_CHAR	* )"task14 task",		
				 (OS_TASK_PTR )task14_task,			
				 (void		* )0,					
				 (OS_PRIO	  )TASK14_TASK_PRIO, 		
				 (CPU_STK	* )&TASK14_TASK_STK[0],	
				 (CPU_STK_SIZE)TASK14_STK_SIZE/10,	
				 (CPU_STK_SIZE)TASK14_STK_SIZE,		
				 (OS_MSG_QTY  )0,					
				 (OS_TICK	  )3,	//3��ʱ��Ƭ����3*5=15ms 				
				 (void		* )0,				
				 (OS_OPT	  )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
				 (OS_ERR	* )&err);
	#endif
					 

	OS_CRITICAL_EXIT();	//�˳��ٽ���
	OSTaskDel((OS_TCB*)0,&err);	//ɾ��start_task��������
}

