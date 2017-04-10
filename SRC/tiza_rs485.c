#define RS485_GLOBAL

#include "tiza_rs485.h"
#include "tiza_selfdef.h"

//#define UART_485_DEBUG

#if 0
#define RS485_MAX_BUF_LEN 				512

typedef struct
{
	uint8 rx_done_flag;
	uint8 rx_buf[RS485_MAX_BUF_LEN];
	uint16 rx_counter;
	uint16 rx_timeout_decounter;
}RS485_BUF_STRUCT;
#endif

#define DISABLE_485_RECV()	(GPIO_SetBits(GPIOG, GPIO_Pin_5));
#define ENABLE_485_RECV()	(GPIO_ResetBits(GPIOG, GPIO_Pin_5));

//static RS485_BUF_STRUCT g_rs485_struct;
//static uint8 rs485_rx_data[RS485_MAX_BUF_LEN];


// wei chai do this
extern void RS485RecData(uint8 *data, uint16 len);


// Local functions
#if 0
static void RS485ParaInit(void)
{
	g_rs485_struct.rx_counter = 0;
	g_rs485_struct.rx_timeout_decounter = 0;
	g_rs485_struct.rx_done_flag = INVALID_VAL_55;
}
#endif

/*****  interface for wei chai  *****/
// 0 : success    1: failed
uint8 RS485OpenPort(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOG, ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);//使能USART2时钟
 
	//串口2对应引脚复用映射
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2); //GPIOA2复用为USART2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2); //GPIOA3复用为USART2
	
	//USART2端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA2，PA3

   //USART2 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART2, &USART_InitStructure); //初始化串口2
	
	USART_Cmd(USART2, ENABLE);  //使能串口2
	
	USART_ClearFlag(USART2, USART_FLAG_TC);

	// enable recv
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

	// 485_en pin
    GPIO_InitStructure.GPIO_Pin= GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;  //推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL; 
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //高速GPIO
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	
	ENABLE_485_RECV(); //recv mode

	return 0;
}

// 0 : success    1: failed
uint8 RS485ClosePort(void)
{
	// disble recv
	USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);//关闭相关中断
	USART_DeInit(USART2);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, DISABLE);//关闭USART2时钟

	return 0;
}

uint8 RS485SendData(uint8 *data, uint16 len)
{
	uint16 i;

	DISABLE_485_RECV();
	
	for(i=0; i<len; i++)
	{
		USART_SendData(USART2, data[i]);
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	}

	ENABLE_485_RECV();

	return 0;
}

void USART2_IRQHandler(void)                	//串口2中断服务程序
{
	u8 Res;
#if SYSTEM_SUPPORT_OS  //使用UCOS操作系统
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res = USART_ReceiveData(USART2);				//读取接收到的数据
		
	#ifdef UART2_BRIDDGE_UART1
		USART_SendData(USART1, Res);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	#else
		#if 0
		if(g_rs485_struct.rx_counter >= RS485_MAX_BUF_LEN)
		{
			g_rs485_struct.rx_counter -= 1;
		}
		g_rs485_struct.rx_buf[g_rs485_struct.rx_counter++] = Res;
		g_rs485_struct.rx_timeout_decounter = 3;
		#endif

		RS485RecData(&Res, 1);
	#endif
  } 
	
#if SYSTEM_SUPPORT_OS  
	OSIntExit();    	//退出中断
#endif
}

#if 0
// 检查数据是否接受完成   0 : success
uint8 RS485TryRxDone(void)
{
	uint16 rx_bytes;
	uint16 i;
	
	if(g_rs485_struct.rx_timeout_decounter > 0)
	{
		g_rs485_struct.rx_timeout_decounter -= 1;
	}
	
	if((g_rs485_struct.rx_timeout_decounter == 0)&&(g_rs485_struct.rx_counter > 0))
	{
		if(g_rs485_struct.rx_counter <= RS485_MAX_BUF_LEN)
		{
			rx_bytes = g_rs485_struct.rx_counter;
			MemCpy(rs485_rx_data, g_rs485_struct.rx_buf, rx_bytes);
			
			RS485ParaInit();

			RS485RecData(rs485_rx_data, rx_bytes);
		
			#ifdef UART_485_DEBUG
				for(i=0; i < rx_bytes; i++)
				{
					printf("%02X", rs485_rx_data[i]);
				}
			#endif

			return 0;
		}
	}
	
	return 1;
}
#endif

