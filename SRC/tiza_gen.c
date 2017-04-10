
#define GEN_GLOBAL
#define GEN_DEBUG

#include "tiza_include.h"
#include "tiza_gen.h"
#include "tiza_selfdef.h"


#define GSM_MAX_BUF_LEN 1500
	
#define TERMINAL_ID_LEN				5			///终端ID长度

#define SIM_CARD_IMSI_LEN			15			///SIM卡IMSI长度
#define GSM_IMEI_LEN				15


typedef struct
{
  uint8   Business;  	//业务
  uint8   Ope;    		//服务
  uint8   Flow;    		//流
  uint8*  SendData;    	//AT指令的参数
  uint16  SendDataLen; 	//AT指令的参数的长度
  uint8*  AckData;     	//AT指令模块应答的数据
  uint16  AckDataLen; 	//AT指令模块应答的长度
  uint16  AckTimeOut; 	//该AT指令发送后判断超时的时间
  uint16  Delay;        //如果一个指令执行完毕后需要等待一段时间才能执行下一条指令
  uint8   TryTime;		//尝试次数
}GEN_FLOW;

typedef enum
{
    AT_RUN_STATUS_SEND_CMD = 0,
    AT_RUN_STATUS_WAIT_ACK,
    AT_RUN_STATUS_SEND_EXTRA,
    AT_RUN_STATUS_WAIT_EXTRA_ACK,
    AT_RUN_STATUS_DELAY_NEXT_CMD
}AT_RUN_STATUS;

typedef enum
{
    AT_POWER_ON_STATUS_OFF = 0,
    AT_POWER_ON_STATUS_ON,
    AT_POWER_ON_STATUS_CTRL_ON,
    AT_POWER_ON_STATUS_CTRL_OFF,
    AT_POWER_ON_STATUS_DELAY
}AT_POWER_ON_STATUS;


typedef struct
{
	uint8 *cmd_text;/// 指令内容
	uint16 max_ms_wait;///最长等待秒数
	uint8 tx_max_count;///最大发送次数
	//uint8 exe_flag;///执行结果
	//void (*fun)(uint8 *data,uint16 len,uint8 flag);///函数处理
}AT_CMD_STRUCT;///AT命令结构体


typedef struct
{
	uint8 rx_done_flag;
	uint8 rx_buf[GSM_MAX_BUF_LEN];
	uint16 rx_counter;
	uint16 rx_timeout_decounter;

	uint8 reset_counter;///GSM模块重启计数
	uint8 no_simcard_flag;
	uint8 csq_val;
	uint8 imei[GSM_IMEI_LEN];
	
}GSM_STRUCT;///GSM数据结构体

typedef enum
{
    TYPE_TCP,
    TYPE_UDP
}SOCKET_TYPE;

typedef enum
{
    MASTER_INDEX = 0,
    SLAVE_INDEX,
    SERVER_MAX_NUM
}IP_INDEX;

typedef enum{
	DEVICE_STATUS_NET_OK = 0,
	DEVICE_STATUS_INIT,		
	DEVICE_STATUS_RESET,
	DEVICE_STATUS_NET_NOT_OPEN,
	DEVICE_STATUS_NO_SIM_CARD
}DEVICE_STATUS;

typedef enum
{
    PROCESS_STATUS_INIT = 0,
    PROCESS_STATUS_CONNECT_NET,
    PROCESS_STATUS_CONNECT_SERVER,
    PROCESS_STATUS_SEND,
    PROCESS_STATUS_RECV,
    PROCESS_STATUS_DISCONNECT_SERVER,
	PROCESS_STATUS_DISCONNECT_NET,
    PROCESS_STATUS_RESET
}PROCESS_STATUS;

typedef struct
{
	IP_NUM ip_num;
	ip_st server_addr;
	SERVER_STATUS server_status;
	SEND_STATUS send_status;
}SERVER_INFO_STRUCT;

typedef struct
{
	uint8 data[GSM_MAX_BUF_LEN];
	uint16 len;
}BUFFER_STRUCT;

static GSM_STRUCT g_gsm_struct;


typedef enum
{
	AT_INDEX = 0, 
	AT_E0_INDEX,
	AT_IMEI_INDEX,
	AT_CSQ_INDEX,
	AT_CREG_INDEX,
	AT_CGREG_INDEX,
	AT_CSOCKAUTH_INDEX,
	AT_CIPMODE_INDEX,
	AT_NETOPEN_INDEX,
	AT_NETCLOSE_INDEX,
	AT_CIPOPEN_UDP_INDEX,
	AT_CIPOPEN_TCP_INDEX,
	AT_CIPCLOSE_INDEX,
	AT_CIPSEND_TCP_INDEX,
	AT_CIPSEND_UDP_INDEX,

	AT_POWER_ON_INDEX,
	AT_POWER_RESET_INDEX
}AT_CMD_INDEX;


#define WAIT_100MS 100	///延时
#define WAIT_1S    3000	///延时1s
#define WAIT_3S    3000	///延时3s
#define SEND_1T 1		///发送1次

#define EXE_NO	 	0X00 ///未执行
#define EXE_OK	 	0X01
#define EXE_FAIL 	0X02

#define NETOPEN_EXTRA_WAIT_TIME	60000
#define NETCLOSE_EXTRA_WAIT_TIME 10000

#define CIPOPEN_EXTRA_WAIT_TIME	30000
#define CIPSEND_EXTRA_WAIT_TIME	10000
#define CIPCLOSE_EXTRA_WAIT_TIME 10000

#define CIPSEND_FLAG_WAIT_TIME 100

#define NEXT_AT_CMD_DELAY	500

#define CHECK_UART_FRAME_INTERVAL 50

// power control
#define ON_MODEM_PWR()		(GPIO_SetBits(GPIOD, GPIO_Pin_8))
#define OFF_MODEM_PWR()		(GPIO_ResetBits(GPIOD, GPIO_Pin_8))
#define LOW_MODEM_IGT()		(GPIO_ResetBits(GPIOC, GPIO_Pin_0))
#define HIGH_MODEM_IGT()	(GPIO_SetBits(GPIOC, GPIO_Pin_0))

#if 1
// local functions
static uint8 AtTx(void);
static uint8 AtE0Tx(void);
static uint8 AtImeiTx(void);
static uint8 AtCsqTx(void);
static uint8 AtCregTx(void);
static uint8 AtCgregTx(void);

static uint8 AtRx(void);
static uint8 AtE0Rx(void);
static uint8 AtImeiRx(void);
static uint8 AtCsqRx(void);
static uint8 AtCregRx(void);
static uint8 AtCgregRx(void);

// 网络服务相关
static uint8 AtSockAuthTx(void);
static uint8 AtIpModeTx(void);
static uint8 AtNetOpenTx(void);
static uint8 AtIpOpenUdpTx(void);
static uint8 AtIpOpenTcpTx(void);
static uint8 AtIpCloseTx(void);
static uint8 AtIpSendTcpTx(void);
static uint8 AtIpSendUdpTx(void);
static uint8 AtNetCloseTx(void);

static uint8 AtSockAuthRx(void);
static uint8 AtIpModeRx(void);
static uint8 AtNetOpenRx(void);
static uint8 AtIpOpenUdpRx(void);
static uint8 AtIpOpenTcpRx(void);
static uint8 AtIpCloseRx(void);
static uint8 AtIpSendTcpRx(void);
static uint8 AtIpSendUdpRx(void);
static uint8 AtNetCloseRx(void);

// flow function
static uint8 FlowAtIpSendTcp(void);
static uint8 FlowAtIpClose(void);
static uint8 FlowAtIpOpenTcp(void);
static uint8 FlowAtNetClose(void);
static uint8 FlowAtNetOpen(void);
static uint8 FlowAtIpMode(void);
static uint8 FlowAtSockAuth(void);
static uint8 FlowAtCgreg(void);
static uint8 FlowAtCreg(void);
static uint8 FlowAtCsq(void);
static uint8 FlowAtE0(void);
static uint8 FlowAt(void);


// process function
static uint8 GenInitProcess(void);
static uint8 GenConnectProcess(void);
static uint8 GenDisconnectNetProcess(void);
static uint8 GenConnectSerProcess(void);
static uint8 GenDisconnectSerProcess(void);
static uint8 GenSendDataProcess(void);
static uint8 GenRecvDataProcess(void);
static uint8 GenResetProcess(void);

static void GsmPowerInit(void);
static void GsmUsartInit(u32 uart_bpr,uint8 data_bits,uint8 stop_bits,uint8 parity_check);

//static void GsmSendAtCmd(uint8 cmd_index,uint8 app_data[],uint8 app_len,uint8 mat_data[],uint8 mat_len);

static uint8 GenTryRxDone(void);

static void GsmUartIsRxDone(void);

static void GsmUsartParaInit(void);
static void GsmUsartFixedLenSend(uint8 data[],uint16 len);

static void GenSendAtCmd(void);

static uint8 TryParseRecvData(uint8 * Data, uint16 Len);

#endif


static GEN_FLOW g_gen_flow = {PROCESS_STATUS_RECV, 0, 0, 0, 0, 0, 0, 0, 0, 0};

static SERVER_INFO_STRUCT g_server_info[SERVER_MAX_NUM];

static IP_NUM g_operation_ip_num = 0;

static DEVICE_STATUS g_device_status = DEVICE_STATUS_NET_NOT_OPEN;
static GEN_INIT_STATUS g_init_status = GEN_INIT_NO;
static NET_LINK_STATUS g_net_link_status = NET_LINK_STATUS_CONNECT_FAILED;

static BUFFER_STRUCT g_send_buffer;

static uint16 g_check_uart_frame_interval = CHECK_UART_FRAME_INTERVAL;

static uint8 gen_rx_data[GSM_MAX_BUF_LEN];
static uint8 gen_rx_payload_data[GSM_MAX_BUF_LEN];
extern void GenRecData(uint8 IPnum, uint8 *Dat, uint16 Len);


// Timer interupt
static void TIM3IntInit(u16 arr,u16 psc);
static void TIM3IntDisable(void);
static uint32 g_1ms_count = 0;


#if 1

#define UDP_LOCAL_PORT	8888

uint8 AT[] = {"AT\x0d"};///AT命令
uint8 AT_E0[] = {"ATE0\x0d"};///关闭回显

// sim card info
uint8 AT_IPR[] = {"AT+IPR=38400;&w\x0d"};///设置波特率为38400
uint8 AT_CIMI[] = {"AT+CIMI\x0d\x0a"};///读取SIM卡IMSI
uint8 AT_CNUM[] = {"AT+CNUM\x0d"};///查询本机号码
uint8 AT_CPBS_ON[] = {"AT+CPBS=\"ON\"\x0d"};///设置本机号使能
uint8 AT_CPBW[] = {"AT+CPBW=1,"};///写入本机号
uint8 AT_IMEI[] = {"AT+SMEID?\x0d\x0a"};///查询GSM模块的IMEI

// check info
uint8 AT_CSQ[] = {"AT+CSQ\x0d"};///查询信号强度
uint8 AT_CREG[] = {"AT+CREG?\x0d"};///查询注册情况
uint8 AT_CGREG[] = {"AT+CGREG?\x0d"};///查询网络注册情况

// message

// call

// low power

// 网络服务命令
uint8 AT_CSOCKAUTH[] = {"AT+CSOCKAUTH=,,\"ctnet@mycdma.cn\",\"vnet.mobi\"\x0d\x0a"};           ///设置网络
//uint8 AT_CSOCKAUTH[] = {"AT+CSOCKAUTH=,,\"CARD\",\"CARD\"\x0d\x0a"};           ///设置网络

uint8 AT_NETOPEN[] = {"AT+NETOPEN\x0d\x0a"};                           ///打开网络
uint8 AT_NETCLOSE[] = {"AT+NETCLOSE\x0d\x0a"};                         ///关闭网络

uint8 AT_CIPOPEN_TCP[] = {"AT+CIPOPEN=%u,\"TCP\",\"%s\",%s\x0d\x0a"};       	 ///打开TCP链接
uint8 AT_CIPSEND_TCP[] = {"AT+CIPSEND=%u,%u\x0d\x0a"};           		///发送TCP数据

uint8 AT_CIPOPEN_UDP[] = {"AT+CIPOPEN=%u,\"UDP\",,,%u\x0d\x0a"};         ///打开UDP链接
uint8 AT_CIPSEND_UDP[] = {"AT+CIPSEND=%u,%u,%s,%s\x0d\x0a"};           		///发送UDP数据

uint8 AT_CIPCLOSE[] = {"AT+CIPCLOSE=%u\x0d\x0a"};               	   ///关闭链接

uint8 AT_CIPMODE[] = {"AT+CIPMODE=%u\x0d\x0a"};               	   	///mode type: 0 for cmd mode,1 for transparent mode

uint8 AT_IPADDR[] = "AT+IPADDR\x0d\x0a";       				// CHECK IP ADDR

// external cmd
uint8 AT_CIPHEAD_ON[] = {"AT+CIPHEAD=1\x0d\x0a"};			// ADD "+IPD" HEADER
uint8 AT_CIPHEAD_OFF[] = {"AT+CIPHEAD=0\x0d\x0a"};			// DONT ADD "+IPD" HEADER

uint8 AT_CIPSRIP_ON[] = {"AT+CIPHEAD=1\x0d\x0a"};			// SHOW "RECV FROM" HEADER
uint8 AT_CIPSRIP_OFF[] = {"AT+CIPHEAD=0\x0d\x0a"};			// DONT SHOW "RECV FROM" HEADER

uint8 RECV_IPD_ACK[] = "+IPD";       		// RECV DATA LEN, ADD "+IPD" HEADER
uint8 RECV_FROM_ACK[] = "RECV FROM:";       // RECV FORM: STRING IF SET SHOW "RECV FROM" HEADER

// ack
uint8 OK_ACK[] = "OK";
uint8 CONNECT[] = "CONNECT";
uint8 CRLF_ACK[] = "\x0d\x0a";
uint8 NETOPEN_EXTRA_OK[] = "+NETOPEN: 0";	// NET IS OPENED SUCCESS
uint8 NETCLOSE_EXTRA_OK[] = "+NETCLOSE: 0";	// NET IS CLOSED SUCCESS
uint8 CIPOPEN_EXTRA_OK[] = "+CIPOPEN: %u,0";	// IP IS OPENED SUCCESS
uint8 CIPSEND_EXTRA_OK[] = "+CIPSEND: %u,%u,%u";	// SEND SUCCESS
uint8 CIPCLOSE_EXTRA_OK[] = "+CIPCLOSE: %u,0";	// IP IS CLOSED SUCCESS
uint8 NETOPEN_ALREADY_OPENED[] = "+IP ERROR: Network is already opened";	// NET IS OPENED ALREADY

static uint8 gsm_uart_rx_data[GSM_MAX_BUF_LEN];


AT_CMD_STRUCT g_at_cmd_struct[] = 
		{
			{AT, 		WAIT_100MS*3,	30*SEND_1T},
			{AT_E0, 	WAIT_100MS*3,	3*SEND_1T},
			{AT_IMEI,	WAIT_100MS*3,	3*SEND_1T},
			{AT_CSQ,	WAIT_100MS*3,	50*SEND_1T},
			{AT_CREG,	WAIT_100MS*3,	20*SEND_1T},
			{AT_CGREG,	WAIT_100MS*3,	3*SEND_1T},

	        {AT_CSOCKAUTH, 	WAIT_100MS*3,	3*SEND_1T},
	        {AT_CIPMODE,	WAIT_100MS*3,	3*SEND_1T},
	        {AT_NETOPEN,	WAIT_100MS*3,	3*SEND_1T},
			{AT_NETCLOSE,	WAIT_100MS*3,	3*SEND_1T},
	        {AT_CIPOPEN_UDP,	WAIT_100MS*3,	1*SEND_1T},
			{AT_CIPOPEN_TCP,	WAIT_100MS*3,	3*SEND_1T},
			{AT_CIPCLOSE,		WAIT_100MS*3,	1*SEND_1T},
			{AT_CIPSEND_TCP,	WAIT_100MS*3,	10*SEND_1T},
			{AT_CIPSEND_UDP,	WAIT_100MS*3,	1*SEND_1T},
		};
							

static void delay_ms(uint16 time)
{ 
  uint32 tmp;// = time * 21;
  while(time--)
  {
    tmp = 21000;
    while(tmp--)
    {
      __NOP();
      __NOP();
      __NOP();
      __NOP();
    }
  }
}

static uint16 FindFirstCrLf(uint16 start, const uint8 *data, uint16 Len)
{
	uint16 i;
	
	for(i=start; i < Len; i++)
	{
		if(data[i] == 0x0d || data[i] == 0x0a)
		{
			return i;
		}
	}

	return 0;
}

static uint16 FindFirstChar(uint16 start, char key_char,const uint8 *data, uint16 Len)
{
	uint16 i;
	
	for(i=start; i < Len; i++)
	{
		if(data[i] == key_char)
		{
			return i;
		}
	}

	return 0;
}

static uint16 Str2Uint16(uint8 *data, uint16 len)
{
	uint16 i, sum=0;
	
	if(len == 0 || len > 5)
		return 0;

	sum += data[0] - 0x30;
	
	for(i=1; i < len; i++)
	{
		sum *= 10;
		sum += data[i] - 0x30;
	}

	return sum;
}

// at functions

static void GenSendAtCmd(void)
{
	uint16 i;
	
	if(g_gen_flow.SendData != NULL && g_gen_flow.SendDataLen > 0)
	{
		GsmUsartParaInit();
		GsmUsartFixedLenSend(g_gen_flow.SendData, g_gen_flow.SendDataLen);
		

		#ifdef GEN_DEBUG
		for(i=0; i < g_gen_flow.SendDataLen; i++)
		{
			printf("%c", g_gen_flow.SendData[i]);
		}
		printf("\r\n");
		#endif
	}
}

static uint8 TryParseRecvData(uint8 * Data, uint16 Len)
{
	uint16 i, mat_index, mat_end, recv_data_len, mat_end2, data_index=0, len;
	IP_NUM ip_num = 0;
	uint8 *data;
	
	if(Data == NULL || Len == 0)
	{
		return 1;
	}
	
	// check ip to ipnum
	while(data_index < Len)
	{
		ip_num = 0;

		data =  Data + data_index;
		len = Len - data_index;
		
		mat_index = SubMatch(RECV_FROM_ACK, StrLen(RECV_FROM_ACK, 0), data, len);

		if(mat_index > 0)
		{
			mat_end = FindFirstChar(mat_index, ':', data, len);

			mat_end2 = FindFirstCrLf(mat_end+1, data, len);
			
			if(mat_end > 0 && mat_end2 > 0)
			{				
				for(i=0; i < SERVER_MAX_NUM; i++)
				{
					if(MemCmp(data + mat_index, g_server_info[i].server_addr.ip, mat_end - mat_index) == TRUE
						&& MemCmp(data + mat_end + 1, g_server_info[i].server_addr.port, mat_end2 - mat_end - 1) == TRUE)
					{
						ip_num = i+1;
						break;
					}
				}
			}
			else
			{
				return 1;
			}
		}
		else
		{
			return 1;
		}

		if(ip_num == 0)
		{
			return 1;
		}

		#ifdef GEN_DEBUG
		printf("IP num = %u\r\n", ip_num);
		#endif
		
		// has recv data
		mat_index = SubMatch(RECV_IPD_ACK, StrLen(RECV_IPD_ACK, 0), data, len);
		
		if(mat_index > 0)
		{
			mat_end = FindFirstCrLf(mat_index, data, len);

			if(mat_end > 0)
			{
				data_index += mat_end;
				
				recv_data_len = Str2Uint16(data + mat_index, mat_end - mat_index);
				
			#ifdef GEN_DEBUG
				printf("recv len = %u\r\n", recv_data_len);
			#endif
			
				if(recv_data_len > 0 && recv_data_len < len)
				{
					data_index += recv_data_len;
					
					MemCpy(gen_rx_payload_data, data + mat_index + (mat_end - mat_index) + 2, recv_data_len);

					GenRecData(ip_num, gen_rx_payload_data, recv_data_len);

					return 0;
				}
			}
			else
			{
				return 1;
			}
		}
		else
		{
			return 1;
		}
	}
}

static uint8 g_power_on_status = AT_POWER_ON_STATUS_ON;

uint8 PowerOnDelay(void)
{
	if(g_gen_flow.Delay > 0)
	{
		return 2;
	}
	else
	{
		g_power_on_status++;
		return 0;
	}
}

static uint8 FlowPowerOn(void)
{

	switch(g_gen_flow.Flow)
	{
		case AT_POWER_ON_STATUS_ON:
			g_power_on_status = AT_POWER_ON_STATUS_ON;
			
			ON_MODEM_PWR();		///再开机
			g_gen_flow.Delay = 1000;
			g_gen_flow.Flow = AT_POWER_ON_STATUS_DELAY;
			break;

		case AT_POWER_ON_STATUS_CTRL_ON:
			HIGH_MODEM_IGT();
			g_gen_flow.Delay = 500;
			g_gen_flow.Flow = AT_POWER_ON_STATUS_DELAY;
			break;

		case AT_POWER_ON_STATUS_CTRL_OFF:
			LOW_MODEM_IGT();
			g_gen_flow.Delay = 3000;
			g_gen_flow.Flow = AT_POWER_ON_STATUS_DELAY;
			break;

		case AT_POWER_ON_STATUS_DELAY:
			if(PowerOnDelay() == 0)
			{
				g_gen_flow.Flow = g_power_on_status;
			}

			if(g_power_on_status > AT_POWER_ON_STATUS_CTRL_OFF)
			{
				return 0;
			}
			break;
	}

	return 1;
}


// 0: success  1: failed  2: running
uint8 AtDelay(void)
{
	if(g_gen_flow.Delay > 0)
	{
		return 2;
	}
	else
	{
		return 0;
	}
}

uint8 AtTx(void)
{
	g_gen_flow.SendData = g_at_cmd_struct[g_gen_flow.Ope].cmd_text;
	g_gen_flow.SendDataLen = StrLen(g_at_cmd_struct[g_gen_flow.Ope].cmd_text, 0);
	
	GenSendAtCmd();
	
	g_gen_flow.Flow = AT_RUN_STATUS_WAIT_ACK;
	g_gen_flow.AckTimeOut = g_at_cmd_struct[g_gen_flow.Ope].max_ms_wait;

	return 0;
}

uint8 AtRx(void)
{
	uint16 rx_len, mat_index,i;
	
	// ack back
	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		rx_len = g_gsm_struct.rx_counter;
		MemCpy(gsm_uart_rx_data, g_gsm_struct.rx_buf, rx_len);

		GsmUsartParaInit();
		
		mat_index = SubMatch(OK_ACK, StrLen(OK_ACK, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
			g_gen_flow.Delay = NEXT_AT_CMD_DELAY;
			
			return 0;
		}
	}

	// not time out
	if(g_gen_flow.AckTimeOut > 0)
	{
		return 2;
	}

	#ifdef GEN_DEBUG
	printf("at null ack time out\r\n");
	#endif

	return 1;
}

uint8 FlowAt(void)
{

	switch(g_gen_flow.Flow)
	{
		case AT_RUN_STATUS_SEND_CMD:			
			return AtTx();

		case AT_RUN_STATUS_WAIT_ACK:
			return AtRx();

		case AT_RUN_STATUS_DELAY_NEXT_CMD:
			return AtDelay();
	}
}

uint8 AtE0Tx(void)
{
	g_gen_flow.SendData = g_at_cmd_struct[g_gen_flow.Ope].cmd_text;
	g_gen_flow.SendDataLen = StrLen(g_at_cmd_struct[g_gen_flow.Ope].cmd_text, 0);
	
	GenSendAtCmd();

	g_gen_flow.Flow = AT_RUN_STATUS_WAIT_ACK;
	g_gen_flow.AckTimeOut = g_at_cmd_struct[g_gen_flow.Ope].max_ms_wait;

	return 0;
}

uint8 AtE0Rx(void)
{
	uint16 rx_len, mat_index;
	
	// ack back
	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		rx_len = g_gsm_struct.rx_counter;
		MemCpy(gsm_uart_rx_data, g_gsm_struct.rx_buf, rx_len);

		GsmUsartParaInit();
		
		mat_index = SubMatch(OK_ACK, StrLen(OK_ACK, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
			g_gen_flow.Delay = NEXT_AT_CMD_DELAY;
			
			return 0;
		}
	}

	// not time out
	if(g_gen_flow.AckTimeOut > 0)
	{
		return 2;
	}

	#ifdef GEN_DEBUG
	printf("at e0: time out\r\n");
	#endif

	return 1;
}

uint8 FlowAtE0(void)
{

	switch(g_gen_flow.Flow)
	{
		case AT_RUN_STATUS_SEND_CMD:
			return AtE0Tx();

		case AT_RUN_STATUS_WAIT_ACK:
			return AtE0Rx();

		case AT_RUN_STATUS_DELAY_NEXT_CMD:
			return AtDelay();
	}
}

uint8 AtImeiTx(void)
{
	g_gen_flow.SendData = g_at_cmd_struct[g_gen_flow.Ope].cmd_text;
	g_gen_flow.SendDataLen = StrLen(g_at_cmd_struct[g_gen_flow.Ope].cmd_text, 0);
	
	GenSendAtCmd();

	g_gen_flow.Flow = AT_RUN_STATUS_WAIT_ACK;
	g_gen_flow.AckTimeOut = g_at_cmd_struct[g_gen_flow.Ope].max_ms_wait;

	return 0;
}

uint8 AtImeiRx(void)
{
	uint8 res;
	uint8 cmp_data[] = {"+SMEID: "};
	
	uint16 rx_len, mat_index;
	
	// ack back
	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		rx_len = g_gsm_struct.rx_counter;
		MemCpy(gsm_uart_rx_data, g_gsm_struct.rx_buf, rx_len);

		GsmUsartParaInit();
		
		mat_index = SubMatch(OK_ACK, StrLen(OK_ACK, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			mat_index = SubMatch(cmp_data,StrLen(cmp_data,0),gsm_uart_rx_data,rx_len);
			if(mat_index > 0)
			{
				res = IsValidAscii(gsm_uart_rx_data+mat_index,GSM_IMEI_LEN-1);	// 14
				if(res)
				{
					MemCpy(g_gsm_struct.imei,gsm_uart_rx_data+mat_index,GSM_IMEI_LEN-1);	// 14
					g_gsm_struct.imei[GSM_IMEI_LEN-1] = '\0';

					#ifdef GEN_DEBUG
					printf("IMEI: %s\r\n", g_gsm_struct.imei);
					#endif

					g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
					g_gen_flow.Delay = NEXT_AT_CMD_DELAY;
					
					return 0;
				}
				else
				{
					return 1;
				}
			}	
		}
	}

	// not time out
	if(g_gen_flow.AckTimeOut > 0)
	{
		return 2;
	}

	#ifdef GEN_DEBUG
	printf("at smeid: time out\r\n");
	#endif

	return 1;
}

uint8 FlowAtImei(void)
{

	switch(g_gen_flow.Flow)
	{
		case AT_RUN_STATUS_SEND_CMD:
			return AtImeiTx();

		case AT_RUN_STATUS_WAIT_ACK:
			return AtImeiRx();

		case AT_RUN_STATUS_DELAY_NEXT_CMD:
			return AtDelay();
	}
}

uint8 AtCsqTx(void)
{
	g_gen_flow.SendData = g_at_cmd_struct[g_gen_flow.Ope].cmd_text;
	g_gen_flow.SendDataLen = StrLen(g_at_cmd_struct[g_gen_flow.Ope].cmd_text, 0);
	
	GenSendAtCmd();
	
	g_gen_flow.Flow = AT_RUN_STATUS_WAIT_ACK;
	g_gen_flow.AckTimeOut = g_at_cmd_struct[g_gen_flow.Ope].max_ms_wait;

	return 0;
}

static uint8 ParseCsqValue(uint8 *data, uint16 len)
{
	uint16 mat_index;
	uint8 res,tmp_val;
	uint8 cmp_data[] = {"+CSQ: "};

	res = 1;
	
	mat_index = SubMatch(cmp_data,StrLen(cmp_data,0),data,len);
	if(mat_index > 0)
	{
		if(data[mat_index+1] == ',')
		{
			if(IsValidNum(data+mat_index,1))
			{
				tmp_val = data[mat_index] - '0';
				res = 0;
			}
		}
		else
		{
			if(IsValidNum(data+mat_index,2))
			{
				tmp_val = data[mat_index] - '0';
				tmp_val *= 10;
				tmp_val += data[mat_index+1] - '0';
				res = 0;
			}
		}
		
		if(res == 0)
		{
			if(tmp_val <= 31)
			{
				g_gsm_struct.csq_val = tmp_val;
				
				#ifdef GEN_DEBUG
				printf("csq value = %u\r\n", g_gsm_struct.csq_val);
				#endif
			}
			else
			{
				return 1;
			}
		}
	}	

	return res;
}

uint8 AtCsqRx(void)
{
	uint16 rx_len, mat_index;

	uint8 res,tmp_val;
	uint8 cmp_data[] = {"+CSQ: "};
	
	// ack back
	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		rx_len = g_gsm_struct.rx_counter;
		MemCpy(gsm_uart_rx_data, g_gsm_struct.rx_buf, rx_len);

		GsmUsartParaInit();
		
		mat_index = SubMatch(OK_ACK, StrLen(OK_ACK, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			if(ParseCsqValue(gsm_uart_rx_data, rx_len) == 1)
			{
				return 1;
			}

			g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
			g_gen_flow.Delay = NEXT_AT_CMD_DELAY;
			
			return 0;
		}
	}

	// not time out
	if(g_gen_flow.AckTimeOut > 0)
	{
		return 2;
	}

	#ifdef GEN_DEBUG
	printf("at csq: time out\r\n");
	#endif

	return 1;
}

uint8 FlowAtCsq(void)
{

	switch(g_gen_flow.Flow)
	{
		case AT_RUN_STATUS_SEND_CMD:
			return AtCsqTx();

		case AT_RUN_STATUS_WAIT_ACK:
			return AtCsqRx();

		case AT_RUN_STATUS_DELAY_NEXT_CMD:
			return AtDelay();
	}
}


uint8 AtCregTx(void)
{
	g_gen_flow.SendData = g_at_cmd_struct[g_gen_flow.Ope].cmd_text;
	g_gen_flow.SendDataLen = StrLen(g_at_cmd_struct[g_gen_flow.Ope].cmd_text, 0);
	
	GenSendAtCmd();

	g_gen_flow.Flow = AT_RUN_STATUS_WAIT_ACK;
	g_gen_flow.AckTimeOut = g_at_cmd_struct[g_gen_flow.Ope].max_ms_wait;

	return 0;
}

uint8 AtCregRx(void)
{
	uint8 cmp_data[] = {"+CREG: "};
	
	uint16 rx_len, mat_index;
	
	// ack back
	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		rx_len = g_gsm_struct.rx_counter;
		MemCpy(gsm_uart_rx_data, g_gsm_struct.rx_buf, rx_len);

		GsmUsartParaInit();
		
		mat_index = SubMatch(OK_ACK, StrLen(OK_ACK, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			mat_index = SubMatch(cmp_data,StrLen(cmp_data,0),gsm_uart_rx_data,rx_len);

			if(mat_index > 0)
			{
				if((gsm_uart_rx_data[mat_index+2] == '1')||(gsm_uart_rx_data[mat_index+2] == '5'))
				{
					g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
					g_gen_flow.Delay = NEXT_AT_CMD_DELAY;
			
					return 0;
				}
			}
			
			return 1;
		}
	}

	// not time out
	if(g_gen_flow.AckTimeOut > 0)
	{
		return 2;
	}

	#ifdef GEN_DEBUG
	printf("at creg: time out\r\n");
	#endif

	return 1;	
}

uint8 FlowAtCreg(void)
{

	switch(g_gen_flow.Flow)
	{
		case AT_RUN_STATUS_SEND_CMD:
			return AtCregTx();

		case AT_RUN_STATUS_WAIT_ACK:
			return AtCregRx();

		case AT_RUN_STATUS_DELAY_NEXT_CMD:
			return AtDelay();
	}
}

uint8 AtCgregTx(void)
{
	g_gen_flow.SendData = g_at_cmd_struct[g_gen_flow.Ope].cmd_text;
	g_gen_flow.SendDataLen = StrLen(g_at_cmd_struct[g_gen_flow.Ope].cmd_text, 0);
	
	GenSendAtCmd();

	g_gen_flow.Flow = AT_RUN_STATUS_WAIT_ACK;
	g_gen_flow.AckTimeOut = g_at_cmd_struct[g_gen_flow.Ope].max_ms_wait;

	return 0;
}

uint8 AtCgregRx(void)
{
	uint8 cmp_data[] = {"+CGREG: "};

	uint16 rx_len, mat_index;
	
	// ack back
	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		rx_len = g_gsm_struct.rx_counter;
		MemCpy(gsm_uart_rx_data, g_gsm_struct.rx_buf, rx_len);

		GsmUsartParaInit();
		
		mat_index = SubMatch(OK_ACK, StrLen(OK_ACK, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			mat_index = SubMatch(cmp_data,StrLen(cmp_data,0),gsm_uart_rx_data,rx_len);
			if(mat_index > 0)
			{
				if((gsm_uart_rx_data[mat_index+2] == '1')||(gsm_uart_rx_data[mat_index+2] == '5'))
				{
					g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
					g_gen_flow.Delay = NEXT_AT_CMD_DELAY;
			
					return 0;
				}
			}
		
			return 1;
		}
	}

	// not time out
	if(g_gen_flow.AckTimeOut > 0)
	{
		return 2;
	}

	#ifdef GEN_DEBUG
	printf("at cgreg: time out\r\n");
	#endif

	return 1;
}

uint8 FlowAtCgreg(void)
{

	switch(g_gen_flow.Flow)
	{
		case AT_RUN_STATUS_SEND_CMD:
			return AtCgregTx();

		case AT_RUN_STATUS_WAIT_ACK:
			return AtCgregRx();

		case AT_RUN_STATUS_DELAY_NEXT_CMD:
			return AtDelay();
	}
}


// 网络服务相关函数
uint8 AtSockAuthTx(void)
{
	g_gen_flow.SendData = g_at_cmd_struct[g_gen_flow.Ope].cmd_text;
	g_gen_flow.SendDataLen = StrLen(g_at_cmd_struct[g_gen_flow.Ope].cmd_text, 0);
	
	GenSendAtCmd();
	
	g_gen_flow.Flow = AT_RUN_STATUS_WAIT_ACK;
	g_gen_flow.AckTimeOut = g_at_cmd_struct[g_gen_flow.Ope].max_ms_wait;

	return 0;
}

uint8 AtSockAuthRx(void)
{
	uint16 rx_len, mat_index;
	
	// ack back
	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		rx_len = g_gsm_struct.rx_counter;
		MemCpy(gsm_uart_rx_data, g_gsm_struct.rx_buf, rx_len);

		GsmUsartParaInit();
		
		mat_index = SubMatch(OK_ACK, StrLen(OK_ACK, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
			g_gen_flow.Delay = NEXT_AT_CMD_DELAY;
			
			return 0;
		}
	}

	// not time out
	if(g_gen_flow.AckTimeOut > 0)
	{
		return 2;
	}

	#ifdef GEN_DEBUG
	printf("at sock auth: time out\r\n");
	#endif

	return 1;
}

uint8 FlowAtSockAuth(void)
{

	switch(g_gen_flow.Flow)
	{
		case AT_RUN_STATUS_SEND_CMD:
			return AtSockAuthTx();

		case AT_RUN_STATUS_WAIT_ACK:
			return AtSockAuthRx();

		case AT_RUN_STATUS_DELAY_NEXT_CMD:
			return AtDelay();
	}
}

uint8 AtIpModeTx(void)
{	
	char temp_cmd[40];

	// cmd mode
	sprintf(temp_cmd, g_at_cmd_struct[g_gen_flow.Ope].cmd_text, 0);
	
	g_gen_flow.SendData = temp_cmd;
	g_gen_flow.SendDataLen = StrLen(temp_cmd, 0);
	
	GenSendAtCmd();
	
	g_gen_flow.Flow = AT_RUN_STATUS_WAIT_ACK;
	g_gen_flow.AckTimeOut = g_at_cmd_struct[g_gen_flow.Ope].max_ms_wait;

	return 0;
}

uint8 AtIpModeRx(void)
{	
	uint16 rx_len, mat_index;
	
	// ack back
	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		rx_len = g_gsm_struct.rx_counter;
		MemCpy(gsm_uart_rx_data, g_gsm_struct.rx_buf, rx_len);

		GsmUsartParaInit();
		
		mat_index = SubMatch(OK_ACK, StrLen(OK_ACK, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
			g_gen_flow.Delay = NEXT_AT_CMD_DELAY;
			
			return 0;
		}
	}

	// not time out
	if(g_gen_flow.AckTimeOut > 0)
	{
		return 2;
	}

	#ifdef GEN_DEBUG
	printf("at ip mode: time out\r\n");
	#endif

	return 1;
}

uint8 FlowAtIpMode(void)
{

	switch(g_gen_flow.Flow)
	{
		case AT_RUN_STATUS_SEND_CMD:
			return AtIpModeTx();

		case AT_RUN_STATUS_WAIT_ACK:
			return AtIpModeRx();

		case AT_RUN_STATUS_DELAY_NEXT_CMD:
			return AtDelay();
	}
}

uint8 AtNetOpenTx(void)
{
	g_gen_flow.SendData = g_at_cmd_struct[g_gen_flow.Ope].cmd_text;
	g_gen_flow.SendDataLen = StrLen(g_at_cmd_struct[g_gen_flow.Ope].cmd_text, 0);
	
	GenSendAtCmd();
	
	g_gen_flow.Flow = AT_RUN_STATUS_WAIT_ACK;
	g_gen_flow.AckTimeOut = g_at_cmd_struct[g_gen_flow.Ope].max_ms_wait;

	return 0;
}

uint8 AtNetOpenRx(void)
{
	uint16 rx_len, mat_index;
	
	// ack back
	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		rx_len = g_gsm_struct.rx_counter;
		MemCpy(gsm_uart_rx_data, g_gsm_struct.rx_buf, rx_len);

		GsmUsartParaInit();

		mat_index = SubMatch(NETOPEN_ALREADY_OPENED, StrLen(NETOPEN_ALREADY_OPENED, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
			g_gen_flow.AckTimeOut = NEXT_AT_CMD_DELAY;
	
			return 0;
		}
	
		mat_index = SubMatch(NETOPEN_EXTRA_OK, StrLen(NETOPEN_EXTRA_OK, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
			g_gen_flow.AckTimeOut = NEXT_AT_CMD_DELAY;
	
			return 0;
		}
		
		mat_index = SubMatch(OK_ACK, StrLen(OK_ACK, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_WAIT_EXTRA_ACK;
			g_gen_flow.AckTimeOut = NETOPEN_EXTRA_WAIT_TIME;
	
			return 0;
		}
	}

	// not time out
	if(g_gen_flow.AckTimeOut > 0)
	{
		return 2;
	}

	#ifdef GEN_DEBUG
	printf("at net open: 1st step time out\r\n");
	#endif

	return 1;		    
}

uint8 AtNetOpenExtraRx(void)
{
	uint16 rx_len, mat_index;
	
	// ack back
	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		rx_len = g_gsm_struct.rx_counter;
		MemCpy(gsm_uart_rx_data, g_gsm_struct.rx_buf, rx_len);

		GsmUsartParaInit();

		mat_index = SubMatch(NETOPEN_ALREADY_OPENED, StrLen(NETOPEN_ALREADY_OPENED, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
			g_gen_flow.AckTimeOut = NEXT_AT_CMD_DELAY;
	
			return 0;
		}
		
		mat_index = SubMatch(NETOPEN_EXTRA_OK, StrLen(NETOPEN_EXTRA_OK, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
			g_gen_flow.Delay = NEXT_AT_CMD_DELAY;
			
			return 0;
		}
	}

	// not time out
	if(g_gen_flow.AckTimeOut > 0)
	{
		return 2;
	}

	#ifdef GEN_DEBUG
	printf("at net open: extra time out\r\n");
	#endif

	return 1;
}

uint8 FlowAtNetOpen(void)
{

	switch(g_gen_flow.Flow)
	{
		case AT_RUN_STATUS_SEND_CMD:
			return AtNetOpenTx();

		case AT_RUN_STATUS_WAIT_ACK:
			return AtNetOpenRx();

		case AT_RUN_STATUS_WAIT_EXTRA_ACK:
			return AtNetOpenExtraRx();

		case AT_RUN_STATUS_DELAY_NEXT_CMD:
			return AtDelay();
	}
}

uint8 AtNetCloseTx(void)
{	
	g_gen_flow.SendData = g_at_cmd_struct[g_gen_flow.Ope].cmd_text;
	g_gen_flow.SendDataLen = StrLen(g_at_cmd_struct[g_gen_flow.Ope].cmd_text, 0);
	
	GenSendAtCmd();
	
	g_gen_flow.Flow = AT_RUN_STATUS_WAIT_ACK;
	g_gen_flow.AckTimeOut = g_at_cmd_struct[g_gen_flow.Ope].max_ms_wait;

	return 0;
}

uint8 AtNetCloseRx(void)
{
	uint16 rx_len, mat_index;
	
	// ack back
	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		rx_len = g_gsm_struct.rx_counter;
		MemCpy(gsm_uart_rx_data, g_gsm_struct.rx_buf, rx_len);

		GsmUsartParaInit();

		mat_index = SubMatch(NETCLOSE_EXTRA_OK, StrLen(NETCLOSE_EXTRA_OK, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
			g_gen_flow.AckTimeOut = NEXT_AT_CMD_DELAY;
	
			return 0;
		}
		
		mat_index = SubMatch(OK_ACK, StrLen(OK_ACK, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_WAIT_EXTRA_ACK;
			g_gen_flow.AckTimeOut = NETCLOSE_EXTRA_WAIT_TIME;
	
			return 0;
		}
	}

	// not time out
	if(g_gen_flow.AckTimeOut > 0)
	{
		return 2;
	}

	#ifdef GEN_DEBUG
	printf("at net close: 1st step time out\r\n");
	#endif

	return 1;
}

uint8 AtNetCloseExtraRx(void)
{
	uint16 rx_len, mat_index;
	
	// ack back
	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		rx_len = g_gsm_struct.rx_counter;
		MemCpy(gsm_uart_rx_data, g_gsm_struct.rx_buf, rx_len);

		GsmUsartParaInit();

		TryParseRecvData(gsm_uart_rx_data, rx_len);
		
		mat_index = SubMatch(NETCLOSE_EXTRA_OK, StrLen(NETCLOSE_EXTRA_OK, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
			g_gen_flow.AckTimeOut = NEXT_AT_CMD_DELAY;
	
			return 0;
		}
	}

	// not time out
	if(g_gen_flow.AckTimeOut > 0)
	{
		return 2;
	}

	#ifdef GEN_DEBUG
	printf("at net close: extra time out\r\n");
	#endif

	return 1;
}

uint8 FlowAtNetClose(void)
{

	switch(g_gen_flow.Flow)
	{
		case AT_RUN_STATUS_SEND_CMD:
			return AtNetCloseTx();

		case AT_RUN_STATUS_WAIT_ACK:
			return AtNetCloseRx();

		case AT_RUN_STATUS_WAIT_EXTRA_ACK:
			return AtNetCloseExtraRx();

		case AT_RUN_STATUS_DELAY_NEXT_CMD:
			return AtDelay();
	}
}

uint8 AtIpOpenUdpTx(void)
{	    
}

uint8 AtIpOpenUdpRx(void)
{	    
}

uint8 AtIpOpenTcpTx(void)
{
	char temp_cmd[80];

	sprintf(temp_cmd, (const char*)g_at_cmd_struct[g_gen_flow.Ope].cmd_text, g_operation_ip_num, g_server_info[g_operation_ip_num-1].server_addr.ip, g_server_info[g_operation_ip_num-1].server_addr.port);
	
	g_gen_flow.SendData = (uint8 *)temp_cmd;
	g_gen_flow.SendDataLen = StrLen(temp_cmd, 0);
	
	GenSendAtCmd();

	g_gen_flow.Flow = AT_RUN_STATUS_WAIT_ACK;
	g_gen_flow.AckTimeOut = g_at_cmd_struct[g_gen_flow.Ope].max_ms_wait;

	return 0;
}

uint8 AtIpOpenTcpRx(void)
{
	uint16 rx_len, mat_index;
	uint8 cmp_data[40];
	
	// ack back
	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		rx_len = g_gsm_struct.rx_counter;
		MemCpy(gsm_uart_rx_data, g_gsm_struct.rx_buf, rx_len);

		GsmUsartParaInit();

		TryParseRecvData(gsm_uart_rx_data, rx_len);

		sprintf(cmp_data, (const char*)CIPOPEN_EXTRA_OK, g_operation_ip_num);
				
		mat_index = SubMatch(cmp_data,StrLen(cmp_data, 0),gsm_uart_rx_data,rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
			g_gen_flow.AckTimeOut = NEXT_AT_CMD_DELAY;
	
			return 0;
		}
		
		mat_index = SubMatch(OK_ACK, StrLen(OK_ACK, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_WAIT_EXTRA_ACK;
			g_gen_flow.AckTimeOut = CIPOPEN_EXTRA_WAIT_TIME;
	
			return 0;
		}
	}

	// not time out
	if(g_gen_flow.AckTimeOut > 0)
	{
		return 2;
	}
	
	#ifdef GEN_DEBUG
	printf("at open tcp error: 1st step time out\r\n");
	#endif
	
	return 1;
}

uint8 AtIpOpenTcpExtraRx(void)
{
	uint16 rx_len, mat_index;
	uint8 cmp_data[40];
	
	// ack back
	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		rx_len = g_gsm_struct.rx_counter;
		MemCpy(gsm_uart_rx_data, g_gsm_struct.rx_buf, rx_len);

		GsmUsartParaInit();

		TryParseRecvData(gsm_uart_rx_data, rx_len);
		
		sprintf(cmp_data, (const char*)CIPOPEN_EXTRA_OK, g_operation_ip_num);
				
		mat_index = SubMatch(cmp_data,StrLen(cmp_data, 0),gsm_uart_rx_data,rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
			g_gen_flow.AckTimeOut = NEXT_AT_CMD_DELAY;
	
			return 0;
		}
	}

	// not time out
	if(g_gen_flow.AckTimeOut > 0)
	{
		return 2;
	}

	#ifdef GEN_DEBUG
	printf("at open tcp error: extra time out\r\n");
	#endif
	
	return 1;
}

uint8 FlowAtIpOpenTcp(void)
{

	switch(g_gen_flow.Flow)
	{
		case AT_RUN_STATUS_SEND_CMD:
			return AtIpOpenTcpTx();

		case AT_RUN_STATUS_WAIT_ACK:
			return AtIpOpenTcpRx();

		case AT_RUN_STATUS_WAIT_EXTRA_ACK:
			return AtIpOpenTcpExtraRx();

		case AT_RUN_STATUS_DELAY_NEXT_CMD:
			return AtDelay();
	}
}

uint8 AtIpCloseTx(void)
{	 
	char temp_cmd[40];

	sprintf(temp_cmd, (const char*)g_at_cmd_struct[g_gen_flow.Ope].cmd_text, g_operation_ip_num);
	
	g_gen_flow.SendData = (uint8 *)temp_cmd;
	g_gen_flow.SendDataLen = StrLen(temp_cmd, 0);
	
	GenSendAtCmd();
	
	g_gen_flow.Flow = AT_RUN_STATUS_WAIT_ACK;
	g_gen_flow.AckTimeOut = g_at_cmd_struct[g_gen_flow.Ope].max_ms_wait;

	return 0;
}

uint8 AtIpCloseRx(void)
{	    
	uint16 rx_len, mat_index;
	uint8 cmp_data[40];
	
	// ack back
	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		rx_len = g_gsm_struct.rx_counter;
		MemCpy(gsm_uart_rx_data, g_gsm_struct.rx_buf, rx_len);

		GsmUsartParaInit();
		
		TryParseRecvData(gsm_uart_rx_data, rx_len);

		sprintf(cmp_data, (const char*)CIPCLOSE_EXTRA_OK, g_operation_ip_num);
		
		mat_index = SubMatch(cmp_data, StrLen(cmp_data, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
			g_gen_flow.AckTimeOut = NEXT_AT_CMD_DELAY;
	
			return 0;
		}
		
		mat_index = SubMatch(OK_ACK, StrLen(OK_ACK, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_WAIT_EXTRA_ACK;
			g_gen_flow.AckTimeOut = CIPCLOSE_EXTRA_WAIT_TIME;
	
			return 0;
		}
	}

	// not time out
	if(g_gen_flow.AckTimeOut > 0)
	{
		return 2;
	}

	#ifdef GEN_DEBUG
	printf("at close tcp : 1st step time out\r\n");
	#endif

	return 1;
}

uint8 AtIpCloseExtraRx(void)
{	    
	uint16 rx_len, mat_index;
	uint8 cmp_data[40];
	
	// ack back
	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		rx_len = g_gsm_struct.rx_counter;
		MemCpy(gsm_uart_rx_data, g_gsm_struct.rx_buf, rx_len);

		GsmUsartParaInit();

		TryParseRecvData(gsm_uart_rx_data, rx_len);
		
		sprintf(cmp_data, (const char*)CIPCLOSE_EXTRA_OK, g_operation_ip_num);
		
		mat_index = SubMatch(cmp_data, StrLen(cmp_data, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
			g_gen_flow.AckTimeOut = NEXT_AT_CMD_DELAY;
	
			return 0;
		}
	}

	// not time out
	if(g_gen_flow.AckTimeOut > 0)
	{
		return 2;
	}

	#ifdef GEN_DEBUG
	printf("at close tcp : extra time out\r\n");
	#endif

	return 1;
}

uint8 FlowAtIpClose(void)
{

	switch(g_gen_flow.Flow)
	{
		case AT_RUN_STATUS_SEND_CMD:
			return AtIpCloseTx();

		case AT_RUN_STATUS_WAIT_ACK:
			return AtIpCloseRx();

		case AT_RUN_STATUS_WAIT_EXTRA_ACK:
			return AtIpCloseExtraRx();

		case AT_RUN_STATUS_DELAY_NEXT_CMD:
			return AtDelay();
	}
}

uint8 AtIpSendUdpTx(void)
{
}

uint8 AtIpSendUdpRx(void)
{
}

uint8 AtIpSendTcpTx(void)
{
	char temp_cmd[40];

	sprintf(temp_cmd, (const char*)g_at_cmd_struct[g_gen_flow.Ope].cmd_text, g_operation_ip_num, g_send_buffer.len);
	
	g_gen_flow.SendData = (uint8 *)temp_cmd;
	g_gen_flow.SendDataLen = StrLen(temp_cmd, 0);
	
	GenSendAtCmd();
	
	g_gen_flow.Flow = AT_RUN_STATUS_WAIT_ACK;
	g_gen_flow.AckTimeOut = g_at_cmd_struct[g_gen_flow.Ope].max_ms_wait;

	return 0;
}

uint8 AtIpSendTcpExtraTx(void)
{
	g_gen_flow.SendData = (uint8 *)g_send_buffer.data;
	g_gen_flow.SendDataLen = g_send_buffer.len;
	
	GenSendAtCmd();
	
	g_gen_flow.Flow = AT_RUN_STATUS_WAIT_EXTRA_ACK;
	g_gen_flow.AckTimeOut = CIPSEND_EXTRA_WAIT_TIME;

	return 0;
}


uint8 AtIpSendTcpRx(void)
{
	uint16 rx_len, mat_index;
	uint8 cmp_data[40];
	
	// ack back
	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		rx_len = g_gsm_struct.rx_counter;
		MemCpy(gsm_uart_rx_data, g_gsm_struct.rx_buf, rx_len);

		GsmUsartParaInit();

		TryParseRecvData(gsm_uart_rx_data, rx_len);
		
		mat_index = SubMatch(">", StrLen(">", 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_SEND_EXTRA;
			g_gen_flow.AckTimeOut = CIPSEND_FLAG_WAIT_TIME;
	
			return 0;
		}
	}

	// not time out
	if(g_gen_flow.AckTimeOut > 0)
	{
		return 2;
	}

	#ifdef GEN_DEBUG
	printf("at send tcp : 1st step time out\r\n");
	#endif

	return 1;
}

uint8 AtIpSendTcpExtraRx(void)
{
	uint16 rx_len, mat_index;
	uint8 cmp_data[40];
	
	// ack back
	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		rx_len = g_gsm_struct.rx_counter;
		MemCpy(gsm_uart_rx_data, g_gsm_struct.rx_buf, rx_len);

		GsmUsartParaInit();

		TryParseRecvData(gsm_uart_rx_data, rx_len);
		
		sprintf(cmp_data, (const char*)CIPSEND_EXTRA_OK, g_operation_ip_num, g_send_buffer.len, g_send_buffer.len);
		
		mat_index = SubMatch(cmp_data, StrLen(cmp_data, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_DELAY_NEXT_CMD;
			g_gen_flow.AckTimeOut = NEXT_AT_CMD_DELAY;
	
			return 0;
		}

		mat_index = SubMatch(OK_ACK, StrLen(OK_ACK, 0), gsm_uart_rx_data, rx_len);
		if(mat_index > 0)
		{
			return 2;
		}
	}

	// not time out
	if(g_gen_flow.AckTimeOut > 0)
	{
		return 2;
	}

	#ifdef GEN_DEBUG
	printf("at send tcp : extra time out\r\n");
	#endif

	return 1;
}

uint8 FlowAtIpSendTcp(void)
{

	switch(g_gen_flow.Flow)
	{
		case AT_RUN_STATUS_SEND_CMD:
			return AtIpSendTcpTx();

		case AT_RUN_STATUS_WAIT_ACK:
			return AtIpSendTcpRx();

		case AT_RUN_STATUS_SEND_EXTRA:
			return AtIpSendTcpExtraTx();

		case AT_RUN_STATUS_WAIT_EXTRA_ACK:
			return AtIpSendTcpExtraRx();

		case AT_RUN_STATUS_DELAY_NEXT_CMD:
			return AtDelay();
	}
}

uint8 FlowPowerReset(void)
{
	switch(g_gen_flow.Flow)
	{
		case AT_POWER_ON_STATUS_OFF:			
			OFF_MODEM_PWR();	///先关机
			g_gen_flow.Delay = 3000;
			g_gen_flow.Flow = AT_POWER_ON_STATUS_DELAY;
			break;

		case AT_POWER_ON_STATUS_DELAY:
			if(g_gen_flow.Delay == 0)
			{
				return 0;
			}
			else
			{
				return 2;
			}
			
			break;
	}
}

// 网络服务相关函数 end

void GsmUsartInit(u32 uart_bpr,uint8 data_bits,uint8 stop_bits,uint8 parity_check)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	//串口对应引脚复用映射
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3); //GPIOB10复用为USART3
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); //GPIOB11复用为USART3
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
			
	USART_InitStructure.USART_BaudRate = uart_bpr;
	USART_InitStructure.USART_WordLength = data_bits;
	USART_InitStructure.USART_StopBits = stop_bits;
	USART_InitStructure.USART_Parity = parity_check;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART3, ENABLE);
}

void USART3_IRQHandler(void)///GSM
{
	#ifdef UART3_BRIDDGE_UART1
	uint8 byte;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		byte = USART_ReceiveData(USART3);
		USART_SendData(USART1, byte);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	
	}
	
	#else
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{	
		if(g_gsm_struct.rx_counter >= GSM_MAX_BUF_LEN)
		{
			g_gsm_struct.rx_counter -= 1;
		}
		g_gsm_struct.rx_buf[g_gsm_struct.rx_counter++] = USART_ReceiveData(USART3);
		g_gsm_struct.rx_timeout_decounter = 3;
	}
	#endif
}

static void GsmUsartFixedLenSend(uint8 data[],uint16 len) 
{
	uint16 i;
	
	for(i=0;i<len;i++)
	{
		USART_SendData(USART3,data[i]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == RESET);
	}
}

void GsmUsartParaInit(void)
{
	uint16 i;
	#if 0
	printf("**");
    for(i=0; i < g_gsm_struct.rx_counter; i++)
    {
	    //printf("%02X",gsm_uart_rx_data[ii]);
		printf("%c",gsm_uart_rx_data[i]);
    }
	printf("##\r\n");
	#endif
	
	g_gsm_struct.rx_counter = 0;
	g_gsm_struct.rx_timeout_decounter = 0;
	g_gsm_struct.rx_done_flag = INVALID_VAL_55;
}

void GsmUartIsRxDone(void)
{
	if(g_gsm_struct.rx_timeout_decounter > 0)
	{
		g_gsm_struct.rx_timeout_decounter -= 1;
	}
	
	if((g_gsm_struct.rx_timeout_decounter == 0)&&(g_gsm_struct.rx_counter > 0))
	{
		g_gsm_struct.rx_done_flag = VALID_VAL_AA;
	}
}

#if 0
void GsmSendAtCmd(uint8 cmd_index,uint8 app_data[],uint8 app_len,uint8 mat_data[],uint8 mat_len)///附加的数据及长度,匹配的数据及长度
{
	uint8 i,ack_flag = FALSE;
	uint16 j,rx_len=0,mat_index = 0;
    uint16 ii = 0;
	
	//g_at_cmd_struct[cmd_index].exe_flag = EXE_NO;
	
	for(i=0;i<g_at_cmd_struct[cmd_index].tx_max_count;i++)
	{
		GsmUsartParaInit();
		GsmUsartFixedLenSend(g_at_cmd_struct[cmd_index].cmd_text,StrLen(g_at_cmd_struct[cmd_index].cmd_text,0));
		if(app_len > 0)
		{
			GsmUsartFixedLenSend(app_data,app_len);
		}
		
		#ifdef GEN_DEBUG
			printf((char*)g_at_cmd_struct[cmd_index].cmd_text);
			if(app_len > 0)
			{
                for(ii=0; ii < app_len; ii++)
                {
				    printf("%02X",app_data[ii]);
                }
			}
		#endif

		// wait recv
		for(j=0;j<g_at_cmd_struct[cmd_index].max_ms_wait/20;j++)
		{
			delay_ms(20);
			GsmUartIsRxDone();
			if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
			{
				rx_len = g_gsm_struct.rx_counter;
				MemCpy(gsm_uart_rx_data,g_gsm_struct.rx_buf,rx_len);
				
				#ifdef GEN_DEBUG
                    for(ii=0; ii < rx_len; ii++)
                    {
					    //printf("%02X",gsm_uart_rx_data[ii]);
						printf("%c",gsm_uart_rx_data[ii]);
                    }
				#endif
				
				mat_index = SubMatch(mat_data,mat_len,gsm_uart_rx_data,rx_len);
				if(mat_index > 0)
				{
					ack_flag = TRUE;
					break;
				}
			}
		}
		if(ack_flag)
		{
			break;
		}
	}
	
	//(g_at_cmd_struct[cmd_index].fun)(gsm_uart_rx_data,rx_len,ack_flag);
}
#endif

void GsmPowerInit(void)
{    
	GPIO_InitTypeDef GPIO_InitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOC, ENABLE);
    //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
						   
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;		///CT_GSMMD,4V电源输出控制 
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;  //推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL; 
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //高速GPIO
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOD, GPIO_Pin_8); 		//gsm_pwr
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;	///CT_3G，GSM关机控制引脚
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;  //推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL; 
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //高速GPIO
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOC, GPIO_Pin_0); 		//gsm en
/*
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;		///3G_RESET,复位
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;  //推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL; 
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //高速GPIO
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	//GPIO_ResetBits(GPIOF, GPIO_Pin_8); 		//no reset
*/
}

// 潍柒接口定义-internet

static void ResetProcessStatus(void)
{
	g_operation_ip_num = 0;

	g_gen_flow.Business = PROCESS_STATUS_RECV;
}

static void NetInfoInit(void)
{
	int i;

	for(i=0; i < SERVER_MAX_NUM; i++)
	{
		RamZero((uint8*)&g_server_info[i], sizeof(SERVER_INFO_STRUCT));
		g_server_info[i].server_status = SERVER_STATUS_CONNECT_FAILED;
		g_server_info[i].send_status = SEND_STATUS_READY;
	}

	ResetProcessStatus();

	g_gen_flow.Business = PROCESS_STATUS_INIT;
	g_gen_flow.Ope = AT_POWER_ON_INDEX;
	g_gen_flow.Flow = AT_POWER_ON_STATUS_ON;
	g_gen_flow.TryTime = 0;

	g_device_status = DEVICE_STATUS_INIT;
	g_init_status = GEN_INIT_ING;
	g_net_link_status = NET_LINK_STATUS_CONNECT_FAILED;
	
	RamZero((uint8*)&g_send_buffer, sizeof(BUFFER_STRUCT));
}

// 潍柒需要的接口 ("218.94.153.146", 9904)

/* 
 * uint8 GenInit(void)
 *
 * Param:
 *
 * return: 0:成功  1:失败  2:无SIM卡  3: 模块故障
 *
 * 说明: 返回成功后再做其他操作
 */

uint8 GenInit(void)
{
	// check status if can do this
	if(g_gen_flow.Business != PROCESS_STATUS_RECV)
	{
		return 1;
	}

	TIM3IntInit(10-1,8400-1);	//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数10次为1ms
	
	GsmPowerInit();
	
	GsmUsartInit(115200,0,0,0);

	NetInfoInit();
	
	return 0;
}

uint8 GenInitProcess(void)
{
	uint8 res;

	switch(g_gen_flow.Ope)
	{
		// do power on
		case AT_POWER_ON_INDEX:
			if(FlowPowerOn() == 0)
			{
				g_gen_flow.Ope = AT_INDEX;
				g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;
			}
			break;
			
		case AT_INDEX:
			res = FlowAt();				
			if(res == 1)
			{
				#ifdef GEN_DEBUG
				printf("AT_INDEX failed\r\n");
				#endif

				g_gen_flow.TryTime++;

				if(g_gen_flow.TryTime < g_at_cmd_struct[g_gen_flow.Ope].tx_max_count)
				{
					g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;
				}
				else
				{
					g_gen_flow.TryTime = 0;

					g_device_status = DEVICE_STATUS_INIT;
					g_init_status = GEN_INIT_NO;
					ResetProcessStatus();
				}
				
				return 1;
			}
			
			if(res ==  0 && g_gen_flow.Flow == AT_RUN_STATUS_DELAY_NEXT_CMD)
			{
				g_gen_flow.Ope = AT_E0_INDEX;
				g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;
				
				#ifdef GEN_DEBUG
				printf("AT_INDEX ok\r\n");
				#endif

				g_gen_flow.TryTime = 0;
			}
			break;

		case AT_E0_INDEX:
			res = FlowAtE0();
			if(res == 1)
			{
				#ifdef GEN_DEBUG
				printf("AT_E0_INDEX failed\r\n");
				#endif

				g_gen_flow.TryTime++;

				if(g_gen_flow.TryTime < g_at_cmd_struct[g_gen_flow.Ope].tx_max_count)
				{
					g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;
				}
				else
				{
					g_gen_flow.TryTime = 0;

					g_device_status = DEVICE_STATUS_INIT;
					g_init_status = GEN_INIT_NO;
					ResetProcessStatus();
				}
				
				return 1;
			}
			
			if(res ==  0 && g_gen_flow.Flow == AT_RUN_STATUS_DELAY_NEXT_CMD)
			{
				g_gen_flow.Ope = AT_IMEI_INDEX;
				g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;

				#ifdef GEN_DEBUG
				printf("AT_E0_INDEX ok\r\n");
				#endif

				g_gen_flow.TryTime = 0;
			}			
			break;

		case AT_IMEI_INDEX:
			res = FlowAtImei();
			if(res == 1)
			{
				#ifdef GEN_DEBUG
				printf("AT_IMEI_INDEX failed\r\n");
				#endif

				g_gen_flow.TryTime++;

				if(g_gen_flow.TryTime < g_at_cmd_struct[g_gen_flow.Ope].tx_max_count)
				{
					g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;
				}
				else
				{
					g_gen_flow.TryTime = 0;

					g_device_status = DEVICE_STATUS_INIT;
					g_init_status = GEN_INIT_NO;
					ResetProcessStatus();
				}
				
				return 1;
			}
			
			if(res ==  0 && g_gen_flow.Flow == AT_RUN_STATUS_DELAY_NEXT_CMD)
			{
				g_gen_flow.Ope = AT_CSQ_INDEX;
				g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;

				#ifdef GEN_DEBUG
				printf("AT_IMEI_INDEX ok\r\n");
				#endif

				g_gen_flow.TryTime = 0;
			}	
			break;

		case AT_CSQ_INDEX:
			res = FlowAtCsq();
			if(res == 1)
			{
				#ifdef GEN_DEBUG
				printf("AT_CSQ_INDEX failed\r\n");
				#endif

				g_gen_flow.TryTime++;

				if(g_gen_flow.TryTime < g_at_cmd_struct[g_gen_flow.Ope].tx_max_count)
				{
					g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;
				}
				else
				{
					g_gen_flow.TryTime = 0;

					g_device_status = DEVICE_STATUS_INIT;
					g_init_status = GEN_INIT_NO;
					ResetProcessStatus();
				}
				
				return 1;
			}
			
			if(res ==  0 && g_gen_flow.Flow == AT_RUN_STATUS_DELAY_NEXT_CMD)
			{
				g_gen_flow.Ope = AT_CREG_INDEX;
				g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;

				#ifdef GEN_DEBUG
				printf("AT_CSQ_INDEX ok\r\n");
				#endif

				g_gen_flow.TryTime = 0;
			}	
			break;

		case AT_CREG_INDEX:
			res = FlowAtCreg();
			if(res == 1)
			{
				#ifdef GEN_DEBUG
				printf("AT_CREG_INDEX failed\r\n");
				#endif

				g_gen_flow.TryTime++;

				if(g_gen_flow.TryTime < g_at_cmd_struct[g_gen_flow.Ope].tx_max_count)
				{
					g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;
				}
				else
				{
					g_gen_flow.TryTime = 0;

					g_device_status = DEVICE_STATUS_NO_SIM_CARD;
					g_init_status = GEN_INIT_NO_SIM_CARD;
					ResetProcessStatus();
				}
				
				return 1;
			}
			
			if(res ==  0 && g_gen_flow.Flow == AT_RUN_STATUS_DELAY_NEXT_CMD)
			{
				g_gen_flow.Ope = AT_CGREG_INDEX;
				g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;

				#ifdef GEN_DEBUG
				printf("AT_CREG_INDEX ok\r\n");
				#endif

				g_gen_flow.TryTime = 0;
			}	
			break;

		case AT_CGREG_INDEX:
			res = FlowAtCgreg();
			if(res == 1)
			{
				#ifdef GEN_DEBUG
				printf("AT_CGREG_INDEX failed\r\n");
				#endif

				g_gen_flow.TryTime++;

				if(g_gen_flow.TryTime < g_at_cmd_struct[g_gen_flow.Ope].tx_max_count)
				{
					g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;
				}
				else
				{
					g_gen_flow.TryTime = 0;

					g_device_status = DEVICE_STATUS_NO_SIM_CARD;
					g_init_status = GEN_INIT_NO_SIM_CARD;
					ResetProcessStatus();
				}
				
				return 1;
			}
			
			if(res ==  0 && g_gen_flow.Flow == AT_RUN_STATUS_DELAY_NEXT_CMD)
			{
				g_gen_flow.Ope = AT_CSOCKAUTH_INDEX;
				g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;

				#ifdef GEN_DEBUG
				printf("AT_CGREG_INDEX ok\r\n");
				#endif

				g_gen_flow.TryTime = 0;
			}	
			break;

		case AT_CSOCKAUTH_INDEX:
			res = FlowAtSockAuth();
			if(res == 1)
			{
				g_device_status = DEVICE_STATUS_INIT;
				g_init_status = GEN_INIT_NO;
				ResetProcessStatus();

				#ifdef GEN_DEBUG
				printf("AT_CSOCKAUTH_INDEX failed\r\n");
				#endif
				
				return 1;
			}
			
			if(res ==  0 && g_gen_flow.Flow == AT_RUN_STATUS_DELAY_NEXT_CMD)
			{
				g_gen_flow.Ope = AT_CIPMODE_INDEX;
				g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;

				#ifdef GEN_DEBUG
				printf("AT_CSOCKAUTH_INDEX ok\r\n");
				#endif
			}	
			break;

		case AT_CIPMODE_INDEX:
			res = FlowAtIpMode();
			if(res == 1)
			{
				g_device_status = DEVICE_STATUS_INIT;
				g_init_status = GEN_INIT_NO;
				ResetProcessStatus();

				#ifdef GEN_DEBUG
				printf("AT_CIPMODE_INDEX failed\r\n");
				#endif
				
				return 1;
			}
			
			if(res ==  0 && g_gen_flow.Flow == AT_RUN_STATUS_DELAY_NEXT_CMD)
			{
				g_gen_flow.Ope = AT_CIPMODE_INDEX;
				g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;

				#ifdef GEN_DEBUG
				printf("AT_CIPMODE_INDEX ok\r\n");
				#endif

				g_device_status = DEVICE_STATUS_NET_NOT_OPEN;
				g_init_status = GEN_INIT_OK;
				ResetProcessStatus();

				return 0;
			}				
	}
}

 /* 
  * uint8 GenGetInitStatus(void)
  *
  * Param:
  *
  * return: 0:成功	1:失败	2: 正在初始化  3: 无SIM卡
  *
  */


uint8 GenGetInitStatus(void)
{	
	return g_init_status;
}


/* 
 * uint8 GenGetNetLinkStatus(void)
 *
 * Param:
 *
 * return: 0:网络连接正常  1:正在连接  2: 无网络连接
 *
 * 说明: 返回成功后再做其他操作
 */
 
uint8 GenGetNetLinkStatus(void)
{

    return g_net_link_status;
}

/* 
 * uint8 GenConnect(void)
 *
 *
 * return: 0:成功  1:失败
 *
 */

uint8 GenConnect(void)
{
	// check status if can do this
	if(g_device_status != DEVICE_STATUS_NET_NOT_OPEN)
	{
		return 1;
	}
	
	if(g_gen_flow.Business != PROCESS_STATUS_RECV)
	{
		return 1;
	}
	
	g_gen_flow.Business = PROCESS_STATUS_CONNECT_NET;
	g_gen_flow.Ope = AT_NETOPEN_INDEX;
	g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;
	g_gen_flow.TryTime = 0;

	g_net_link_status = NET_LINK_STATUS_CONNECTING;
	
	return 0;
}

uint8 GenConnectProcess(void)
{
	uint8 res;
	
	res = FlowAtNetOpen();
	
	if(res == 1)
	{
		g_gen_flow.TryTime++;

		if(g_gen_flow.TryTime < g_at_cmd_struct[g_gen_flow.Ope].tx_max_count)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;
		}
		else
		{
			g_gen_flow.TryTime = 0;
			
			g_device_status = DEVICE_STATUS_NET_NOT_OPEN;
			g_net_link_status = NET_LINK_STATUS_CONNECT_FAILED;

			ResetProcessStatus();
		}

		#ifdef GEN_DEBUG
		printf("Net open error\r\n");
		#endif

		return 1;
	}
	
	if(res ==  0 && g_gen_flow.Flow == AT_RUN_STATUS_DELAY_NEXT_CMD)
	{
		g_device_status = DEVICE_STATUS_NET_OK;
		g_net_link_status = NET_LINK_STATUS_CONNECT_OK;

		ResetProcessStatus();

		return 0;
	}
}


/* 
 * uint8 GenDisconnectNet(void)
 * 断开网络
 *
 * return: 0:成功  1:失败
 *
 */

uint8 GenDisconnectNet(void)
{
	if(g_device_status == DEVICE_STATUS_NO_SIM_CARD)
	{
		return 1;
	}
	
	// check status if can do this
	if(g_gen_flow.Business != PROCESS_STATUS_RECV)
	{
		return 1;
	}
	
	g_gen_flow.Business = PROCESS_STATUS_DISCONNECT_NET;
	g_gen_flow.Ope = AT_NETCLOSE_INDEX;
	g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;
	g_gen_flow.TryTime = 0;
	
	return 0;
}

uint8 GenDisconnectNetProcess(void)
{
	uint8 res;
	
	if(g_net_link_status == NET_LINK_STATUS_CONNECT_OK)
	{
		res = FlowAtNetClose();
	
		if(res == 1)
		{
			g_gen_flow.TryTime++;

			if(g_gen_flow.TryTime < g_at_cmd_struct[g_gen_flow.Ope].tx_max_count)
			{
				g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;
			}
			else
			{
				g_device_status = DEVICE_STATUS_NET_NOT_OPEN;
				g_net_link_status = NET_LINK_STATUS_CONNECT_FAILED;

				g_gen_flow.TryTime = 0;
				
				ResetProcessStatus();
			}

			return 1;
		}
		
		if(res ==  0 && g_gen_flow.Flow == AT_RUN_STATUS_DELAY_NEXT_CMD)
		{
			g_device_status = DEVICE_STATUS_NET_NOT_OPEN;
			g_net_link_status = NET_LINK_STATUS_CONNECT_FAILED;

			g_gen_flow.TryTime = 0;

			ResetProcessStatus();

			return 0;
		}
	}
}

/* 
 * uint8 GenGetSerLinkStatus(uint8 IPnum)
 * 
 * Param:
 * IPnum:  1: 服务器1    2:服务器2
 *
 * return: 0:服务器连接正常  1:正在连接  2:无法与服务器建立连接
 *
 */
 
uint8 GenGetSerLinkStatus(uint8 IPnum)
{
	if(IPnum != MASTER_IP && IPnum != SLAVE_IP)
	{
		return 4;
	}
	
	return g_server_info[IPnum-1].server_status;
}

/* 
 * uint8 GenGetSendDataStatus(uint8 IPnum)
 * 
 * Param:
 * IPnum:  1: 服务器1    2:服务器2
 *
 * return: 0:正在发送  1:数据已发送完成，空闲  2: IPnum error
 *
 */
 
uint8 GenGetSendDataStatus(uint8 IPnum)
{
	if(IPnum != MASTER_IP && IPnum != SLAVE_IP)
	{
		return 2;
	}
	
	return g_server_info[IPnum-1].send_status;
}


/* 
 * uint8 GenConnectSer(ip_st *Addr, uint8 IPnum)
 * 
 * Param:
 * Addr: 服务器地址
 * IPnum:  1: 服务器1    2:服务器2
 *
 * return: 0:成功  1:失败
 *
 */

uint8 GenConnectSer(ip_st *Addr, uint8 IPnum)
{
	// check status if can do this
	if(g_device_status == DEVICE_STATUS_NO_SIM_CARD)
	{
		return 1;
	}
	
	if(g_gen_flow.Business != PROCESS_STATUS_RECV)
	{
		return 1;
	}

	if(IPnum != MASTER_IP && IPnum != SLAVE_IP)
	{
		return 1;
	}

	if(Addr == NULL || StrLen(Addr->ip, 0) == 0 || StrLen(Addr->port, 0) == 0)
	{
		return 1;
	}

	if(g_server_info[IPnum-1].server_status == SERVER_STATUS_CONNECT_OK)
	{
		return 1;
	}

	g_server_info[IPnum-1].ip_num = IPnum;
	MemCpy((uint8*)&(g_server_info[IPnum-1].server_addr.ip[0]), (uint8*)&(Addr->ip[0]), NET_IP_LEN);
	MemCpy((uint8*)&(g_server_info[IPnum-1].server_addr.port[0]), (uint8*)&(Addr->port[0]), NET_PORT_LEN);
	g_server_info[IPnum-1].server_status = SERVER_STATUS_CONNECTING;

	g_operation_ip_num = IPnum;

	g_gen_flow.Business = PROCESS_STATUS_CONNECT_SERVER;
	g_gen_flow.Ope = AT_CIPOPEN_TCP_INDEX;
	g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;
	g_gen_flow.TryTime = 0;
	
	return 0;
}

uint8 GenConnectSerProcess(void)
{
	uint8 res;

	if(g_operation_ip_num != MASTER_IP && g_operation_ip_num != SLAVE_IP)
	{
		ResetProcessStatus();
		return 1;
	}

	if(g_server_info[g_operation_ip_num-1].server_status != SERVER_STATUS_CONNECTING)
	{
		ResetProcessStatus();
		return 1;
	}

	res = FlowAtIpOpenTcp();
	
	if(res == 1)
	{
		g_gen_flow.TryTime++;
		
		if(g_gen_flow.TryTime < g_at_cmd_struct[g_gen_flow.Ope].tx_max_count)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;
		}
		else
		{
			g_server_info[g_operation_ip_num-1].server_status = SERVER_STATUS_CONNECT_FAILED;
			
			g_gen_flow.TryTime = 0;

			ResetProcessStatus();
		}
		
		#ifdef GEN_DEBUG
		printf("connect failed: open tcp\r\n");
		#endif
		
		return 1;
	}
	
	if(res ==  0 && g_gen_flow.Flow == AT_RUN_STATUS_DELAY_NEXT_CMD)
	{
		g_server_info[g_operation_ip_num-1].server_status = SERVER_STATUS_CONNECT_OK;

		g_operation_ip_num = 0;

		g_gen_flow.TryTime = 0;
		
		#ifdef GEN_DEBUG
		printf("connect ok: open tcp\r\n");
		#endif
		
		ResetProcessStatus();

		return 0;
	}
}

/* 
 * uint8 GenDisconnectSer(uint8 IPnum)
 * 
 * Param:
 * IPnum:  1: 服务器1    2:服务器2
 *
 * return: 0:成功  1:失败
 *
 */

uint8 GenDisconnectSer(uint8 IPnum)
{
	// check status if can do this
	if(g_device_status == DEVICE_STATUS_NO_SIM_CARD)
	{
		return 1;
	}
	
	if(g_gen_flow.Business != PROCESS_STATUS_RECV)
	{
		return 1;
	}

	if(IPnum != MASTER_IP && IPnum != SLAVE_IP)
	{
		return 1;
	}

	g_operation_ip_num = IPnum;
	
	g_gen_flow.Business = PROCESS_STATUS_DISCONNECT_SERVER;
	g_gen_flow.Ope = AT_CIPCLOSE_INDEX;
	g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;
	
	return 0;
}

uint8 GenDisconnectSerProcess(void)
{
	uint8 res;

	if(g_operation_ip_num != MASTER_IP && g_operation_ip_num != SLAVE_IP)
	{
		ResetProcessStatus();
		return 1;
	}

	if(g_server_info[g_operation_ip_num-1].server_status != SERVER_STATUS_CONNECT_OK 
		&& g_server_info[g_operation_ip_num-1].server_status != SERVER_STATUS_CONNECT_FAILED)
	{
		ResetProcessStatus();
		return 1;
	}

	res = FlowAtIpClose();
	
	if(res == 1)
	{
		ResetProcessStatus();
		
		return 1;
	}
	
	if(res ==  0 && g_gen_flow.Flow == AT_RUN_STATUS_DELAY_NEXT_CMD)
	{
		g_server_info[g_operation_ip_num-1].server_status = SERVER_STATUS_CONNECT_FAILED;

		g_operation_ip_num = 0;
		
		ResetProcessStatus();

		return 0;
	}
}

/* 
 * uint8 GenSendData(uint8 IPnum, uint8 *data, uint16 len, void (*fp)(uint8 result))
 *
 * Param:
 * IPnum: 服务器编号  1: 服务器1  2: 服务器2
 * data: 发送的数据
 * len: 发送数据的长度
 * fp: 数据发送失败时调用的函数
 *
 * return: 0:成功  1:失败
 *
 */

uint8 GenSendData(uint8 IPnum, uint8 *data, uint16 len)
{
	// check status if can do this
	if(g_device_status == DEVICE_STATUS_NO_SIM_CARD)
	{
		return 1;
	}
	
	if(g_gen_flow.Business != PROCESS_STATUS_RECV)
	{
		return 1;
	}

	if(IPnum != MASTER_IP && IPnum != SLAVE_IP)
	{
		return 1;
	}

	if(g_server_info[IPnum-1].server_status != SERVER_STATUS_CONNECT_OK)
	{
		return 1;
	}

	if(data == NULL || len == 0)
	{
		return 1;
	}

	MemCpy(g_send_buffer.data, data, len);
	g_send_buffer.len = len;

	g_operation_ip_num = IPnum;
	
	g_gen_flow.Business = PROCESS_STATUS_SEND;
	g_gen_flow.Ope = AT_CIPSEND_TCP_INDEX;
	g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;

	g_server_info[IPnum-1].send_status = SEND_STATUS_BUSY;
	
	return 0;
}

uint8 GenSendDataProcess(void)
{
	uint8 res;

	if(g_operation_ip_num != MASTER_IP && g_operation_ip_num != SLAVE_IP)
	{
		g_server_info[g_operation_ip_num-1].send_status = SEND_STATUS_READY;
		ResetProcessStatus();
		return 1;
	}
	
	if(g_send_buffer.data == NULL || g_send_buffer.len == 0)
	{
		g_server_info[g_operation_ip_num-1].send_status = SEND_STATUS_READY;
		ResetProcessStatus();
		return 1;
	}
	
	if(g_server_info[g_operation_ip_num-1].server_status != SERVER_STATUS_CONNECT_OK)
	{
		g_server_info[g_operation_ip_num-1].send_status = SEND_STATUS_READY;
		ResetProcessStatus();
		return 1;
	}

	res = FlowAtIpSendTcp();
	
	if(res == 1)
	{
		g_gen_flow.TryTime++;

		if(g_gen_flow.TryTime < g_at_cmd_struct[g_gen_flow.Ope].tx_max_count)
		{
			g_gen_flow.Flow = AT_RUN_STATUS_SEND_CMD;
		}
		else
		{
			g_device_status = DEVICE_STATUS_NET_NOT_OPEN;
			g_gen_flow.TryTime = 0;

			g_server_info[g_operation_ip_num-1].server_status = SERVER_STATUS_CONNECT_FAILED;
			g_server_info[g_operation_ip_num-1].send_status = SEND_STATUS_READY;
		
			ResetProcessStatus();
		}
		
        #ifdef GEN_DEBUG
        printf("tcp send ack error %u times\r\n", g_gen_flow.TryTime);
        #endif
		
		return 1;
	}
	
	if(res ==  0 && g_gen_flow.Flow == AT_RUN_STATUS_DELAY_NEXT_CMD)
	{
		g_device_status = DEVICE_STATUS_NET_OK;

		g_gen_flow.TryTime = 0;

		g_send_buffer.len = 0;

		g_server_info[g_operation_ip_num-1].send_status = SEND_STATUS_READY;
		
		ResetProcessStatus();

		return 0;
	}
}


/* 
 * uint8 GenReset(void)
 *
 * Param:
 *
 * return: 0:成功  1:失败  2:初始化未完成
 *
 */

uint8 GenReset(void)
{
	// check status if can do this
	if(g_gen_flow.Business == PROCESS_STATUS_INIT)
	{
		return 2;
	}
	
	if(g_gen_flow.Business != PROCESS_STATUS_RECV)
	{
		return 1;
	}
	
	g_gen_flow.Business = PROCESS_STATUS_RESET;

	g_gen_flow.Ope = AT_POWER_RESET_INDEX;
	g_gen_flow.Flow = AT_POWER_ON_STATUS_OFF;

	g_device_status = DEVICE_STATUS_RESET;
	
	return 0;
}

uint8 GenResetProcess(void)
{
	if(FlowPowerReset() == 0)
	{
		g_device_status = DEVICE_STATUS_INIT;

		ResetProcessStatus();

		return 0;
	}    
}

/* 
 * uint8 GenResetStart(void)
 *
 * Param:
 *
 * return: 0:成功  1:失败
 *
 */

uint8 GenResetStart(void)
{
	OFF_MODEM_PWR();	///断电 关机
	
	return 0;
}

/* 
 * uint8 GenResetComplete(void)
 *
 * Param:
 *
 * return: 0:成功  1:失败
 *
 */

uint8 GenResetComplete(void)
{
	ON_MODEM_PWR();		///上电 开机
	
	return 0;
}

/* 
 * void GenGetIMEI(uint8 *Imei, uint16 Len)
 *
 * Param:
 * Imei: 存储缓存
 * Len: 数据长度
 *
 */

void GenGetIMEI(uint8 *Imei, uint16 Len)
{
	uint8 i;
	
	if(Len < GSM_IMEI_LEN || Imei == NULL)
	{
		return;
	}

	for(i=0; i < GSM_IMEI_LEN; i++)
	{
		*Imei = g_gsm_struct.imei[i];
		Imei++;
	}
}


static uint8 GenTryRxDone(void)
{
	uint16 rx_bytes;
	uint16 i, mat_index, mat_end, recv_data_len, mat_end2;
	IP_NUM ip_num = 0;

	if(g_gsm_struct.rx_done_flag == VALID_VAL_AA)
	{
		if(g_gsm_struct.rx_counter <= GSM_MAX_BUF_LEN)
		{
			rx_bytes = g_gsm_struct.rx_counter;
			MemCpy(gen_rx_data, g_gsm_struct.rx_buf, rx_bytes);
			
			GsmUsartParaInit();

			TryParseRecvData(gen_rx_data, rx_bytes);

#if 0
			// check ip to ipnum
			mat_index = SubMatch(RECV_FROM_ACK, StrLen(RECV_FROM_ACK, 0), gen_rx_data, rx_bytes);

			if(mat_index > 0)
			{
				mat_end = FindFirstChar(mat_index, ':', gen_rx_data, rx_bytes);

				mat_end2 = FindFirstCrLf(mat_end+1, gen_rx_data, rx_bytes);
				#ifdef GEN_DEBUG
				printf("mat_index=%u, mat_end=%u, mat_end2=%u\r\n", mat_index, mat_end, mat_end2);
				#endif
				if(mat_end > 0 && mat_end2 > 0)
				{
					for(i=0; i < SERVER_MAX_NUM; i++)
					{
						if(MemCmp(gen_rx_data + mat_index, g_server_info[i].server_addr.ip, mat_end - mat_index) == TRUE
							&& MemCmp(gen_rx_data + mat_end + 1, g_server_info[i].server_addr.port, mat_end2 - mat_end - 1) == TRUE)
						{
							ip_num = i+1;
							break;
						}
					}
				}
			}

			if(ip_num == 0)
			{
				return 1;
			}

			#ifdef GEN_DEBUG
			printf("IP num = %u\r\n", ip_num);
			#endif
			
			// has recv data
			mat_index = SubMatch(RECV_IPD_ACK, StrLen(RECV_IPD_ACK, 0), gen_rx_data, rx_bytes);
			
			if(mat_index > 0)
			{
				mat_end = FindFirstCrLf(mat_index, gen_rx_data, rx_bytes);

				if(mat_end > 0)
				{
					recv_data_len = Str2Uint16(gen_rx_data + mat_index, mat_end - mat_index);
				#ifdef GEN_DEBUG
					printf("recv len = %u\r\n", recv_data_len);
				#endif
					if(recv_data_len > 0)
					{
						MemCpy(gen_rx_payload_data, gen_rx_data + mat_index + (mat_end - mat_index) + 2, recv_data_len);

						GenRecData(ip_num, gen_rx_payload_data, recv_data_len);
					}
				}
			}

			#if 0
				for(i=0; i < rx_bytes; i++)
				{
					printf("%02X", gen_rx_data[i]);
				}
			#endif
#endif

			return 0;
		}
	}
	
	return 1;
}

void GenProcess(void)
{
	switch(g_gen_flow.Business)
	{
		case PROCESS_STATUS_INIT:
			//printf("PROCESS_STATUS_INIT process\r\n");
			GenInitProcess();
			break;

		case PROCESS_STATUS_CONNECT_NET:
			//printf("PROCESS_STATUS_CONNECT_NET process\r\n");
			GenConnectProcess();
			break;

		case PROCESS_STATUS_CONNECT_SERVER:
			//printf("PROCESS_STATUS_CONNECT_SERVER process\r\n");
			GenConnectSerProcess();
			break;

		case PROCESS_STATUS_SEND:
			//printf("PROCESS_STATUS_SEND process\r\n");
			GenSendDataProcess();
			break;

		case PROCESS_STATUS_RECV:
			//printf("PROCESS_STATUS_RECV process\r\n");
			GenTryRxDone();
			break;

		case PROCESS_STATUS_DISCONNECT_SERVER:
			//printf("PROCESS_STATUS_DISCONNECT_SERVER process\r\n");
			GenDisconnectSerProcess();
			break;

		case PROCESS_STATUS_DISCONNECT_NET:
			//printf("PROCESS_STATUS_DISCONNECT_NET process\r\n");
			GenDisconnectNetProcess();
			break;

		case PROCESS_STATUS_RESET:
			//printf("PROCESS_STATUS_RESET process\r\n");
			GenResetProcess();
			break;

		default:
			break;
			
	}
}
#endif

#if 1
//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!
void TIM3IntInit(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

void TIM3IntDisable(void)
{
	TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,DISABLE); //使能定时器3
}

//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
		g_1ms_count++;

		if(g_gen_flow.AckTimeOut > 0)
		{
			g_gen_flow.AckTimeOut--;
		}

		if(g_gen_flow.Delay > 0)
		{
			g_gen_flow.Delay--;
		}

		if(g_check_uart_frame_interval > 0)
		{
			g_check_uart_frame_interval--;
		}
		else
		{
			GsmUartIsRxDone();
			g_check_uart_frame_interval = CHECK_UART_FRAME_INTERVAL;
		}
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}
#endif
