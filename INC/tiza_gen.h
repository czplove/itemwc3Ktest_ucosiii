#ifndef __TIZA_GEN_H
#define __TIZA_GEN_H
	
	#ifdef GEN_GLOBAL
		#define EXTERN_GEN
	#else
		#define EXTERN_GEN extern
	#endif
	

    // 潍柒接口定义-internet
	#define NET_IP_LEN					20
	#define NET_PORT_LEN				8
	typedef struct
	{
		char ip[NET_IP_LEN];           	// IP地址
		char port[NET_PORT_LEN];        // 端口
		
	}ip_st;///GPRS参数结构体

	typedef enum{
		NET_LINK_STATUS_CONNECT_OK = 0,
		NET_LINK_STATUS_CONNECTING,
		NET_LINK_STATUS_CONNECT_FAILED
	}NET_LINK_STATUS;
	
	typedef enum{
		SERVER_STATUS_CONNECT_OK = 0,
		SERVER_STATUS_CONNECTING,
		SERVER_STATUS_CONNECT_FAILED
	}SERVER_STATUS;

	typedef enum{
		SEND_STATUS_BUSY = 0,
		SEND_STATUS_READY,
	}SEND_STATUS;

	typedef enum{
		GEN_INIT_OK = 0,
		GEN_INIT_NO,
		GEN_INIT_ING,
		GEN_INIT_NO_SIM_CARD,
	}GEN_INIT_STATUS;

	typedef enum
    {
        MASTER_IP = 1,	// 主IP
        SLAVE_IP = 2	// 副IP
    }IP_NUM;

	
// 潍柒接口定义-internet
EXTERN_GEN uint8 GenInit(void);
EXTERN_GEN uint8 GenConnect(void);
EXTERN_GEN uint8 GenDisconnectNet(void);
EXTERN_GEN uint8 GenConnectSer(ip_st *Addr, uint8 IPnum);
EXTERN_GEN uint8 GenDisconnectSer(uint8 IPnum);
EXTERN_GEN uint8 GenSendData(uint8 IPnum, uint8 *data, uint16 len);

EXTERN_GEN uint8 GenReset(void);	//在状态机里做了断电延时的操作, 之后需要重新初始化

EXTERN_GEN uint8 GenResetStart(void);
EXTERN_GEN uint8 GenResetComplete(void);	// GenResetComplete在调用GenResetStart后3秒再调用

EXTERN_GEN uint8 GenGetInitStatus(void);
EXTERN_GEN uint8 GenGetNetLinkStatus(void);				// 返回 4 表示 没有SIM卡 或 卡无效
EXTERN_GEN uint8 GenGetSerLinkStatus(uint8 IPnum);		// 返回 4 表示 IPnum不正确
EXTERN_GEN uint8 GenGetSendDataStatus(uint8 IPnum);

EXTERN_GEN void GenProcess(void);	//创建一个任务循环运行此函数
EXTERN_GEN void GenGetIMEI(uint8 *Imei, uint16 Len);

#endif
