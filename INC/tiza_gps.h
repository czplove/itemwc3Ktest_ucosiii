#ifndef __TIZA_GPS_H
#define __TIZA_GPS_H
	
	#include "tiza_include.h"

	#include "tiza_rtc.h"
    
	#ifdef GPS_STRUCT_GLOBAL
		#define EXTERN_GPS
	#else
		#define EXTERN_GPS extern
	#endif


	// STATE 
	typedef enum{
		NSS_ANTOK=0,	// 天线正常
		NSS_ANTOPEN,	// 天线开路
		NSS_ANTSHORT,	// 天线短路
		NSS_FAULT		// 其他故障
	}EV_NSS_STS;
	
	typedef struct
	{
		uint8 Status;		// 定位状态  0: 定位    1:不定位
		uint8 PlanetNum;	// 卫星数量
		uint8 Longitude[5];	// 经度(118度52.9276分)	byte0: 01  byte1: 18  byte2: 52  byte3: 92  byte4: 76 (BCD码)
		uint8 EWhemisphere;	// 0: 东半球  1:西半球
		uint8 Latitude[4];	// 纬度(38度52.9276分)	byte0: 38  byte1: 52  byte2: 92  byte3: 76
		uint8 SNhemisphere;	// 0: 南半球  1:北半球
		RTC_ST Time;		// 转换后的东8区时间
		uint8 Altitude[4];	// 海拔高度((-)1238.3 米)  byte0: 0x2d 负值, 0x0 正值  byte1: 12  byte2: 38  byte3: 3(low 4bits)
		uint8 Speed[3];		// 速度(110.5 km/h) byte0: 01  byte1: 10  byte2: 5(low 4bits)
		uint8 Direction[3];	// 方向(270.1) 		byte0: 02  byte1: 70  byte2: 1(low 4bits)
	}NSS_INFO;


	EXTERN_GPS uint8 GetNssAntStatus(void);
	EXTERN_GPS uint8 NssInit(void);
	EXTERN_GPS uint8 GetCurrentPosition(NSS_INFO *NssInfo);
	EXTERN_GPS uint8 NssResetStart(void);
	EXTERN_GPS uint8 NssResetComplete(void);	// NssResetStart调用500ms后可调用NssResetComplete

#endif
