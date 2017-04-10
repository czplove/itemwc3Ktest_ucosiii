#ifndef __TIZA_CAN_H
#define __TIZA_CAN_H

#include "tiza_include.h"

	#ifdef CAN_GLOBAL
		#define EXTERN_CAN
	#else
		#define EXTERN_CAN extern
	#endif

	EXTERN_CAN uint8 CanSendMessage(uint8 port,uint32 can_id,uint8 id_type,uint8* data,uint8 len);	//id_type:  1: ±ê×¼Ö¡  2: À©Õ¹Ö¡
	EXTERN_CAN void CanRecMessage(uint8 port,uint32 CanId,uint8* Dat);
	EXTERN_CAN uint8 CanClosePort(uint8 Port);
	EXTERN_CAN uint8 CanOpenPort(uint8 port,uint32 bps);
	
#endif

