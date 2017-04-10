#ifndef __TIZA_RS485_H
#define __TIZA_RS485_H

#include "tiza_include.h"

	#ifdef RS485_GLOBAL
		#define EXTERN_RS485
	#else
		#define EXTERN_RS485 extern
	#endif	
	
	
	EXTERN_RS485 uint8 RS485OpenPort(u32 bound);
	EXTERN_RS485 uint8 RS485ClosePort(void);
	EXTERN_RS485 uint8 RS485SendData(uint8 *data, uint16 len);

	// Î«²ñ¶¨Òå void RS485RecData(uint8 *data, uint16 len);
	
#endif

