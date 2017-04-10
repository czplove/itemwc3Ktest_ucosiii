#ifndef __TIZA_RS232_H
#define __TIZA_RS232_H

#include "tiza_include.h"

	#ifdef RS232_GLOBAL
		#define EXTERN_RS232
	#else
		#define EXTERN_RS232 extern
	#endif

	EXTERN_RS232 uint8 RS232OpenPort(uint32 bound);
	EXTERN_RS232 uint8 RS232ClosePort(void);
	EXTERN_RS232 uint8 RS232SendData(uint8 Data);
	
	/* Î«²ñ¶¨Òå void RS232RecData(uint8 *Data, uint8 Len) */
	
#endif

