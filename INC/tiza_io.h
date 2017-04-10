#ifndef _TIZA_IO_H
#define _TIZA_IO_H

	#include "tiza_include.h"
	
	#ifdef IO_MOD_GLOBAL
		#define EXTERN_IO
	#else
		#define EXTERN_IO extern
	#endif

	typedef enum{
		LED1 = 1,	// red led
		LED2 = 2	// green led
	}LED_NO;

EXTERN_IO void IoInit(void);  //≥ı ºªØ

EXTERN_IO void LedDisp(uint8 LedNo, uint8 Disp);
EXTERN_IO uint8 GetTearDownStatus(void);
EXTERN_IO uint8 GetExtPowerStatus(void);

#endif
