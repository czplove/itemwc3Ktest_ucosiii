#ifndef _TIZA_MISC_H
#define _TIZA_MISC_H

	#include "tiza_include.h"
	
	#ifdef MISC_GLOBAL
		#define EXTERN_MISC
	#else
		#define EXTERN_MISC extern
	#endif

EXTERN_MISC void GetDeviceSerial(uint8 *DevSerial, uint8 Len);  //�豸ΨһID

#endif
