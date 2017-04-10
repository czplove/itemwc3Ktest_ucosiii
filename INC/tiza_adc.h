#ifndef __TIZA_ADC_H
#define __TIZA_ADC_H	

#include "tiza_include.h"

	#ifdef ADC_GLOBAL
		#define EXTERN_ADC
	#else
		#define EXTERN_ADC extern
	#endif

 							   
EXTERN_ADC void AdcInit(void); 						//ADC通道初始化
EXTERN_ADC uint16  ReadBatAdc(void); 				//获得电池通道的ADC值 
EXTERN_ADC float  ReadBatVoltage(void); 			//获得电池电压 单位:V

#endif
