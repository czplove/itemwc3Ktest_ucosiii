#ifndef __TIZA_WD_H
#define __TIZA_WD_H

#include "tiza_include.h"

	#ifdef WATCH_DOG_GLOBAL
		#define EXTERN_WATCH_DOG
	#else
		#define EXTERN_WATCH_DOG extern
	#endif

	//外部硬件看门狗
	EXTERN_WATCH_DOG void EWdtInit(void);
	EXTERN_WATCH_DOG void ClearEWatchdog(void);

	//内部独立看门狗
	EXTERN_WATCH_DOG void WdtInit(uint16 ReloadValue);		// 复位间隔时间 = ReloadValue * 1ms
	EXTERN_WATCH_DOG void ClearWatchdog(void);				// 重新装载ReloadValue

	/* 注意: 因为内部独立看门狗使用的是内部低速时钟，精度较低，稍微有些漂移，所以在调用
	         ClearIWatchDog的时间间隔与复位的时间间隔相差的不要太多，比如复位间隔为2秒，
	         清看门狗的时间间隔最好在1.5秒以下 */
	
#endif

