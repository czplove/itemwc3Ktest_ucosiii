#ifndef __TEST_TASKS_H
#define __TEST_TASKS_H

#include "tiza_include.h"

	#ifdef TEST_TASKS_GLOBAL
		#define TEST_TASKS_EXTERN
	#else
		#define TEST_TASKS_EXTERN	extern
	#endif

	TEST_TASKS_EXTERN void CanTask(void* p_arg);
	TEST_TASKS_EXTERN void Uart485Task(void* p_arg);
	TEST_TASKS_EXTERN void WatchDogTask(void* p_arg);
	TEST_TASKS_EXTERN void GpsTask(void* p_arg);
	TEST_TASKS_EXTERN void GsmTask(void* p_arg);
	TEST_TASKS_EXTERN void IoTask(void* p_arg);
	TEST_TASKS_EXTERN void RtcTask(void* p_arg);
	TEST_TASKS_EXTERN void FlashTask(void* p_arg);
	TEST_TASKS_EXTERN void SpiFlashTask(void* p_arg);
	TEST_TASKS_EXTERN void AdcTask(void* p_arg);
	TEST_TASKS_EXTERN void SdCardTask(void* p_arg);
	TEST_TASKS_EXTERN void HwTimerTask(void* p_arg);
	
	TEST_TASKS_EXTERN void GenTask(void* p_arg);
	
#endif

