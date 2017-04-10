#define SYS_GLOBAL

#include "tiza_sys.h"

#define SYS_DEBUG

#ifdef CREATE_LIB
__asm void CPU_IntDis(void)
{
  CPSID   I
  BX      LR
}

__asm void CPU_IntEn(void)
{
  CPSIE   I
  BX      LR
}
#endif

//配置CPU各模块时钟
void InitSystem(void)
{
	SystemInit();
}

//ticks:节拍定时器中断间隔,单位：ms
void SysTickConfig(uint32 ticks)
{
	uint32 reload;
	
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//SYSTICK使用外部时钟源	 
	
	reload=168/8;							//每秒钟的计数次数 单位为K		 
	reload*=(1000000/ticks);	//根据delay_ostickspersec设定溢出时间
	//reload为24位寄存器,最大值:16777215	
	if(reload > 0xFFFFFF)
		reload = 0xFFFFFF;
	
	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;//开启SYSTICK中断
	SysTick->LOAD=reload; 					//每1/OS_TICKS_PER_SEC秒中断一次
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //开启SYSTICK
}
