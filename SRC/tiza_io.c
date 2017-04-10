#define IO_MOD_GLOBAL

#include "tiza_io.h"


#define OFF_RED_LED()		(GPIO_SetBits(GPIOB, GPIO_Pin_15))
#define ON_RED_LED()		(GPIO_ResetBits(GPIOB, GPIO_Pin_15))

#define OFF_GREEN_LED()		(GPIO_SetBits(GPIOB, GPIO_Pin_14))
#define ON_GREEN_LED()		(GPIO_ResetBits(GPIOB, GPIO_Pin_14))


//LED IO初始化
void IoInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE,ENABLE); //使能GPIOD的时钟

	/*
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;  //推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;  //上拉输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //高速GPIO
	GPIO_Init(GPIOF,&GPIO_InitStructure);*/

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14 | GPIO_Pin_15;	// red and green led
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;  //推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL; 
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //高速GPIO
	GPIO_Init(GPIOB,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;	// 232_pwr
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;  //推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL; 
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //高速GPIO
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10 | GPIO_Pin_11;	// changer ctrl
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;  //推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL; 
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz; //高速GPIO
	GPIO_Init(GPIOD,&GPIO_InitStructure);
#if 0
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;	// acc
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;//输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; //高速GPIO
	GPIO_Init(GPIOC,&GPIO_InitStructure);
#endif
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;	// shell tamper
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;	//输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; //高速GPIO
	GPIO_Init(GPIOE,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;	// check power
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;//输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz; //高速GPIO
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	//GPIO_SetBits(GPIOF,GPIO_Pin_9); //gsm c_led

    GPIO_SetBits(GPIOC, GPIO_Pin_2); //232 pwr
    
    GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15); // LED1, LED3

	GPIO_ResetBits(GPIOD, GPIO_Pin_10); //off BAT charge

	GPIO_ResetBits(GPIOD, GPIO_Pin_11); //off GPS battary
}

// LedNo: 1: Led1  2: Led2     Disp: 1: on    0: off
void LedDisp(uint8 LedNo, uint8 Disp)
{
	if(LedNo == LED1)
	{
		if(Disp == FALSE)
		{
			OFF_RED_LED();
		}
		else
		{
			ON_RED_LED();
		}
	}

	if(LedNo == LED2)
	{
		if(Disp == FALSE)
		{
			OFF_GREEN_LED();
		}
		else
		{
			ON_GREEN_LED();
		}
	}
}

// 0: no power    1: power on
#if 0
uint8 GetAccStatus(void)
{
	uint8 result;
	
	result = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3);

	return !result;
}
#endif

// 0: ok    1: tear down
uint8 GetTearDownStatus(void)
{
	uint8 result;
	
	result = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_15);

	return !result;
}

// 0: no power    1: power on
uint8 GetExtPowerStatus(void)
{
	uint8 result;
	
	result = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3);

	return result;
}

