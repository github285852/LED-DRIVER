#ifndef __INCLUDES_H
#define __INCLUDES_H

#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "malloc.h"
#include "sys.h"
#include "delay.h"
#include "scheduler.h"
#include "pwm16.h"
#include "dmx512.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "led.h"
#include "adc.h"
#include "stmflash.h"
#include "PID.h"
#include "neuralpid.h"
#include "key.h"
#include "PWM16.h"
#include "ledpower.h"
#include "mymath.h"
#include "wdg.h"
#include "debug.h"
#include "rdm_slave.h"


#if defined PANLE

#include "panle.h"

#elif defined SPOT

#include "spot.h"

#endif

#define KEIL_DEBUG	1
#define DEBUG_I_OUT	0
#define NEURAL_PID	1

#define DEBUG			1

#define CONFIG_SIZE							(1024*5)
#define CONFIG_ADDRESS          (0x0807FFFF + 1 - CONFIG_SIZE)


typedef struct{
	u16 current_min_pwm[LED_CH];//LED直流最小PWM
	float max_current[LED_CH]; //各通道最大电流
	float min_current[LED_CH];////各通道最小电流
}CAL;

typedef struct 
{
	u8 addr;
	CAL cal;
	u32 check;
}CONFIG;


typedef struct
{
	u8 pid_on[LED_CH];
	u8 AutoCal;
	u8 dmx_hanle;
	unsigned char UID[ID_LEN];
	STATUS status;
	CONFIG Config;
}SYS;

extern SYS Sys;


#endif

