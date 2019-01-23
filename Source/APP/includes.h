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

#endif

#define KEIL_DEBUG	0
#define DEBUG_I_OUT	1
#define NEURAL_PID	1

#define DEBUG			1
#define CONFIG_ADDRESS          0x0803F000


typedef struct 
{
	u8 addr;
	u16 current_min_pwm[LED_CH];//LEDֱ����СPWM
	u32 check;
}CONFIG;

typedef struct
{
	u8 pid_on[LED_CH];
	u8 f_find_min_pwm;
	u8 dmx_hanle;
	unsigned char UID[ID_LEN];
	STATUS status;
	CONFIG Config;
}SYS;

extern SYS Sys;


#endif

