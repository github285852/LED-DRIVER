#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

#include "sys.h"

typedef struct
{
	u8 check_flag;
	u8 err_flag;
	u16 cnt_1ms;
	u16 cnt_2ms;
	u16 cnt_5ms;
	u16 cnt_10ms;
	u16 cnt_20ms;
	u16 cnt_50ms;
	u16 time;
}loop_t;

void Loop_check(void);

void Duty_Loop(void);

void Inner_Loop(float);

void Outer_Loop(float);
void Duty_1ms(void);
void Duty_2ms(void);
void Duty_5ms(void);
void Duty_10ms(void);
void Duty_20ms(void);
void Duty_50ms(void);

extern loop_t loop;

#endif

