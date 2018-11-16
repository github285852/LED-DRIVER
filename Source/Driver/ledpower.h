#ifndef __LEDPOWER_H
#define __LEDPOWER_H

#define PWM_MIN   	600
#define PWM_MAX			10000

#define MIN_CURRENT_MA 	5 //最小直流
#define MIN_I		0.0  //ma


void ledpower_task(float T);
void ledpower_init(void);
void Send_I_to_PC(void);
void SetLedPower(unsigned char ch,float mA);
void find_min_current_task(void);

#endif


