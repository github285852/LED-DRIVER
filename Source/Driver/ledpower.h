#ifndef __LEDPOWER_H
#define __LEDPOWER_H

#define CAL_PWM_MIN   	700
#define PWM_MAX			9090 //9090
#define CAL_MAX_PWM	9000

/* MIN_CURRENT_MA  = MIN_I,全部用直流，闭环 */
#define MIN_I		0.2 //ma
#define MIN_CURRENT_MA 	MIN_I	 //最小直流


void ledpower_task(float T);
void ledpower_init(void);
void Send_I_to_PC(void);
void SetLedPower(unsigned char ch,float mA);
void SetLedPowerOpen(unsigned char ch,unsigned short current);
void AutoCalibrateTask(void);

#endif


