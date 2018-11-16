
#include "includes.h"
PID Pid_I_true[LED_CH];
PID Pid_I_lpf[LED_CH];
unsigned int pwm_out[LED_CH];
extern float I_true[LED_CH];
extern float I_lpf[LED_CH];
float Iout[LED_CH];
u16 MIN_CURRENT[LED_CH]={5,17,22,23,21};
u16 CURRENT_MIN_PWM[LED_CH]={891,100,105,100,100};
void ledpower_init(void)
{
	int i;
	for(i=0;i<LED_CH;i++)
	{
    memset(&Pid_I_true[i],0,sizeof(PID));
	  memset(&Pid_I_lpf[i],0,sizeof(PID));

		PID_SetKp(&Pid_I_true[i],0.4);
		PID_SetKi(&Pid_I_true[i],0.02);//0.02
		PID_SetKd(&Pid_I_true[i],0.8);
		
		PID_SetKp(&Pid_I_lpf[i],0.04);
		PID_SetKi(&Pid_I_lpf[i],0.0004);
		PID_SetKd(&Pid_I_lpf[i],0.8);
	}
}


void SetLedPower(u8 ch,float mA)
{
	float pwm;
	int i;
	if(ch<LED_CH)
	{
		if((mA<MIN_CURRENT[ch])&&(mA>0))
		{
			pwm = TARR1 - TARR1*mA/MIN_CURRENT[i];  //导通相当于关闭LED
			set_pwm(ch,CURRENT_MIN_PWM[ch]);
			set_pwm(8+ch,pwm);
			Sys.pid_on[ch] = 0;
		}
		else if(mA!=0)
		{
			PID_SetPoint(&Pid_I_lpf[ch],mA);
		  Pid_I_lpf->SumError = 0;
			Pid_I_true->SumError = 0;
			set_pwm(ch+8,0);
			Sys.pid_on[ch] = 1;
		}
		else
		{
			set_pwm(ch,0);
			set_pwm(8+ch,0);
			Sys.pid_on[ch] = 0;
		}
	}
	Iout[ch] = mA;
}

void limit(u16 x,u16 min ,u16 max)
{
	if(x<min)
	{
		x = min;
	}
	else if(x>max)
	{
		x = max;
	}
}
//单级PID 位置式
//void ledpower_task(float T)
//{
//  float temp;
//	int i;
//	for(i=0;i<LED_CH;i++)
//	{
//		temp = PID_PosLocCalc(&Pid_I_true[i],I_true[i],100,T);
//		pwm_out[i] += temp;
//		
//		pwm_out[i] = LIMIT(pwm_out[i],PWM_MIN,PWM_MAX);
//	  set_pwm(i,pwm_out[i]);
//	}
//}


//串级PID,位置式PID
//		PID_SetKp(&Pid_I_true[i],0.4);
//		PID_SetKi(&Pid_I_true[i],0.2);
//		PID_SetKd(&Pid_I_true[i],0.8);
//		
//		PID_SetKp(&Pid_I_lpf[i],0.04);
//		PID_SetKi(&Pid_I_lpf[i],0.0004);
//		PID_SetKd(&Pid_I_lpf[i],0.8);
void ledpower_task(float T)
{
	u16 pwm;
  static float out1[LED_CH]={0},out2;
	int i;
	for(i=0;i<LED_CH;i++)
	{
		if(Sys.pid_on[i])
		{
			out1[i] += PID_PosLocCalc(&Pid_I_lpf[i],I_lpf[i],10,T);//外环
			
			PID_SetPoint(&Pid_I_true[i],out1[i]);
			
			out2 = PID_PosLocCalc(&Pid_I_true[i],I_true[i],10,T);//内环
			
			pwm_out[i] += out2; 
			pwm_out[i] = LIMIT(pwm_out[i],PWM_MIN,PWM_MAX);
			set_pwm(i,pwm_out[i]);
		}
		else
		{
			if(Iout[i]!=0)
			{
				pwm = TARR1 - TARR1*Iout[i]/MIN_CURRENT[i];  //导通相当于关闭LED
				set_pwm(8+i,pwm);
			}
			else
			{
				set_pwm(8+i,0);
			}
				
		}
	}
}

//单级PID，
//void ledpower_task(float T)
//{
//  float temp;
//	int i;
//	for(i=0;i<LED_CH;i++)
//	{
//		temp = PID_PosLocCalc(&Pid_I_lpf[i],I_lpf[i],100,T);         //增量式 ，P = 0.8 , I= 0,D = 0.05
////		temp = PID_PosLocCalc(&Pid_I_true[i],I_true[i],2300,T);  //位置式  ，P=0.8;I=0.01;D=0.8
//		pwm_out[i] += temp;
//		pwm_out[i] = LIMIT(pwm_out[i],PWM_MIN,PWM_MAX);
//	  set_pwm(i,pwm_out[i]);
//	}
//}
//串级PID
//void ledpower_task(float T)
//{
//  static float out1[LED_CH]={0},out2;
//	int i;
//	for(i=0;i<LED_CH;i++)
//	{
//		
//		out1[i] += PID_PosLocCalc(&Pid_I_lpf[i],I_lpf[i],2300,T);//外环  ，P=0.04;I=0.0005;D=1
//		//out1[i] += PID_IncLocCalc(&Pid_I_lpf[i],I_lpf[i],T);   //外环
//		
//		PID_SetPoint(&Pid_I_true[i],out1[i]);
//		
//		//out2 = PID_PosLocCalc(&Pid_I_true[i],I_true[i],2300,T);//内环   //位置式  ，P=0.8;I=0.01;D=0.8
//		out2 = PID_IncLocCalc(&Pid_I_true[i],I_true[i],T);//内环

//		pwm_out[i] += out2; 
//		pwm_out[i] = LIMIT(pwm_out[i],PWM_MIN,PWM_MAX);
//	  set_pwm(i,pwm_out[i]);
//	}
//}


void Send_I_to_PC(void)
{
	u16 I[LED_CH];
	u8 buf[6]={0xA5,0,0,0,0,0xAA};
  int i;
	for(i=0;i<LED_CH;i++)
	{
		I[i] = I_true[i]*10;
	}
  memcpy(buf+1,(u8 *)&I[0],2);
	for(i=0;i<LED_CH;i++)
	{
		I[i] = I_lpf[i]*10;
	}
  memcpy(buf+3,(u8 *)&I[0],2);
	rs485_send_buf(buf,6);
}

