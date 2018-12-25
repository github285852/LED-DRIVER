
#include "includes.h"
PID Pid_I_true[LED_CH];
PID Pid_I_lpf[LED_CH];
unsigned int pwm_out[LED_CH];
extern float I_true[LED_CH];
extern float I_lpf[LED_CH];
//float Iout[LED_CH];
//B1
//float MIN_CURRENT[LED_CH]={17,20.8,16.5,13.4,19};
//u16 CURRENT_MIN_PWM[LED_CH]={850,1000,1060,1050,900};
//B2
//float MIN_CURRENT[LED_CH]={21,20.5,16.9,13.7,20.6};
//u16 CURRENT_MIN_PWM[LED_CH]={1000,1150,970,1060,950}; 

float MIN_CURRENT[LED_CH]={5,5,5,5,5};
u16 CURRENT_MIN_PWM[LED_CH]={815,815,870,815,815};

////A1
//float MIN_CURRENT[LED_CH]={18,13,14.8,18,17};
//u16 CURRENT_MIN_PWM[LED_CH]={860,950,1050,1100,905};// {950,1020,960,970,1000};
//float MIN_CURRENT[LED_CH]={2,12.26,17,20.6,1};
//u16 CURRENT_MIN_PWM[LED_CH]={872,1020,960,970,918};// {950,1020,960,970,1000};

#if NEURAL_PID
const NeuralPID Initnpid=
{
	.M = 1,
	.MK = 2,
	.xiteP = 0.50,
	.xiteI = 0.25,
	.xiteD = 0.50,
	.wkp = 264860224,
	.wki = 132578984,
	.wkd = 62064808, //初始加权
	.exp = 0,
	.out = PWM_MIN,
};
NeuralPID I_npid[LED_CH];
#endif


void ledpower_init(void)
{
	int i;
	#if	NEURAL_PID
	for(i=0;i<LED_CH;i++)
	{
		I_npid[i] = Initnpid;
	}
	#else
	for(i=1;i<4;i++)
	{
    memset(&Pid_I_true[i],0,sizeof(PID));
	  memset(&Pid_I_lpf[i],0,sizeof(PID));

		PID_SetKp(&Pid_I_true[i],0.6);
		PID_SetKi(&Pid_I_true[i],0.0006);
		PID_SetKd(&Pid_I_true[i],0.4);
	}

	PID_SetKp(&Pid_I_true[0],0.3);
	PID_SetKi(&Pid_I_true[0],0.0003);
	PID_SetKd(&Pid_I_true[0],0.2);
	
	PID_SetKp(&Pid_I_true[4],0.3);
	PID_SetKi(&Pid_I_true[4],0.0003);
	PID_SetKd(&Pid_I_true[4],0.2);
	#endif
	for(i=0;i<LED_CH;i++)
	{
		CURRENT_MIN_PWM[i] = Sys.Config.current_min_pwm[i];
		MIN_CURRENT[i] = MIN_CURRENT_MA;
	}
}
//开环直接设置LED
void SetLedPowerOpen(u8 ch,u16 pwm)
{
	u16 pwmout;
	Sys.pid_on[ch] = 0;
	if(pwm==0)
	{
		pwmout = 0;
	}
	else
	{
		pwmout = pwm + Sys.Config.current_min_pwm[ch];
	}
	set_pwm(ch,pwmout);
}

void SetLedPower(u8 ch,float mA)
{
	float pwm;
	if(ch<LED_CH)
	{
		#if NEURAL_PID
		NeuralPID_SetExp(&I_npid[ch],mA);
		#else
		PID_SetPoint(&Pid_I_true[ch],mA);
		#endif
		if(mA <= MIN_I)  //亮度比较小，就关
		{
			set_pwm(ch,0);
			set_pwm(8+ch,0);
			Sys.pid_on[ch] = 0;
			I_npid[ch].out = PWM_MIN;
//			pwm_out[ch] = PWM_MIN;
			pwm_out[ch]  = CURRENT_MIN_PWM[ch];
		}
//		else if(mA<MIN_CURRENT[ch])
//		{
//			pwm = TARR1 - TARR1*mA/MIN_CURRENT[ch];  //导通相当于关闭LED
//			set_pwm(ch,CURRENT_MIN_PWM[ch]);
//			set_pwm(8+ch,pwm);
//			Sys.pid_on[ch] = 0;
//			I_npid[ch].out = CURRENT_MIN_PWM[ch];
//			pwm_out[ch] = CURRENT_MIN_PWM[ch];
//		}
		else 
		{
			set_pwm(ch+8,0);
			Sys.pid_on[ch] = 1;
		}
	}
//	Iout[ch] = mA;
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
void ledpower_task(float T)
{
  float temp;
	int i;
	for(i=0;i<LED_CH;i++)
	{
		if(Sys.pid_on[i])
		{
			#if NEURAL_PID
			pwm_out[i] = (u16)SignleNeuralAdaptivPID(&I_npid[i],I_true[i]);//返回的是累计值
			LIMIT(pwm_out[i],PWM_MIN,PWM_MAX);
			set_pwm(i,pwm_out[i]);
			#else
			temp = PID_PosLocCalc(&Pid_I_true[i],I_true[i],65535,T);//返回的是增量
			pwm_out[i] += temp;
			
			pwm_out[i] = LIMIT(pwm_out[i],PWM_MIN,PWM_MAX);
			set_pwm(i,pwm_out[i]);
			#endif
		}
	}
}

void find_min_current_task(void)
{
	int i,timer,tim;
	u16 out;
	float error;
	if(Sys.f_find_min_pwm)
	{
		for(i=0;i<LED_CH;i++)
		{
			MIN_CURRENT[i] = I_true[i]; //当电流非常小时获取，
			out = 800;
			//MIN_CURRENT[i] = 0;
//			SetLedPower(i,MIN_CURRENT_MA);
//			Sys.pid_on[i] = 1;
//			timer = 0;
//			tim =0;
			while(1)
			{
					IWDG_Feed();//喂狗
					set_pwm(i,out);
					delay_ms(10);
					if((I_true[i]-MIN_CURRENT[i])> 0.1)
					{
						timer++;
						if(timer>200)
						{
							MIN_CURRENT[i] = MIN_CURRENT_MA;
							Sys.Config.current_min_pwm[i] = CURRENT_MIN_PWM[i] = out;
							break;
						}
					}
					else
					{
						out++;
					}
//				ledpower_task(0);
//				IWDG_Feed();//喂狗
//				delay_ms(5);
//				tim++;
//				if(tim>2000)
//				{
//				//错误，10S还没调好，
//				}
//				#if NEURAL_PID
//				error = I_npid[i].error;
//				out = I_npid[i].out;
//				#else
//				erro = Pid_I_true[i].LastError;
//				out = pwm_out[i];
//				#endif
//				if((error)<0.1)//误差比较小
//				{
//					timer++;
//					if(timer>200)//持续1S
//					{
//						MIN_CURRENT[i] = MIN_CURRENT_MA;
//						Sys.Config.current_min_pwm[i] = CURRENT_MIN_PWM[i] = out;
//						break;
//					}
//				}
			}
		}
		SaveConfig();
		Sys.f_find_min_pwm = 0;
	}
}



//串级PID,位置式PID
//		PID_SetKp(&Pid_I_true[i],0.4);
//		PID_SetKi(&Pid_I_true[i],0.2);
//		PID_SetKd(&Pid_I_true[i],0.8);
//		
//		PID_SetKp(&Pid_I_lpf[i],0.04);
//		PID_SetKi(&Pid_I_lpf[i],0.0004);
//		PID_SetKd(&Pid_I_lpf[i],0.8);
//void ledpower_task(float T)
//{
//	u16 pwm;
//  static float out1[LED_CH]={0},out2;
//	int i;
//	for(i=0;i<LED_CH;i++)
//	{
//		if(Sys.pid_on[i])
//		{
//			out1[i] += PID_PosLocCalc(&Pid_I_lpf[i],I_lpf[i],10,T);//外环
//			
//			PID_SetPoint(&Pid_I_true[i],out1[i]);
//			
//			out2 = PID_PosLocCalc(&Pid_I_true[i],I_true[i],10,T);//内环
//			
//			pwm_out[i] += out2; 
//			pwm_out[i] = LIMIT(pwm_out[i],PWM_MIN,PWM_MAX);
//			set_pwm(i,pwm_out[i]);
//		}
//	}
//}

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

