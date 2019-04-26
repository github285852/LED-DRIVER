#include "includes.h"

float EXP_OUT_I[LED_CH]={0.0,0.0,0.0,0.0,0.0};
u16 EXP_PWM[LED_CH]={0,0,0,0,0};
u16 I_changes=0;
u8 sum_erro=0;
float DMX_CTL_I[LED_CH];
float Last_DMX_CTL_I[LED_CH];
const float PID_MK_TAB[LED_CH] = {MK0,MK1,MK2,MK3,MK4};

void SysInit(void)
{
	int i;
	for(i=0;i<LED_CH;i++)
	{
		EXP_PWM[i] = 0;
	}
	Sys.Config.addr = 1;
}


int handle_dmx_data(void)
{
	int i;
#if KEIL_DEBUG
	for(i = 0;i<LED_CH;i++)
	#if DEBUG_I_OUT
		SetLedPower(i,EXP_OUT_I[i]);
	#else
		set_pwm(i,EXP_PWM[i]);
	#endif
#else
	u8 *p;
///	float temp_I;
	unsigned char sum=0;
	u16 data;
	if(Sys.dmx_hanle==0)
		return 0;
	switch(DMX512_RX_BUF[0])
	{
		case 0:
		case 1:
		p = &DMX512_RX_BUF[Sys.Config.addr];
		for(i=0;i<16;i++)
		{
			sum += p[i];
		}
		if(p[i] == sum)//
		{
			for(i=1;i<LED_CH;i++)
			{
				data = *(p+i*2)<<8 | *(p+i*2+1);
				
				if(DMX512_RX_BUF[0]==0)
				{
					DMX_CTL_I[i] = Sys.Config.cal.max_current[i]*data/65536.0;
					SetLedPower(i,DMX_CTL_I[i]);
					if(Last_DMX_CTL_I[i] != DMX_CTL_I[i])
						I_changes++;
					Last_DMX_CTL_I[i] =  DMX_CTL_I[i];
				}
				else
				{
					if(i==2)
					{
						SetLedPowerOpen(0,data);
					}
					else
					{
						SetLedPowerOpen(i,data);
					}
				}
			}		
			data = *(p)<<8 | *(p+1);
			data = TARR1*data/65536.0;
			//set_pwm(5,data);//FAN_OUT
			if(data !=0)
				set_pwm(7,TARR);
			else	
				set_pwm(7,0);
		}
		else
		{
			sum_erro++;
		}
		break;
		//case 0xcc://RDM

		//break;
		default:break;
	}
#endif
	Sys.dmx_hanle = 0;
	return 0;
}
