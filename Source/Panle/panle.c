#include "includes.h"


float EXP_OUT_I[LED_CH]={0.0,0.0,0.0,0.0,0.0};
u16 EXP_PWM[LED_CH]={0,0,0,0,0};
u16 I_changes=0;
u8 sum_erro=0;
float DMX_CTL_I[LED_CH];
float Last_DMX_CTL_I[LED_CH];

const unsigned char A_DUID[ID_LEN] = {'A','B','O','A','R','D'};
const unsigned char B_DUID[ID_LEN] = {'B','B','O','A','R','D'};

const float PID_MK_TAB[LED_CH] = {MK0,MK1,MK2,MK3,MK4};


void SysInit(void)
{
//	if(IS_A_BOARD)
//	{
//		Sys.Config.addr = A_BOARD_ADDR;
//		memcpy(Sys.UID,A_DUID,ID_LEN);
//	}
//	else
//	{
//		Sys.Config.addr = B_BOARD_ADDR;
//		memcpy(Sys.UID,B_DUID,ID_LEN);
//	}
//	if(I_CHECK_PIN==0)
//	{
//		delay_ms(10);
//		if(I_CHECK_PIN==0) //找出电流等于零的临界值
//			Sys.AutoCal = 1;
//	}
	Sys.Config.addr = 0;
	NEXT_OUT = 0;
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
	//
	if(Sys.Config.addr==0)//初始化地址，
	{
		if(DMX512_DATA_BUF[0]==2)
		{
			NEXT_OUT = 1;
			Sys.Config.addr = DMX512_DATA_BUF[1];
			//递交给下一个驱动板
			if(MallocRDMTxBuf(2))//DMA 发送问题
				return 1;	
			delay_ms(100);
			RDM_SendBuf[0] = 2;
			RDM_SendBuf[1] = Sys.Config.addr+17;
			RDMDMASend(RDM_SendBuf,2);
		}
		return 0;
	}
//	Sys.Config.addr = 18;
	switch(DMX512_DATA_BUF[0])
	{
		case 0:
		case 1:
		p = &DMX512_DATA_BUF[Sys.Config.addr];
		for(i=0;i<16;i++)
		{
			sum += p[i];
		}
		if(p[i] == sum)//
		{
			for(i=0;i<LED_CH;i++)
			{
				data = *(p+i*2)<<8 | *(p+i*2+1);
				
				if(DMX512_DATA_BUF[0]==0)
				{
					DMX_CTL_I[i] = Sys.Config.cal.max_current[i]*data/65536.0;
					SetLedPower(i,DMX_CTL_I[i]);
					if(Last_DMX_CTL_I[i] != DMX_CTL_I[i])
						I_changes++;
					Last_DMX_CTL_I[i] =  DMX_CTL_I[i];
				}
				else
				{
					SetLedPowerOpen(i,TARR*data/65536.0);
				}
			}
			data = *(p+i*2)<<8 | *(p+i*2+1);
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
		/*在数据字节接收的地方处理 */
		//break;
		default:break;
	}
#endif
	Sys.dmx_hanle = 0;
	return 0;
}
