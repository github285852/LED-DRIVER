#include "includes.h"

u8 DMX512_DATA_BUF[513];
CONFIG default_data = 
{
0
};
SYS Sys;
extern float I_true[LED_CH];
float EXP_OUT_I[LED_CH]={0.0,0.0,0.0,0.0,0.0};
u16 EXP_PWM[LED_CH];
float DMX_CTL_I[LED_CH];
float Last_DMX_CTL_I[LED_CH];
u16 I_changes=0;
u8 sum_erro=0;
u16 max_current[5]={MAX_CURRENT0,MAX_CURRENT1,MAX_CURRENT2,MAX_CURRENT3,MAX_CURRENT4};//mA


void GPIO_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD,ENABLE);//ʹ��PORTA,PORTEʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // SWD����,PB3 PB4,PA15,Ϊ��ͨIO
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 	GPIO_Init(GPIOC, &GPIO_InitStructure);//
	PCout(0) = 1;
	PCout(2) = 1;
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
// 	GPIO_Init(GPIOB, &GPIO_InitStructure);//
//	PBout(6) =  1;
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8 | GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 			 //����
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//
}

int main(void)
{
//	int i;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SysTick_Configuration();
	GPIO_init();
	Debug_init();
  PWM16_init();
	LoadConfig();
	ADC1Init();
	dmx512_init();
	ledpower_init();
  Debug_printf("LED DRIVER V1.0\r\n");
	PanleInit();
//	I_true[0] = 0;
//	while(1)
//	{
//		I_true[0]++;
//		if(I_true[0]>100)
//			I_true[0] = 0;
//	  Send_I_to_PC();
//		delay_ms(10);
//	}
//	
	//set_pwm(0,1294);
	IWDG_Init(4,625);    //���Ƶ��Ϊ64,����ֵΪ625,���ʱ��Ϊ1s	
	//��ʾ
	//���ҽ�����Сֱ��PWM
	if(I_CHECK_PIN==0)
	{
		delay_ms(10);
		if(I_CHECK_PIN==0) //�ҳ�������������ٽ�ֵ
			Sys.AutoCal = 1;
	}
  while(1)
  {		
		Duty_Loop();
  }
  return 0;
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

		//break;
		default:break;
	}
#endif
	Sys.dmx_hanle = 0;
	return 0;
}

