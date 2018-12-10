#include "includes.h"


CONFIG default_data = 
{
0
};



void GPIO_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD,ENABLE);//使能PORTA,PORTE时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // SWD可用,PB3 PB4,PA15,为普通IO
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 	GPIO_Init(GPIOC, &GPIO_InitStructure);//
	PCout(0) = 1;
	PCout(2) = 1;
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
// 	GPIO_Init(GPIOB, &GPIO_InitStructure);//
//	PBout(6) =  1;
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8 | GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 			 //上拉
 	GPIO_Init(GPIOD, &GPIO_InitStructure);//
}

SYS Sys;
extern float I_true[LED_CH];
int main(void)
{
	int i;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SysTick_Configuration();
	GPIO_init();
  PWM16_init();
	LoadConfig();
	ADC1Init();
	dmx512_init();
	ledpower_init();
  Debug_printf("LED DRIVER V1.0\r\n");
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
	//IWDG_Init(4,625);    //与分频数为64,重载值为625,溢出时间为1s	
	//显示
	//自我矫正最小直流PWM
	if(I_CHECK_PIN==0)
	{
		delay_ms(10);
		if(I_CHECK_PIN==0) //找出电流等于零的临界值
			Sys.f_find_min_pwm = 1;
	}
  while(1)
  {		
		Duty_Loop();
  }
  return 0;
}
u16 EXP_PWM[LED_CH];
float DMX_CTL_I[LED_CH];
u16 max_current[5]={MAX_CURRENT0,MAX_CURRENT1,MAX_CURRENT2,MAX_CURRENT3,MAX_CURRENT4};//mA
float EXP_OUT_I[LED_CH];
void receiving_dmx_data(void)
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
	float temp_I;
	unsigned char sum=0;
	u16 data;
	if(IS_A_BOARD)
	{
		Sys.Config.addr = 1;
	}
	else
	{
		Sys.Config.addr = 18;
	}
	p = &DMX512_RX_BUF[Sys.Config.addr];
	for(i=0;i<16;i++)
	{
		sum += p[i];
	}
	if(p[i] == sum)
	{
		for(i=0;i<LED_CH;i++)
		{
			data = *(p+i*2)<<8 | *(p+i*2+1);
			DMX_CTL_I[i] = max_current[i]*data/65536.0;
			SetLedPower(i,DMX_CTL_I[i]);
		}
		data = *(p+i*2)<<8 | *(p+i*2+1);
		data = TARR1*data/65536.0;
		//set_pwm(5,data);//FAN_OUT
		if(data !=0)
			PAout(8) = 1;
		else
			
			PAout(8) = 0;
	}
#endif
}

