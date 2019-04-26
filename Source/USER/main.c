#include "includes.h"

CONFIG default_data = 
{
0
};
extern float I_true[LED_CH];
u16 max_current[5]={MAX_CURRENT0,MAX_CURRENT1,MAX_CURRENT2,MAX_CURRENT3,MAX_CURRENT4};//mA
SYS Sys;
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
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//

	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8 | GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 			 //上拉
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
	SysInit();
//	IWDG_Init(4,625);    //与分频数为64,重载值为625,溢出时间为1s	
	//显示
	//自我矫正最小直流PWM
  while(1)
  {		
		Duty_Loop();
  }
  return 0;
}


