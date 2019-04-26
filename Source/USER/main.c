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
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD,ENABLE);//ʹ��PORTA,PORTEʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // SWD����,PB3 PB4,PA15,Ϊ��ͨIO
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 	GPIO_Init(GPIOC, &GPIO_InitStructure);//
	PCout(0) = 1;
	PCout(2) = 1;
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//

	
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
	SysInit();
//	IWDG_Init(4,625);    //���Ƶ��Ϊ64,����ֵΪ625,���ʱ��Ϊ1s	
	//��ʾ
	//���ҽ�����Сֱ��PWM
  while(1)
  {		
		Duty_Loop();
  }
  return 0;
}


