#include "pwm16.h"
#include "mymath.h"
/* 
TIMӳ�������Ĳο��ֲ�119ҳ

PWM0  ->	PA15
PWM1	->	PB3
PWM2	->	PB10
PWM3	->	PB11

PWM4	->	PC6
PWM5	->	PC7
PWM6	->	PC8
PWM7	->	PC9

PWM8	->	PB6
PWM9	->	PB7
PWM10	->	PB8
PWM11	->	PB9

PWM12	->	PA0
PWM13	->	PA1

//PWM14	->	PA2
//PWM15	->	PA3

*/

void TIM1_PWM_Init(u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
  GPIO_Init(GPIOA,&GPIO_InitStructure);

   //��ʼ��TIM2
	
	TIM_TimeBaseStructure.TIM_Period = 10-1; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =	psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

 	//��ʼ��TIM2 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM2 OC1
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
	
//	//��ʼ��TIM2 Channel2 PWMģʽ	 
//	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM2 OC2
//	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
// 
// 	//��ʼ��TIM2 Channel3 PWMģʽ	 
//	TIM_OC3Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM2 OC1
//	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
//	
//	//��ʼ��TIM2 Channel4 PWMģʽ	 
//	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM2 OC1
//	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���


  TIM_ARRPreloadConfig(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);//pwm���
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM3

}

void fanboOut(void)
{
	//450k
	TIM1_PWM_Init(18-1);
	TIM_SetCompare2(TIM1,9-1);
	//TIM_SetCompare2(TIM1,0);
}

//TIM2 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM2_PWM_Init(u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
 
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);//��ȫ��ӳ��
	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
  GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3|GPIO_Pin_10|GPIO_Pin_11;
  GPIO_Init(GPIOB,&GPIO_InitStructure);
  
   //��ʼ��TIM2
	TIM_TimeBaseStructure.TIM_Period = TARR-1; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

 	//��ʼ��TIM2 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM2 OC1
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
	
	//��ʼ��TIM2 Channel2 PWMģʽ	 
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM2 OC2
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
 
 	//��ʼ��TIM2 Channel3 PWMģʽ	 
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM2 OC1
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
	
	//��ʼ��TIM2 Channel4 PWMģʽ	 
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM2 OC1
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
  
	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIM3
}


//TIM3 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM3_PWM_Init(u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
 //	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC	| RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);//��ȫ��ӳ��
 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
  GPIO_Init(GPIOC,&GPIO_InitStructure);
  
   //��ʼ��TIM3
	TIM_TimeBaseStructure.TIM_Period = TARR-1; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

 	//��ʼ��TIM3 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC1
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
	
	//��ʼ��TIM3 Channel2 PWMģʽ	 
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC2
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
 
 	//��ʼ��TIM3 Channel3 PWMģʽ	 
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC1
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
	
	//��ʼ��TIM3 Channel4 PWMģʽ	 
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM3 OC1
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
  
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
}

//TIM4 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM4_PWM_Init(u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
 //	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB	| RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
  GPIO_Init(GPIOB,&GPIO_InitStructure);
  
   //��ʼ��TIM3
	TIM_TimeBaseStructure.TIM_Period = TARR1-1; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

 	//��ʼ��TIM4 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4 OC1
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���
	
	//��ʼ��TIM4 Channel2 PWMģʽ	 
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4 OC2
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
 
 	//��ʼ��TIM4 Channel3 PWMģʽ	 
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4 OC3
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���
	
	//��ʼ��TIM4 Channel4 PWMģʽ	 
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM4 OC4
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���
  
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM3
}

//TIM5 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM5_PWM_Init(u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
 	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA	| RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
  GPIO_Init(GPIOA,&GPIO_InitStructure);
  
   //��ʼ��TIM5
	TIM_TimeBaseStructure.TIM_Period = TARR1-1; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

 	//��ʼ��TIM5 Channel1 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 OC1
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR1�ϵ�Ԥװ�ؼĴ���
	
	//��ʼ��TIM5 Channel2 PWMģʽ	 
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 OC2
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR2�ϵ�Ԥװ�ؼĴ���
 
 	//��ʼ��TIM5 Channel3 PWMģʽ	 
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 OC3
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR1�ϵ�Ԥװ�ؼĴ���
	
	//��ʼ��TIM5 Channel4 PWMģʽ	 
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM5 OC4
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR1�ϵ�Ԥװ�ؼĴ���
  
	TIM_Cmd(TIM5, ENABLE);  //ʹ��TIM5
}

void set_pwm(unsigned char ch,unsigned int duty)
{
	if(ch>=8)
	{
		duty = LIMIT(duty,0,TARR1);
		duty = TARR1 - duty;
	}
	else
	{
		duty = LIMIT(duty,0,TARR);
		duty = TARR - duty;
	}
	switch(ch)
	{
		case 0:TIM_SetCompare1(TIM2,duty);break;
		case 1:TIM_SetCompare2(TIM2,duty);break;
		case 2:TIM_SetCompare3(TIM2,duty);break;
		case 3:TIM_SetCompare4(TIM2,duty);break;
		
		case 4:TIM_SetCompare1(TIM3,duty);break;
		case 5:TIM_SetCompare2(TIM3,duty);break;
		case 6:TIM_SetCompare3(TIM3,duty);break;
		case 7:TIM_SetCompare4(TIM3,duty);break;
		
		case 8:TIM_SetCompare1(TIM4,duty);break;
		case 9:TIM_SetCompare2(TIM4,duty);break;
		case 10:TIM_SetCompare3(TIM4,duty);break;
		case 11:TIM_SetCompare4(TIM4,duty);break;
		
		case 12:TIM_SetCompare1(TIM5,duty);break;
		case 13:TIM_SetCompare2(TIM5,duty);break;
		case 14:TIM_SetCompare3(TIM5,duty);break;
		case 15:TIM_SetCompare4(TIM5,duty);break;
		default:break;
	}
}

void PWM16_init(void)
{
	int i;
	TIM2_PWM_Init(1);
	TIM3_PWM_Init(1);
	//TIM4_PWM_Init(1);
	//TIM5_PWM_Init(1);
	for(i=0;i<7;i++)
	{
		set_pwm(i,0);
	}
	fanboOut();
}



