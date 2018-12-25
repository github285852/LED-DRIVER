#include "includes.h"
#include "string.h"
u32 tim_over=0;
u32 break_tim;
//DMXData *DmxData;
//���ջ����� 	
u8 DMX512_RX_BUF[513];  	//���ջ���,���513���ֽ�.
//���յ������ݳ���
u16 DMX512_RX_CNT=0;   	
u8 TEST_SUM;
u32 SUM_CNT;
void wait_receive_dmx_data(void);
void receiving_dmx_data(void);

void dmx512_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DMX_USART,ENABLE);//ʹ��DMX_USARTʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DMX_TIM, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//ʹ�ø��ù���ʱ��
	#if DMX_USART_GPIO_REMAP
	GPIO_PinRemapConfig(GPIO_Remap_DMX_USART,ENABLE);
	#endif
	GPIO_InitStructure.GPIO_Pin = DMX_RXEN_PIN;				 //PA12�˿�����
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(DMX_RXEN_GPIO, &GPIO_InitStructure);
	
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin =	DMX_TX_PIN; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(DMX_TX_GPIO, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = DMX_RX_PIN;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(DMX_RX_GPIO, &GPIO_InitStructure);//��ʼ��GPIOA.10  
	
	GPIO_EXTILineConfig(DMX_GPIO_PortSource,DMX_EXTI_PinSource);
	EXTI_InitStructure.EXTI_Line= DMX_EXTI_Line;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);	 	
	
	NVIC_InitStructure.NVIC_IRQChannel = DMX_EXTI_IRQn;			
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								
  NVIC_Init(&NVIC_InitStructure);
	
	//��ʱ����ʼ��
	TIM_TimeBaseStructure.TIM_Period = 65535; //	�Զ���ת��
	TIM_TimeBaseStructure.TIM_Prescaler = 71; //
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //
	TIM_TimeBaseInit(DMX_TIM, &TIM_TimeBaseStructure); //
	TIM_ITConfig(DMX_TIM,TIM_IT_Update,ENABLE ); //ʹ��ʱ��
	NVIC_InitStructure.NVIC_IRQChannel = DMX_TIM_IRQn;  //TIM3??
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //?????0?
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //????3?
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ?????
	NVIC_Init(&NVIC_InitStructure);  //???NVIC???
	TIM_Cmd(DMX_TIM, ENABLE);  //??TIMx	
	
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_DMX_USART,ENABLE);//��λ����
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_DMX_USART,DISABLE);//ֹͣ��λ
 
	USART_InitStructure.USART_BaudRate = 250000;	//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ���ݳ���
	USART_InitStructure.USART_StopBits = USART_StopBits_2;//����ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;///��żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�ģʽ

  USART_Init(DMX_USART, &USART_InitStructure);  //��ʼ������
  
	NVIC_InitStructure.NVIC_IRQChannel = DMX_USART_IRQn; //ʹ�ܴ���2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //�����ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure); //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
 
  USART_ITConfig(DMX_USART, USART_IT_RXNE, ENABLE);//���ж�
	USART_Cmd(DMX_USART, ENABLE);                    //ʹ�ܴ��� 

	DMX_RXEN;	//Ĭ��Ϊ����ģʽ
}

u16 tim;
void DMX_EXTI_IRQHandler(void)
{
	static u8 last_sum;
	 //if(PAin(2)==0)//
	if(DMX_RX_GPIO->IDR&DMX_RX_PIN)//������
	{
			break_tim = tim_over*65535 + DMX_TIM->CNT;
		  if((break_tim>=88)&&(break_tim<=1000000))
			{
				receiving_dmx_data();
				
//				if(USART_GetITStatus(DMX_USART, USART_IT_RXNE) != RESET) //���յ�����
//				{	 
//					USART_ReceiveData(DMX_USART); 	//��ȡ���յ�������
//				}
				if(last_sum != TEST_SUM)
					SUM_CNT++;
				last_sum = TEST_SUM;
				TEST_SUM = 0;
				tim = break_tim;
				DMX512_RX_CNT =0;
				//�����ⲿ�ж�10
				//EXTI->IMR &= 0xffffffbf;
				/*
				������һ֡4us,��������ˣ��ȱ����Լ��Ļ�������
				
				*/
			}
	}
	else //�½���
	{
		tim_over = 0;
		DMX_TIM->CNT=0;
	}
	EXTI->PR = DMX_EXTI_Line;	
}

void DMX_TIM_IRQHandler(void)   //TIM6�ж�
{
	if (TIM_GetITStatus(DMX_TIM, TIM_IT_Update) != RESET)  //���TIM3�����жϷ������
	{
		TIM_ClearITPendingBit(DMX_TIM, TIM_IT_Update);  //���TIMx�����жϱ�־ 
		tim_over++;
	}
}


void DMX_USART_IRQHandler(void)
{
	u8 res;	    
	if(USART_GetFlagStatus(DMX_USART,USART_FLAG_NE)==SET)
	{
		//USART_ClearFlag(DMX_USART,USART_FLAG_NE); 
		USART_ReceiveData(DMX_USART);    
	}
	else if(USART_GetFlagStatus(DMX_USART,USART_FLAG_FE)==SET)
	{
		//USART_ClearFlag(DMX_USART,USART_FLAG_FE); 
		USART_ReceiveData(DMX_USART);    
	}
	else if(USART_GetFlagStatus(DMX_USART,USART_FLAG_ORE)==SET)
	{
		//USART_ClearFlag(DMX_USART,USART_FLAG_ORE); 
		USART_ReceiveData(DMX_USART);    
	}	
	else if(USART_GetITStatus(DMX_USART, USART_IT_RXNE) != RESET) //���յ�����
	{	 
		res =USART_ReceiveData(DMX_USART); 	//��ȡ���յ�������
		uart_resiver(res);
		TEST_SUM += res;
		if(DMX512_RX_CNT<513)
		{
			DMX512_RX_BUF[DMX512_RX_CNT]=res;		//��¼���յ���ֵ
			DMX512_RX_CNT++;						//������������1 
			if(DMX512_RX_CNT==513)
			{
				DMX512_RX_CNT = 0;
				//wait_receive_dmx_data();
				
				//���ⲿ�ж�10
				//	EXTI->IMR |= 0x00000040;
			}
		}
	}

} 


void DMX512_handle(void)
{
//	if(Sys.dmx_handle)
//	{

//	}
}

#define USART_REC_LEN  			200  
u8  USART_RX_BUF[USART_REC_LEN]; 
u16 USART_RX_STA;  

void uart_resiver(u8 Res)
{
	if((USART_RX_STA&0x8000)==0)
	{
		if(USART_RX_STA&0x4000)
		{
			if(Res!=0x0a)USART_RX_STA=0;
			else USART_RX_STA|=0x8000;	
		}
		else 
		{	
			if(Res==0x0d)
				USART_RX_STA|=0x4000;
			else
			{
				USART_RX_BUF[USART_RX_STA&0X3FFF]=Res;
				USART_RX_STA++;
				if(USART_RX_STA>(USART_REC_LEN-1))
					USART_RX_STA=0;  
			}		 
		}
	} 
}
void rs485_send_str(unsigned char *str)
{
	DMX_TXEN;
	while(*str != 0)
	{
		while((USART2->SR&0X40)==0);//????,??????   
    USART2->DR = *str;     
		str++;
	}		
	while((USART2->SR&0X40)==0);//????,??????   
	DMX_RXEN;
}
void rs485_send_buf(unsigned char *buf,char len)
{
	DMX_TXEN;
	while(len != 0)
	{
		while((USART2->SR&0X40)==0);//????,??????   
    USART2->DR = *buf;     
		buf++;
		len--;
	}		
	while((USART2->SR&0X40)==0);//????,??????   
	DMX_RXEN;
}
unsigned int pwm[32]={0,0};
void uart_duty(void)
{
	unsigned char *p,*base_p;
	unsigned char cmd;
	unsigned char buf[50];
	unsigned int h;
	unsigned char s,i;
	unsigned temp;
	float temp_f;
	if(USART_RX_STA&0x8000)
	{
		USART_RX_BUF[USART_RX_STA&0X3FFF]=0;
		//PWM���
		p = strstr((const char*)USART_RX_BUF,(const char*)"PWM");
		if(p != NULL)
		{
			p += 3;
			cmd = atoi((const char*)p);
			if(cmd<32)
			{
				p = strstr((const char*)USART_RX_BUF,(const char*)":");
				if(p != NULL)
				{
					pwm[cmd] = atoi((const char*)(p+1));
					set_pwm(cmd,pwm[cmd]);
					//set_pwm(cmd+8,0);
					Sys.pid_on[cmd] = 0;
				}
				sprintf((char *)buf,(const char*)"PWM%d=%d Done\r\n",cmd,pwm[cmd]);
				rs485_send_str(buf);
			}
			else
				rs485_send_str((unsigned char *)"ERROR\r\n");
		}
		/////////////////�����������
		p = strstr((const char*)USART_RX_BUF,(const char*)"IOUT");
		if(p != NULL)
		{
			p += 4;
			cmd = atoi((const char*)p);
			if(cmd<32)
			{
				p = strstr((const char*)USART_RX_BUF,(const char*)":");
				if(p != NULL)
				{
					temp_f = atof((const char*)(p+1));
					SetLedPower(cmd,temp_f);
					if(Sys.pid_on==0)
					{
						//ledpower_init();
					}
				}
				sprintf((char *)buf,(const char*)"IOUT%d=%0.1f Done\r\n",cmd,temp_f);
				rs485_send_str(buf);
			}
			else
				rs485_send_str((unsigned char *)"ERROR\r\n");
		}	
		
		p = strstr((const char*)USART_RX_BUF,(const char*)"HSI:");
		if(p!=NULL)
		{
			h = atoi((const char*)(p+4));
			if(h>=0&&(h<=360))
			{
				p = strstr((const char*)p,(const char*)",");
				if(p!=NULL)
				{
					s = atoi((const char*)(p+1));
					if((s>=0)&&(s<=100))
					{
						p = strstr((const char*)(p+1),(const char*)",");
						if(p!=NULL)
						{
							i = atoi((const char*)(p+1));
							if((i>=0)&&(i<=100))
							{

							}
							else
							{
								rs485_send_str((unsigned char *)"I ERROR\r\n");
							}
						}
					}
					else
					{
						rs485_send_str((unsigned char*)"S ERROR\r\n");
					}
				}
			}
			else
			{
				rs485_send_str((unsigned char *)"H ERROR\r\n");
			}
		}
		p = strstr((const char*)USART_RX_BUF,(const char*)"SET DAC ");
		if(p != NULL)
		{
			base_p = p;
			p = strstr((const char*)(base_p +8),(const char*)"ADDR:");
			if(p != NULL)
			{
				temp = (int)atoi((const char*)(p+5));
				if((temp>=0)&&(temp<=5))
				{
					
				}
				else
				{
					rs485_send_str((unsigned char *)"param ERROR\r\n");
				}
			}
			p = strstr((const char*)(base_p +8),(const char*)"CH:");
			if(p !=NULL)
			{
				temp = (int)atoi((const char*)(p+3));
				if((temp>=0)&&(temp<=DAC_CHALE-1))
				{
					p = strstr((const char*)(p+1),(const char*)",");
					if(p != NULL)
					{
						temp_f = (float)atof((const char*)(p+1));

						rs485_send_str((unsigned char *)"mcp4728_update OK\r\n");
					}
				}
				else
				{
					rs485_send_str((unsigned char *)"param ERROR\r\n");
				}
			}
		}
		USART_RX_STA=0;  
	}
}
