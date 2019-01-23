#include "debug.h"
#include "stm32f10x.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "includes.h"
#include "stdarg.h"

void Debug_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DEBUG_USART,ENABLE);//ʹ��DMX_USARTʱ��
//	#if GPIO_REMAP
//	GPIO_PinRemapConfig(GPIO_Remap_DMX_USART,ENABLE);
//	#endif

	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin =	DEBUG_TX_PIN; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(DEBUG_TX_GPIO, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = DEBUG_RX_PIN;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(DEBUG_RX_GPIO, &GPIO_InitStructure);//��ʼ��GPIOA.10  
		
	
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_DEBUG_USART,ENABLE);//��λ����
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_DEBUG_USART,DISABLE);//ֹͣ��λ
 
	USART_InitStructure.USART_BaudRate = 115200;	//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ���ݳ���
	USART_InitStructure.USART_StopBits = USART_StopBits_2;//����ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;///��żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�ģʽ

  USART_Init(DEBUG_USART, &USART_InitStructure);  //��ʼ������
  
	NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQn; //ʹ�ܴ���2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //�����ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure); //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
 
  USART_ITConfig(DEBUG_USART, USART_IT_RXNE, ENABLE);//���ж�
	USART_Cmd(DEBUG_USART, ENABLE);                    //ʹ�ܴ��� 
	
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
			if(Res!=0x0a)
			{
				USART_RX_STA=0;
			}
			else 
			{
				USART_RX_STA|=0x8000;
			}
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

void debug_send_str(unsigned char *str)
{
	while(*str != 0)
	{
		while((DEBUG_USART->SR&0X40)==0)
		{}
    DEBUG_USART->DR = *str;     
		str++;
	}		
}

unsigned int pwm[32]={0,0};
void uart_duty(void)
{
	char *p;
	unsigned char cmd;
//	unsigned int h;
//	unsigned char s;
//	unsigned temp;
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
				Debug_printf("PWM%d=%d Done\r\n",cmd,pwm[cmd]);
			}
			else
				Debug_printf("ERROR\r\n");
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
					if(Sys.pid_on[cmd]==0)
					{
						//ledpower_init();
					}
				}
				Debug_printf("IOUT%d=%0.1f Done\r\n",cmd,temp_f);
			}
			else
				Debug_printf("ERROR\r\n");
		}	
		
		USART_RX_STA=0;  
	}
}



void DEBUG_USART_IRQHandler(void)
{
	u8 res;	    
	if(USART_GetFlagStatus(DEBUG_USART,USART_FLAG_NE)==SET)
	{
		//USART_ClearFlag(DMX_USART,USART_FLAG_NE); 
		USART_ReceiveData(DEBUG_USART);    
	}
	else if(USART_GetFlagStatus(DEBUG_USART,USART_FLAG_FE)==SET)
	{
		//USART_ClearFlag(DMX_USART,USART_FLAG_FE); 
		USART_ReceiveData(DEBUG_USART);    
	}
	else if(USART_GetFlagStatus(DEBUG_USART,USART_FLAG_ORE)==SET)
	{
		//USART_ClearFlag(DMX_USART,USART_FLAG_ORE); 
		USART_ReceiveData(DEBUG_USART);    
	}	
	else if(USART_GetITStatus(DEBUG_USART, USART_IT_RXNE) != RESET) //���յ�����
	{	 
		res = USART_ReceiveData(DEBUG_USART); 	//��ȡ���յ�������
		uart_resiver(res);
	}
} 

void Debug_printf(char* fmt,...)  
{
	#if DEBUG
	u8 *pbuf,*p;
	va_list ap;
	pbuf = malloc(50);
	if(!pbuf)									
	{
		return ;
	}
	va_start(ap,fmt);
	vsprintf((char*)pbuf,fmt,ap);
	va_end(ap);			
  p = pbuf;
	debug_send_str(p);
	free(pbuf);
	#endif
}



