#include "includes.h"
#include "string.h"

#if EN_RDM


unsigned char *RDM_SendBuf=NULL;


#endif

u32 tim_over=0;
u32 break_tim;
//DMXData *DmxData;
//���ջ����� 	
u8 DMX512_DATA_BUF[513]= {1,2,3,4,5,6,8,9,10,11,12,13,14,15,16,17};
u8 DMX512_RX_BUF[513]={1,2,3,4,5,6,8,9,10,11,12,13,14,15,16,17};  	//���ջ���,���513���ֽ�.

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
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
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

#if EN_RDM

	DMX_USART->CR3 |=  0x80;//ʹ��UART3 DMA����
	RDM_DMA_Channle->CPAR = (u32)&USART2->DR;
	RDM_DMA_Channle->CMAR = (u32)RDM_SendBuf;
	
	RDM_DMA_Channle->CCR = 0x0090;    //8λ�������ַ���䣬�洢�����������ȹر�
	RDM_DMA_Channle->CNDTR = 0;       //����512��
	//RDM_DMA_Channle->CCR |= 0x01;     //��������
	RDM_DMA_Channle->CCR |= 0x02; //����DMA�����ж�
	
	NVIC_InitStructure.NVIC_IRQChannel = RDM_DMA_Channle_IRQn;			
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;	 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;					
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								
  NVIC_Init(&NVIC_InitStructure);

#endif
	DMX_RXEN;	//Ĭ��Ϊ����ģʽ
}

u16 tim;
void DMX_EXTI_IRQHandler(void)
{
//static u8 last_sum;
	 //if(PAin(2)==0)//
	u16 t;
	if(DMX_RX_GPIO->IDR&DMX_RX_PIN)//������
	{
			break_tim = tim_over*65535 + DMX_TIM->CNT;
		  if((break_tim>=88)&&(break_tim<=1000000))
			{
				#if EN_RDM
				RDM_RecevieRst(&led_drv_rdm_rev);
				#endif
				Sys.dmx_hanle = 1;
				t = GetSysTime_us();
		
				memcpy(DMX512_DATA_BUF,DMX512_RX_BUF,DMX512_RX_CNT);
				
				tim = GetSysTime_us() - t;
				if(tim>5)
					DMX512_RX_CNT =0;
				DMX512_RX_CNT =0;
//				if(last_sum != TEST_SUM)
//					SUM_CNT++;
//				last_sum = TEST_SUM;
//				TEST_SUM = 0;
	//			tim = break_tim;

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
		//uart_resiver(res);
		//TEST_SUM += res;
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
		
		#if EN_RDM
			RDM_Receive(&led_drv_rdm_rev,DMX512_RX_BUF);
		#endif
	}

} 


void DMX512_handle(void)
{
//	if(Sys.dmx_handle)
//	{

//	}
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

u32 baude=250000,baude_last=250000;
void change_baude(void)
{
	if(baude_last!=baude)
	{
		USART_InitTypeDef USART_InitStructure;
		USART_InitStructure.USART_BaudRate = baude;	//����������
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8λ���ݳ���
		USART_InitStructure.USART_StopBits = USART_StopBits_2;//һ��ֹͣλ
		USART_InitStructure.USART_Parity = USART_Parity_No;///��żУ��λ
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�ģʽ

		USART_Init(USART2, &USART_InitStructure);  //��ʼ������
		baude_last = baude;
	}
}

#if EN_RDM

int MallocRDMTxBuf(unsigned short len)
{
	if(RDM_SendBuf != NULL)
	{
		Debug_printf(">>>%s:last \r\n",__FUNCTION__);
		return 1;
	}
	RDM_SendBuf = (unsigned char *)mymalloc(SRAMIN,len);
	if(RDM_SendBuf == NULL)
	{
		Debug_printf(">>>%s:mymalloc erro!\r\n",__FUNCTION__);
		return 1;
	}
	return 0;
}

int RDMDMASend(unsigned char *buf,unsigned short len)
{
	if(buf==NULL)
		return 1;
	DMX_TXEN;//����ģʽ
	delay_us(200);
	while(RDM_DMA_Channle->CNDTR)//�ȴ���һ�δ������
	{
	}
	while((DMX_USART->SR&0X40)==0);//���ͽ���
	RDM_DMA_Channle->CCR &= ~DMA_CCR1_EN;//�ر�ͨ��
  RDM_DMA_Channle->CMAR = (u32)buf;
	RDM_DMA_Channle->CCR = 0x0090;    //8λ�������ַ���䣬�洢�����������ȹر�
	RDM_DMA_Channle->CNDTR = len;   //����512��
	RDM_DMA_Channle->CCR |= DMA_CCR1_EN;     //��������
	RDM_DMA_Channle->CCR |= 0x02; //����DMA��������ж�
	return 0;
}

void RDM_DMA_Send_IRQHandler(void)
{
	if(RDM_DMA->ISR&RDM_DMA_ISR_TCIF)
	{
		myfree(SRAMIN,RDM_SendBuf);
		RDM_SendBuf =  NULL;
		RDM_DMA->IFCR |= RDM_DMA_ISR_TCIF;//���������ɱ�־
		RDM_DMA_Channle->CCR &= ~DMA_CCR1_EN;     //�رմ���//���ʱ��������ֽ�û������
		while((DMX_USART->SR&0X40)==0);//���ͽ���
		//DMX_TX_GPIO->ODR |= DMX_TX_PIN;
		DMX_RXEN;//����ģʽ
	}
}

#endif
