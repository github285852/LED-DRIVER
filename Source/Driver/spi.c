#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "malloc.h"
#include "st7735s.h"
#include "stdlib.h"

u8 DMAING=0;
void SPI3_init(void)
{
  SPI_InitTypeDef  SPI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE );//SPI2ʱ��ʹ�� 	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2,ENABLE);
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;  //����SPI�������˫�������ģʽ:SPI����Ϊ����
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI3, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI3, ENABLE); //ʹ��SPI����
	
//	SPI3_ReadWriteByte(0xAA);//��������		
	
	//DMA2 ���� SPI3TX ͨ����
	                   //�洢����ַ����   �洢����
	DMA2_Channel2->CCR =  DMA_CCR1_DIR | DMA_CCR1_MSIZE_0 | DMA_CCR2_TCIE;//���ж�
	DMA2_Channel2->CPAR = SPI3_BASE + 0x0C; //�����ַ
	
	SPI3->CR2 |= SPI_CR2_TXDMAEN;// ����DMA����
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel2_IRQn;			
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;					
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								
  NVIC_Init(&NVIC_InitStructure);
	
	
	//DMA2_Channel2->CMAR = 
	//
	//DMA2_Channel2->CNDTR = ��Ŀ
  //DMA2_Channel2->CCR |= DMA_CCR1_EN;//ʹ��ͨ��
}

void SPI_SET_8(void)
{
	SPI3->CR1 &= ~SPI_CR1_SPE;//�ر�SPI
	SPI3->CR1 &= ~SPI_CR1_DFF;
	SPI3->CR1 |= SPI_CR1_SPE;//��SPI
}

void SPI_SET_16(void)
{
	SPI3->CR1 &= ~SPI_CR1_SPE;//�ر�SPI
	SPI3->CR1 |= SPI_CR1_DFF;
	SPI3->CR1 |= SPI_CR1_SPE;//��SPI 
}

//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI3_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;		
	while(DMAING);
	while(DMA2_Channel2->CNDTR)
	{
	//	retry++;
	//	if(retry>200)return 0;
	}
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
		retry++;
		if(retry>200)return 0;
	}		
	while((SPI3->SR&SPI_I2S_FLAG_BSY)|| ((SPI3->SR&SPI_I2S_FLAG_TXE)==0)); //���ָ����SPI��־λ�������:���ͻ���ձ�־λ	
	SPI_SET_8();
	SPI_I2S_SendData(SPI3, TxData); //ͨ������SPIx����һ������
	retry=0;
/*
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET) //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI3); //����ͨ��SPIx������յ�����
*/		
}

u8 SPI3_write_bytes(uint32_t *addr,u16 num)
{
	u8 retry=0;
	while(DMAING);
	while(DMA2_Channel2->CNDTR)
	{
		//retry++;
	//	if(retry>200)return 0;
	}
	while((SPI3->SR&SPI_I2S_FLAG_BSY)|| ((SPI3->SR&SPI_I2S_FLAG_TXE)==0)) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
	//	retry++;
	//	if(retry>200)return 0;
	}	
	while(DMA2_Channel2->CNDTR)
	{
	//	retry++;
	//	if(retry>200)return 0;
	}
	SPI_SET_8();
	DMA2_Channel2->CCR &= ~DMA_CCR1_EN;//�ر�ͨ��
	DMA2_Channel2->CCR |=  DMA_CCR1_DIR | DMA_CCR1_MINC;//��ַ����
	DMA2_Channel2->CMAR = (uint32_t) addr;
	DMA2_Channel2->CNDTR = num;
	DMA2_Channel2->CCR |= DMA_CCR1_EN;//ʹ��ͨ��
	DMAING = 1;
	return 1;
}

u8 SPI3_write_half_words(uint32_t *addr,u16 num)
{
	u8 retry=0;
	while(DMAING);
	while(DMA2_Channel2->CNDTR)
	{
	//	retry++;
	//	if(retry>200)return 0;
	}
	while((SPI3->SR&SPI_I2S_FLAG_BSY)|| ((SPI3->SR&SPI_I2S_FLAG_TXE)==0)) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
		//retry++;
		//if(retry>200)return 0;
	}	
	while(DMA2_Channel2->CNDTR)
	{
	//	retry++;
	//	if(retry>200)return 0;
	}
	SPI_SET_16();
	DMA2_Channel2->CCR &= ~DMA_CCR1_EN;//�ر�ͨ��
	DMA2_Channel2->CCR |=  DMA_CCR1_DIR | DMA_CCR1_MINC | DMA_CCR1_MSIZE_0 | DMA_CCR1_PSIZE_0;//16λ����ַ����
	DMA2_Channel2->CMAR = (uint32_t) addr;
	DMA2_Channel2->CNDTR = num;
	DMA2_Channel2->CCR |= DMA_CCR1_EN;//ʹ��ͨ��
	DMAING = 1;
	return 1;
}

//������ͬһ��16λ����
u8 SPI3_send_same_half_word(uint32_t *addr,u16 num)
{
	u8 retry=0;
	while(DMAING);
	while(DMA2_Channel2->CNDTR)
	{
		//retry++;
	//	if(retry>200)return 0;
	}
	while((SPI3->SR&SPI_I2S_FLAG_BSY)|| ((SPI3->SR&SPI_I2S_FLAG_TXE)==0)) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
		//retry++;
		//if(retry>200)return 0;
	}	
	while(DMA2_Channel2->CNDTR)
	{
		//retry++;
	//	if(retry>200)return 0;
	}
	SPI_SET_16();
	DMA2_Channel2->CCR &= ~DMA_CCR1_EN;//�ر�ͨ
	DMA2_Channel2->CCR |=  DMA_CCR1_DIR | DMA_CCR1_MSIZE_0 | DMA_CCR1_PSIZE_0; //�洢16λ��
	DMA2_Channel2->CMAR = (uint32_t) addr;
	DMA2_Channel2->CNDTR = num;
	DMA2_Channel2->CCR |= DMA_CCR1_EN;//ʹ��ͨ��
	DMAING = 1;
	return 1;
}

//SPI �ٶ����ú���
//SpeedSet:
//SPI_BaudRatePrescaler_2   2��Ƶ   
//SPI_BaudRatePrescaler_8   8��Ƶ   
//SPI_BaudRatePrescaler_16  16��Ƶ  
//SPI_BaudRatePrescaler_256 256��Ƶ 
void SPI3_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI3->CR1&=0XFFC7;
	SPI3->CR1|=SPI_BaudRatePrescaler;	//����SPI2�ٶ� 
	SPI_Cmd(SPI3,ENABLE); 
} 
extern u16 *LCD_BUF;
//extern Picture pic;
void DMA2_Channel2_IRQHandler(void)
{
	if(DMA2->ISR&DMA_ISR_TCIF2)
	{
//		if(pic.data != 0 )
//		{
//			//free(pic.data);
//		}
		DMAING = 0;
		//CS = 1;
		DMA2->IFCR |= DMA_IFCR_CTCIF2;//���������ɱ�־
	}
}