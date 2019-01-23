#ifndef __DMX512_H
#define __DMX512_H


#define EN_RDM	1


#define DMX_USART											USART2
#define RCC_APB1Periph_DMX_USART			RCC_APB1Periph_USART2
#define RCC_APB1Periph_DMX_TIM				RCC_APB1Periph_TIM6
#define DMX_USART_IRQn								USART2_IRQn

#define DMX_USART_GPIO_REMAP										0
#define GPIO_Remap_DMX_USART					GPIO_FullRemap_USART2

#define DMX_RXEN_GPIO									GPIOA
#define DMX_RX_GPIO										GPIOA
#define DMX_TX_GPIO										GPIOA
#define DMX_RXEN_PIN									GPIO_Pin_1
#define DMX_RX_PIN										GPIO_Pin_3
#define DMX_TX_PIN										GPIO_Pin_2

#define DMX_GPIO_PortSource						GPIO_PortSourceGPIOA
#define DMX_EXTI_PinSource						GPIO_PinSource3
#define DMX_EXTI_Line									EXTI_Line3
#define DMX_EXTI_IRQn									EXTI3_IRQn

#define DMX_TIM												TIM6
#define DMX_TIM_IRQn									TIM6_IRQn

#define DMX_USART_IRQHandler					USART2_IRQHandler
#define DMX_EXTI_IRQHandler						EXTI3_IRQHandler
#define DMX_TIM_IRQHandler						TIM6_IRQHandler


#if EN_RDM

#define RDM_DMA												DMA1
#define RDM_DMA_ISR_TCIF							DMA_IFCR_CTCIF7
#define RDM_DMA_Channle								DMA1_Channel7
#define RDM_DMA_Channle_IRQn					DMA1_Channel7_IRQn
#define RDM_DMA_Send_IRQHandler				DMA1_Channel7_IRQHandler

int MallocRDMTxBuf(unsigned short len);																								
int RDMDMASend(unsigned char *buf,unsigned short len);

extern unsigned char *RDM_SendBuf;

#endif

#define DMX_RXEN				PAout(1) = 0
#define DMX_TXEN				PAout(1) = 1

void dmx512_init(void);
void DMX512_handle(void);
extern unsigned char DMX512_RX_BUF[513];

void uart_duty(void);
void rs485_send_str(unsigned char *str);
void rs485_send_buf(unsigned char *buf,char len);
void uart_resiver(unsigned char Res);

#endif


