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
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DEBUG_USART,ENABLE);//使能DMX_USART时钟
//	#if GPIO_REMAP
//	GPIO_PinRemapConfig(GPIO_Remap_DMX_USART,ENABLE);
//	#endif

	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin =	DEBUG_TX_PIN; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(DEBUG_TX_GPIO, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = DEBUG_RX_PIN;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(DEBUG_RX_GPIO, &GPIO_InitStructure);//初始化GPIOA.10  
		
	
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_DEBUG_USART,ENABLE);//复位串口
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_DEBUG_USART,DISABLE);//停止复位
 
	USART_InitStructure.USART_BaudRate = 115200;	//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据长度
	USART_InitStructure.USART_StopBits = USART_StopBits_2;//两个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;///奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发模式

  USART_Init(DEBUG_USART, &USART_InitStructure);  //初始化串口
  
	NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQn; //使能串口2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //从优先级2级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能外部中断通道
	NVIC_Init(&NVIC_InitStructure); //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
 
  USART_ITConfig(DEBUG_USART, USART_IT_RXNE, ENABLE);//开中断
	USART_Cmd(DEBUG_USART, ENABLE);                    //使能串口 
	
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

/*
FUNCTION:判定该字符串是命令
INPUT:buf,要判别的字符，cmd 判定的命令。params,参数个数
OUTPUT:param,取得的参数 指针
RETURN：1,是该命令
命令后面直接跟参数，参数之间用','隔开
*/
int IsCmd(unsigned char *buf,char *cmd,u8 params,float *param)
{
	char *p;
	int i = 0;
	p = strstr((const char*)buf,(const char*)cmd);
	if(p != NULL)
	{	
		p += strlen(cmd);
		while(i<params)
		{
			param[i] = atof((const char*)p);
			p = strstr((const char*)p,(const char*)",");
			if( p!= NULL)
			{
				i++;
				p++;
			}
			else
			{
				break;
			}
		}
		return 1;
	}
	else
	{
		return 0;
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
		//PWM输出
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
		/////////////////期望电流输出
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
		if(IsCmd(USART_RX_BUF,"CHECK",0,NULL))
		{
			Sys.AutoCal = 1;
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
	else if(USART_GetITStatus(DEBUG_USART, USART_IT_RXNE) != RESET) //接收到数据
	{	 
		res = USART_ReceiveData(DEBUG_USART); 	//读取接收到的数据
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



