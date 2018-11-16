#include "stm32f10x_adc.h"
#include "stm32f10x.h"
#include "includes.h"
#include "adc.h"
#include "math.h"

u16 ADC1_BUFFER[BUFFER_NUMS]={0,0,0};
u32 adc_data;
void ADC1Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//DMA_InitTypeDef DMA_InitStructure;
  //DMA_Channel_TypeDef DMA_Channel1;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);  //?? DMA?? 
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
	ADC_DeInit(ADC1); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);//PA.4Init  //
	
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//	??????
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//???????????? 
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;  //ADC ????? 
	ADC_InitStructure.ADC_NbrOfChannel = ADC_CH_NUMS;  //通道数目 
	ADC_Init(ADC1, &ADC_InitStructure);          //???????????? ADCx   
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_239Cycles5); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_239Cycles5); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_239Cycles5);
	ADC_DMACmd(ADC1,ENABLE);//??ADC1?DMA
	ADC_Cmd(ADC1, ENABLE);                          //????? ADC1 
	ADC_ResetCalibration(ADC1);                          //??????    
	while(ADC_GetResetCalibrationStatus(ADC1));  //???????? 
	ADC_StartCalibration(ADC1);                          //?? AD?? 
	while(ADC_GetCalibrationStatus(ADC1));          //?????? 

	//DMA1_Channel1 NVIC ??
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//?????3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//????3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ????
	NVIC_Init(&NVIC_InitStructure);	//??????????VIC???
	
	DMA1_Channel1->CPAR = (u32)&ADC1->DR;
	DMA1_Channel1->CMAR = (u32)ADC1_BUFFER;
	
	DMA1_Channel1->CCR = 0x15A0;    //16位，外设地址不变，存储递增，必须先关闭
	DMA1_Channel1->CNDTR = BUFFER_NUMS;//BUFFER_NUMS;   //传输100次
	DMA1_Channel1->CCR |= 0x01;     //开启传输
	DMA1_Channel1->CCR |= 0x02; //开启DMA传输中断
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);  //使能软件转换功能   
}

void slect_ch(unsigned char ch)
{
//	SELECT(ch);
	switch(ch)
	{
		case 0:SETA = 0;SETB = 0;SETC = 0;break;
		case 1:SETA = 1;SETB = 0;SETC = 0;break;
		case 2:SETA = 0;SETB = 1;SETC = 0;break;
		case 3:SETA = 1;SETB = 1;SETC = 0;break;
		case 4:SETA = 0;SETB = 0;SETC = 1;break;
		default :break;
	}
}

unsigned int ADCH[LED_CH];
unsigned int ADCM[LED_CH];
unsigned int ADCL[LED_CH];
float I_true[LED_CH];
float I_lpf[LED_CH];
#define A  0.1

u16 last_t,tim1;
void DMA1_Channel1_IRQHandler(void)
{
	int i,j,pos;
  static unsigned char in_ch=0;
	u32 temp_data[ADC_CH_NUMS]={0,0,0};
	u32 t;
	float q;
	if(DMA1->ISR&0x02)//DMA传输完成
	{
		for(i=5;i<MEAN_NUMS;i++)//去掉之前一个值
		{
			for(j=0;j<ADC_CH_NUMS;j++)
			{
				pos = i*3+j;
				temp_data[j] += ADC1_BUFFER[pos];
			}
		}
		ADCL[in_ch] = temp_data[0]/(MEAN_NUMS-5);
		ADCM[in_ch] = temp_data[1]/(MEAN_NUMS-5);	
		ADCH[in_ch] = temp_data[2]/(MEAN_NUMS-5);	
////		adc_data = temp_data/MEAN_NUMS; 
////		adc_data = A*last_data + adc_data*(1-A);//一阶低通滤波
////		last_data = adc_data;

//		ADCL[in_ch] = ADC1_BUFFER[0];
//		ADCM[in_ch] = ADC1_BUFFER[1];	
//		ADCH[in_ch] = ADC1_BUFFER[2];	
		
		if(ADCL[in_ch]>4000)
		{
//			if(ADCM[in_ch]>4000)
//			{
//				I_true[in_ch] = ADCH[in_ch]/4096.0*3.3; //X *0.1*1000
//			}
//			else
			{
				I_true[in_ch] = ADCM[in_ch]/4096.0*3.3/OP1;  // *0.1*1000/OP1
			}
		}
		else
		{
			I_true[in_ch] = ADCL[in_ch]/4096.0*3.3/OP2;//   *0.1*1000/OP2
		}
		I_true[in_ch] *= (1000.0/Rfb);
		if(I_true[in_ch]<0.1)
			I_true[in_ch] = 0;
		in_ch++;
		if(in_ch>=LED_CH)
		{
			in_ch = 0;
			t = GetSysTime_us();
			tim1 = t - last_t;
			last_t = t;
		//	q = (2*3.14159*tim1)/10000.0f;
			for(i=0;i<LED_CH;i++)
			{
				I_lpf[i] += A*(I_true[i] - I_lpf[i]);
			}
			//
		}
		slect_ch(in_ch);//
		//slect_ch(1);//
		DMA1->IFCR|= 0x02;//清楚标志
	}
}

