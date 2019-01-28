
#ifndef __ADC_H
#define __ADC_H

#define MEAN_NUMS 10
#define ADC_CH_NUMS	3
#define BUFFER_NUMS  (ADC_CH_NUMS*MEAN_NUMS)

#define SETA   PCout(0)
#define SETB   PCout(1)
#define SETC   PCout(2)
#define SELECT(x)				{GPIOC->ODR &= 0xFFF8;GPIOC->ODR |= x;}
//#define OP1					20
//#define OP2					420
//#define Rfb					0.1
void ADC1Init(void);


extern unsigned int adc_data;


#endif





