#include "stm32f10x.h"
#include "key.h"
#include "sys.h" 
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//������������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////  
								    

#include "key.h"

volatile unsigned char key_status=0; 
unsigned int key_value=0;
volatile unsigned int  key_press_time=0;

void key_service(void)
{
	static unsigned long key_release_time=0;
	static unsigned char key_click=0;
	static unsigned int Continue=0;
	unsigned int ReadData=0;
	unsigned int Release;
	unsigned int Trigger;
	unsigned int key_in;
	key_in = (~(GPIOE->IDR)&0xFFC9);
	if(key_in)
	{
		ReadData = key_in;
	}
	Trigger  = ReadData & (ReadData ^ Continue);   
	Release  = (ReadData ^ Trigger ^ Continue);    
	Continue = ReadData; 	
	if (Trigger) key_press_time = 0; 
	if (Continue)
	{
		if (key_press_time++ > press_time_min)
		{
		    key_status = 254; 
				key_value = Continue;
		}
	}
	if (Release)
	{
		if ((key_press_time > click_time_min) && (key_press_time <= click_time_max)) // ????, ??30ms?500ms
		{
	    key_click++;
		}
		else if ((key_press_time > click_time_max) && (key_press_time <= press_time_min))
		{
	    key_click = 0;
		}
		else if (key_press_time > press_time_min)  
		{
	    key_status = 255;
		}
		key_release_time = 0;
		key_value = Release;
	}
	if (key_release_time++ > click_time_max)
	{
		if (key_click)
		{
	    key_status = key_click;
	    key_click = 0;
		}
	}
}
