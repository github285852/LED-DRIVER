/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * ����   �������ƴ�
 * �ļ���  ��scheduler.c
 * ����    ���������
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
**********************************************************************************/
#include "scheduler.h"
#include "delay.h"
#include "includes.h"

s16 loop_cnt;

loop_t loop;
void Loop_check()  //TIME INTTERRUPT 1msִ��һ��
{
	loop.time++; //u16
	loop.cnt_2ms++;
	loop.cnt_5ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;

	if( loop.check_flag == 1)
	{
		loop.err_flag ++;     //ÿ�ۼ�һ�Σ�֤��������Ԥ������(1ms)��û�����ꡣ
	}
	else
	{	
		loop.check_flag = 1;	//�ñ�־λ��ѭ�����������
	}
}

float duty_time[6][2];
void Duty_1ms(void)
{
  
  duty_time[0][1] = GetSysTime_us()/1000000.0f - duty_time[0][0];
  IWDG_Feed();
	duty_time[0][0] = GetSysTime_us()/1000000.0f;
}

void Duty_2ms(void)
{
  
  duty_time[1][1] = GetSysTime_us()/1000000.0f - duty_time[1][0];
	//ledpower_task(duty_time[2][1]);
	duty_time[1][0] = GetSysTime_us()/1000000.0f;
}

void Duty_5ms(void)
{
	duty_time[2][1] = GetSysTime_us()/1000000.0f - duty_time[2][0];
	ledpower_task(duty_time[2][1]);
	DMX512_handle();
		uart_duty();
	duty_time[2][0] = GetSysTime_us()/1000000.0f;
}

	u16 corlo=0;
u8 KEY;
void Duty_10ms(void)
{
  duty_time[3][1] = GetSysTime_us()/1000000.0f - duty_time[3][0];
	find_min_current_task();
	duty_time[3][0] = GetSysTime_us()/1000000.0f;
}
extern long test_pos;
void receiving_dmx_data(void);
void Duty_20ms(void)
{
//  static int t = 0;
//	t++;
//	if(t<10)
//	{
//		return;
//	}
//	t = 0;
	duty_time[4][0] = GetSysTime_us()/1000000.0f;
	receiving_dmx_data();
  duty_time[4][1] = GetSysTime_us()/1000000.0f - duty_time[4][0];
//	ucs8903_update_chanle(ucs8903_data);
//	mcp4728_update_chanle(mcp4728_data);
	//Send_I_to_PC();
}
u16 t;
void change_baude(void);
void Duty_50ms(void)
{
	duty_time[5][0] = GetSysTime_us()/1000000.0f;

  duty_time[5][1] = GetSysTime_us()/1000000.0f - duty_time[5][0];
}

void Duty_Loop(void)   					//�����������Ϊ1ms���ܵĴ���ִ��ʱ����ҪС��1ms��
{
	if( loop.check_flag == 1 )
	{
		Duty_1ms();							//����1ms������
		if( loop.cnt_2ms >= 2 )
		{
			loop.cnt_2ms = 0;
			Duty_2ms();						//����2ms������
		}
		if( loop.cnt_5ms >= 5 )
		{
			loop.cnt_5ms = 0;
			Duty_5ms();						//����5ms������
		}
		if( loop.cnt_10ms >= 10 )
		{
			loop.cnt_10ms = 0;
			
			Duty_10ms();					//����10ms������
		}
		if( loop.cnt_20ms >= 20 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//����20ms������
		}
		if( loop.cnt_50ms >= 50 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//����50ms������
		}	
		loop.check_flag = 0;		//ѭ��������ϱ�־
	}
}


	/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
	

