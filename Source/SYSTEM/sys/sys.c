#include "sys.h"
#include "delay.h"
#include "dmx512.h"
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"
#include "stdlib.h"
//THUMBָ�֧�ֻ������
//�������·���ʵ��ִ�л��ָ��WFI  
void WFI_SET(void)
{
	__ASM volatile("wfi");		  
}
//�ر������ж�
void INTX_DISABLE(void)
{		  
	__ASM volatile("cpsid i");
}
//���������ж�
void INTX_ENABLE(void)
{
	__ASM volatile("cpsie i");		  
}
//����ջ����ַ
//addr:ջ����ַ
__asm void MSR_MSP(u32 addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}

#define DEBUG 	1

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
	rs485_send_str(p);
	//Picture_ShowString(*pic,x,y,pic->w-x,pic->h-y,size,pbuf);	
	free(pbuf);
	#endif
}
