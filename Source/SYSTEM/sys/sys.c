#include "sys.h"
#include "delay.h"
#include "dmx512.h"
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"
#include "stdlib.h"
//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI  
void WFI_SET(void)
{
	__ASM volatile("wfi");		  
}
//关闭所有中断
void INTX_DISABLE(void)
{		  
	__ASM volatile("cpsid i");
}
//开启所有中断
void INTX_ENABLE(void)
{
	__ASM volatile("cpsie i");		  
}
//设置栈顶地址
//addr:栈顶地址
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
