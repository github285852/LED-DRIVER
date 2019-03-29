#ifndef __SPOT_H
#define __SPOT_H

#define LED_CH		5
#define MAX_CURRENT0		3000
#define MAX_CURRENT1		1600
#define MAX_CURRENT2		1600
#define MAX_CURRENT3		1600
#define MAX_CURRENT4		3000

#define OP1					30//20
#define OP2					120//420

#define Rfb0					0.06
#define Rfb1					0.075
#define Rfb2					0.05
#define Rfb3					0.045
#define Rfb4					0.04
#define Rfb5					0.045
#define Rfb6					0.04

void SysInit(void);
int handle_dmx_data(void);

#endif

