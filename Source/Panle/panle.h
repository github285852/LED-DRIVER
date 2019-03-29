#ifndef __PANLE_H
#define __PANLE_H



#define LED_CH		5
#define MAX_CURRENT0		3000
#define MAX_CURRENT1		1600
#define MAX_CURRENT2		1600
#define MAX_CURRENT3		1600
#define MAX_CURRENT4		3000

#define OP1					30.1//20
#define OP2					120.4//420

#define Rfb0					0.033
#define Rfb1					0.05
#define Rfb2					0.05
#define Rfb3					0.05
#define Rfb4					0.033
#define Rfb5					0.045
#define Rfb6					0.04


#define MK0 1
#define MK1 1.5
#define MK2 1.5
#define MK3 1.5
#define MK4 1


#define IS_A_BOARD			PAin(10)
#define I_CHECK_PIN			PAin(8)
#define NEXT_OUT				PBout(7)

#define A_BOARD_ADDR		1
#define B_BOARD_ADDR		18

void SysInit(void);
int handle_dmx_data(void);


extern const float PID_MK_TAB[LED_CH];

#endif
