#ifndef __PANLE_H
#define __PANLE_H



#define LED_CH		5
#define MAX_CURRENT0		3000
#define MAX_CURRENT1		1600
#define MAX_CURRENT2		1600
#define MAX_CURRENT3		1600
#define MAX_CURRENT4		3000

#define OP1					30//20
#define OP2					120//420

#define Rfb0					0.03
#define Rfb1					0.05
#define Rfb2					0.05
#define Rfb3					0.05
#define Rfb4					0.03


#define IS_A_BOARD			PAin(10)
#define I_CHECK_PIN			PAin(8)

#define A_BOARD_ADDR		1
#define B_BOARD_ADDR		18

void PanleInit(void);

#endif
