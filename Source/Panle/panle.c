#include "includes.h"

const unsigned char A_DUID[ID_LEN] = {'A','B','O','A','R','D'};
const unsigned char B_DUID[ID_LEN] = {'B','B','O','A','R','D'};


void PanleInit(void)
{
	if(IS_A_BOARD)
	{
		Sys.Config.addr = A_BOARD_ADDR;
		memcpy(Sys.UID,A_DUID,ID_LEN);
	}
	else
	{
		Sys.Config.addr = B_BOARD_ADDR;
		memcpy(Sys.UID,B_DUID,ID_LEN);
	}
	
}

