
#include "rdm.h"

void RDM_RecevieRst(RDM_REV *rdm)
{
	rdm->handle = 0;
	rdm->RevCnt = 0;
	rdm->sum = 0;
	rdm->len = RDM_MIN_LEN;
	rdm->IsRDM = 0;
}

void RDM_Receive(RDM_REV *rdm,unsigned char *buf)
{
	if(rdm->handle==0)
	{
		if(rdm->RevCnt==0)
		{
			if(buf[0]==0xCC)
			{
				rdm->IsRDM = 1;
			}
		}
		if(rdm->IsRDM)
		{
			if(rdm->RevCnt==2) //¶ÁÈ¡³¤¶È
				rdm->len = buf[2];
			rdm->RevCnt++;
			if(rdm->RevCnt>=rdm->len+2)
			{
				rdm->handle = 1;
				rdm->IsRDM = 0;
			}
		}
	}
}

void rdm_handle(RDM_DAT *rdm)
{
	

}

void RMD_SetPreData()
{

	

}
	
