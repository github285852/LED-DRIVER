#ifndef __RDM_SLAVE_H
#define __RDM_SLAVE_H

#include "rdm.h"


extern RDM_REV led_drv_rdm_rev;

int RDMSlaveHandle(unsigned char *RdmBuf);
void LedDrvRDMRecevieTask(void);

#endif



