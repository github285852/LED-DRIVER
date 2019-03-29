#include "includes.h"


RDM_REV led_drv_rdm_rev;

int GetCmdResponse(RDM_PRE *rx_rdmpre,RDM_MDB *rx_mdb);

int RDMSlaveHandle(unsigned char *RdmBuf)
{
	int i;
	unsigned short sum=0,temp;
	RDM_PRE *rdmpre = (RDM_PRE *)RdmBuf;
	RDM_MDB *mdb = (RDM_MDB *)(RdmBuf+20);
	for(i=0;i<rdmpre->Length;i++)
		sum += RdmBuf[i];
	temp = RdmBuf[i]<<8 | RdmBuf[i+1];
	if(temp != sum)
		return 1;//检验和出错
	if(memcmp(rdmpre->DestID,Sys.UID,ID_LEN))
		return 2;//目的地址不符
	switch(mdb->CmdClass)
	{
		case GET_CMD: GetCmdResponse(rdmpre,mdb);break;
		case SET_CMD: break;
		case DISCOVERY_CMD: break;
		default:return 3;
	}
	return 0;
}

int GetCmdResponse(RDM_PRE *rx_rdmpre,RDM_MDB *rx_mdb)
{
	unsigned short dmx_len,sum=0;
	int i;
	RDM_PRE tx_rdmpre;
	RDM_MDB tx_mdb;
	
	tx_mdb.CmdClass = GET_CMD_RES;
	
	tx_rdmpre.StartCode = STARTCODE;
	tx_rdmpre.SubStartCode = SUBSTARTCOED;
	memcpy(tx_rdmpre.DestID,rx_rdmpre->SourceID,ID_LEN);
	memcpy(tx_rdmpre.SourceID,Sys.UID,ID_LEN);
	tx_rdmpre.TransNo = rx_rdmpre->TransNo;
	tx_rdmpre.Type = rx_rdmpre->Type;
	tx_rdmpre.MessageCnt = 0;
	tx_rdmpre.SubDev = 0;

	switch(rx_mdb->Parameter)
	{
		case PID_STATUS: //返回状态
		{
			tx_mdb.Parameter = PID_STATUS;
			tx_mdb.DataLength = sizeof(STATUS);

			tx_rdmpre.Length = 24 + tx_mdb.DataLength ;
			dmx_len = tx_rdmpre.Length + 2;
			
			if(MallocRDMTxBuf(dmx_len))//DMA 发送问题
				return 1;		
			Sys.status.Code = 0x1234;
			Sys.status.Temp = 205;
			memcpy(RDM_SendBuf + 24,(char *)&Sys.status,tx_mdb.DataLength);
			break;
		}
//		case PID_SET_ADDR: //设置地址
//		{
//			tx_mdb.Parameter = PID_SET_ADDR;
//			tx_mdb.DataLength = 0;

//			tx_rdmpre.Length = 24 + tx_mdb.DataLength ;
//			dmx_len = tx_rdmpre.Length + 2;
//			//
//			Sys.Config.addr = (u8)rx_mdb->pData[0];
//			//
//			if(MallocRDMTxBuf(dmx_len))//DMA 发送问题
//				return 1;	
//			break;
//		}
		default :break;
	}
	
	memcpy(RDM_SendBuf,(char *)&tx_rdmpre,20);
	memcpy(RDM_SendBuf + 20,(char *)&tx_mdb,4);
	for(i=0;i<tx_rdmpre.Length;i++)
	{
		sum += RDM_SendBuf[i];
	}
	RDM_SendBuf[i] = sum >> 8;
	RDM_SendBuf[i+1] = sum & 0x00FF;
	RDMDMASend(RDM_SendBuf,dmx_len);
	return 0;
}

void LedDrvRDMRecevieTask(void)
{
	if(led_drv_rdm_rev.handle)
	{
		RDMSlaveHandle(DMX512_RX_BUF);
		led_drv_rdm_rev.handle = 0;
	}

}	

