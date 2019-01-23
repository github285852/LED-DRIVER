#ifndef __RDM_H
#define __RDM_H


#define RDM_MIN_LEN				24
#define MDB_OFFSET				20
#define MDB_DATA_OFFSET		24
#define SUM_LEN						2
#define RDM_ID_LEN			6
#define ID_LEN			6
#define DMXSERIAL_MAX 512                  ///< The max. number of supported DMX data channels
#define DMXSERIAL_MIN_SLOT_VALUE 0         ///< The min. value a DMX512 slot can take
#define DMXSERIAL_MAX_SLOT_VALUE 255       ///< The max. value a DMX512 slot can take
#define DMXSERIAL_MAX_RDM_STRING_LENGTH 32 ///< The max. length of a string in RDM

#define STARTCODE			0xcc
#define SUBSTARTCOED			0x01

#define GET_CMD					0
#define GET_CMD_RES			0
#define SET_CMD					1
#define SET_CMD_RES			1
#define DISCOVERY_CMD					2
#define DISCOVERY_CMD_RES			2


#define PID_STATUS			0


typedef struct{
  unsigned char    	StartCode;    // Start Code 0xCC for RDM
  unsigned char     SubStartCode; // Start Code 0x01 for RDM
  unsigned char     Length;       // packet length
  unsigned char     DestID[ID_LEN];
  unsigned char     SourceID[ID_LEN];

  unsigned char     TransNo;     // transaction number, not checked
  unsigned char     Type;    // ResponseType
  unsigned char     MessageCnt;     // 
  unsigned short		SubDev;      // sub device number (root = 0) 
}RDM_PRE; //Êý¾ÝÍ·


typedef struct{
	unsigned char     CmdClass;     // command class
	unsigned short 		Parameter;	   // parameter ID
	unsigned char    	DataLength;   // parameter data length in bytes
	unsigned char     *pData;   // data byte field
}RDM_MDB; // struct RDMDATA


typedef struct{
	RDM_PRE pre;
	RDM_MDB mdb;
	unsigned short sum;
}RDM_DAT; // struct RDMDATA

typedef struct{
	unsigned char IsRDM;
	unsigned char handle;
	unsigned short sum;
	unsigned char len;
	unsigned char RevCnt;	
}RDM_REV;

typedef struct{
	unsigned short Code;
	unsigned short Temp;
}STATUS; // struct RDMDATA


extern RDM_PRE RDMPre;


void RDM_RecevieRst(RDM_REV *rdm);
void RDM_Receive(RDM_REV *rdm,unsigned char *buf);

#endif







