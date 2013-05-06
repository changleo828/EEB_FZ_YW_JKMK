#ifndef _ctype_h_
#define _ctype_h_

//type
/* ********************************************************************** */
#define uchar unsigned char
enum{
    ADDR_DEV = 0x0010,
    ADDR_CID,
    ADDR_PID,
    ADDR_VID,
    ADDR_FID,
    ADDR_ZERO,
    ADDR_RETURN_ID,
    ADDR_MAX,
};
enum SELECT_GET_CHAR{
    SEL_CHAR_COMPANY,
    SEL_CHAR_DESCRIBE
};
typedef enum _ErrorCode{
    ERRCODE_DeviceAddrErr = 1,
    ERRCODE_CRC16Error,
    ERRCODE_OnlyReceiveAddr,
    ERRCODE_ReceiveDataErr,
    ERRCODE_FunCodeInvalid,
    ERRCODE_ReceiveBitErr,
    ERRCODE_RXARRAYTEMP_MAXErr,
} ERRORCODE;
typedef enum _FunCode{
    GET_VALUE = 3,
    GET_DATA = 4,
    SET_DEVICEADDR = 6,
    CommunicationTest = 0x64, 
    SET_VALUE = 0x65,
    GET_ID = 0x66,
    GET_CHAR,
} FUNCODE;
typedef struct _MODBUSFrame{
    unsigned char Addr;
    FUNCODE FunCode;
    unsigned char *Data;
    unsigned int  Len;
    unsigned char CRCH;
    unsigned char CRCL;
} MODBUSFRAME;
typedef struct _YiBiaoReg{
    unsigned int addr;
    unsigned int data;
} YI_BOAI_REG;
typedef union {
    unsigned char ch[4];
    float F;
    //double D;
} FLOAT;
typedef union {
    unsigned char ch[4];
    long L;
    //double D;
} LONG;
#ifndef NULL
#define NULL (void *)(0)
#endif
/* ********************************************************************** */

#endif
