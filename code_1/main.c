/* ********************************************************************** */
/* 
 * 
 * 
 */
/* ********************************************************************** */

//
/* ********************************************************************** */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <math.h>
#include "ctype.h"
#include "CRC16.h"
#include "config.h"
/* ********************************************************************** */


//
/* ********************************************************************** */
void ErrorDisplay(unsigned int ErrCode);
/* ********************************************************************** */

/* ********************************************************************** */
unsigned int READ_WORD_EEP(unsigned int * addr) {eeprom_busy_wait();return eeprom_read_word(addr);}
void WRITE_WORD_EEP(unsigned int x,unsigned int * addr) {eeprom_busy_wait();eeprom_write_word(addr,x);}
unsigned char READ_BYTE_EEP(unsigned char * addr) {eeprom_busy_wait();return eeprom_read_byte(addr);}
void WRITE_BYTE_EEP(unsigned char x,unsigned char * addr) {eeprom_busy_wait();eeprom_write_byte(addr,x);}
/* ********************************************************************** */

/* ********************************************************************** */
unsigned char ADDR_EEP __attribute__((section(".eeprom"))) = def_DeviceAddr;
unsigned int ID0_EEP  __attribute__((section(".eeprom"))) = ID0;
unsigned int ID1_EEP  __attribute__((section(".eeprom"))) = ID1;
unsigned int ZERO_EEP  __attribute__((section(".eeprom"))) = AUTO_ZERO;

static unsigned char MEASURE_MODE;
static unsigned int  ADJ_ZERO;
static FLOAT V_DATA[8];//0~7 adc检测结果，浮点数。
//static FLOAT V_DATA[];//0~7 adc检测结果，浮点数
static LONG ID;//设备ID号
/* ********************************************************************** */

volatile unsigned char RxMode = 0;
volatile unsigned char RxFrameComplete = 0;
volatile unsigned char WorkMode = 0;
volatile unsigned char RXDataTemp = 0;
volatile MODBUSFRAME RxFrame;
volatile unsigned char DeviceAddr;
unsigned char RxArrayTemp[RXARRAYTEMP_MAX];
volatile unsigned int  ADCResult = 0;
volatile unsigned int  ErrFlag;
volatile unsigned int  ErrCodeDisplay;
volatile unsigned int  T_C;
/* ********************************************************************** */



/* ********************************************************************** */
#define TIMERDELAY
#ifdef TIMERDELAY
/* ---------------------------------------------------------------------- */
#define TimerEnable()       do{TIMSK |= (1 << TOIE1);}while(0)
#define TimerDisable()      do{TIMSK &= ~(1 << TOIE1);}while(0)

#define TimerStop()         do{TCCR1B &= ~((1 << CS12)|(1 << CS11)|1 << CS10);}while(0)
#define TimerStart()        do{TCCR1B |= (1 << CS11);}while(0)

#define SetTimerCount(v)    do{\
                                TCNT1H = (unsigned char)((unsigned int)(v)>>8);\
                                TCNT1L = (unsigned char)((unsigned int)(v)& 0xFF);\
                            }while(0)
/* ---------------------------------------------------------------------- */
volatile unsigned int   TimerCycleI;//
volatile unsigned int   TimerCycleF;//
volatile unsigned char  TimerOver;//
/* ---------------------------------------------------------------------- */
void TimerDelayMs(unsigned int s)
{
    TimerCycleI = s >> 6;                   //TimerCycleN = s/64
    TimerCycleF = 65535 - (s & 0x3F)*1000;  //TimerCycleF = s%64
    TimerOver = 0;
    
    switch(TimerCycleI)
    {
    case 0:
        SetTimerCount(TimerCycleF);
        break;
    case -1:
        TimerDisable();
        TimerOver = 1;
        break;
    default:
        SetTimerCount(1535);
        //TCNT1H = 1535/256;
        //TCNT1L = 1535%256;
        break;
    }
TimerEnable();
TimerStart();
}
/* ---------------------------------------------------------------------- */
SIGNAL(SIG_OVERFLOW1)
{
    
    TimerCycleI--;
    switch(TimerCycleI)
    {
    case 0:
    	SetTimerCount(TimerCycleF);
    	break;
    case -1:
    	TimerStop();
    	TimerDisable();
    	TimerOver = 1;
    	return;
    default:
    	SetTimerCount(1535);
    	break;	
    }
}
#endif
/* ********************************************************************** */






/* ********************************************************************** */
void USART_Init(unsigned int baud)
{
    /* set baud rate */
    UBRRH = (F_CPU/baud/16-1)/256;
    UBRRL = (F_CPU/baud/16-1)%256;
    /* Enable Receiver and Transmitter Enable Receiver interrupt */
    UCSRB =(1<<RXCIE)|(1<<RXEN)|(1<<TXEN);
    /* Set frame format: 8data,2stop bit */
    UCSRC = (1<<URSEL)|
        (0<<UMSEL)|    /* 0:Asynchronous 1:Synchronous */
        (2<<UPM0)|    /* 0:Disable 1:Reserved 2:Even Parity 3:Odd Parity */
        (0<<USBS)|    /* Stop Bit 0:1-bit 1:2-bit */
        (3<<UCSZ0);    /* Data bit 0 ~ 7 : 5,6,7,8,-,-,-,9 */
}
void USART_Transmit(unsigned char data)
{
    while(!(UCSRA & (1<<UDRE)));
    UDR = data;
}

#define putchar USART_Transmit
void printk(char *s)
{
    if(s == NULL || *s == '\0')
        return;
    TX485();
    while(*s != '\0'){
        putchar(*s++);
    }
    RX485();
}
/* ********************************************************************** */
void RxOverrunTimer(void)
{
    /* enabile Timer0 */
    TCNT2 = FrameInterval;
    TIFR |= (1<<TOV2);
    TIMSK |= (1<<TOIE2);
    RxFrameComplete = 0;
}
SIGNAL(SIG_OVERFLOW2)
{
    TIMSK &= ~(1<<TOIE2);
    TIFR |= (1<<TOV2);
#if 1
    switch(RxMode)
    {
        case 1: /* receive Addr only */
            //ErrorDisplay(ERRCODE_OnlyReceiveAddr);
            break;
        case 2: /* receive Addr and FunCode only */
            //ErrorDisplay(ERRCODE_ReceiveDataErr);
            break;
        case 3: /* receive All */
            if(RxFrame.Len < 2){
                //ErrorDisplay(ERRCODE_ReceiveDataErr);
                RxFrame.Data = RxArrayTemp + 2;
                break;
            }
            RxFrameComplete = 1;
            RxFrame.CRCL = *(RxFrame.Data - 2);
            RxFrame.CRCH = *(RxFrame.Data - 1);
            RxFrame.Data = RxArrayTemp + 2;
            break;
        default :
		    break;
            //ErrorDisplay(ERRCODE_ReceiveDataErr);
    }
    /* Stop Timer2 */
    TIMSK &= ~(1<<TOIE2);
    TCNT2 = 0x00;
    RxMode = 0;
#else
    PORTB ^= (1<<0);
    RxOverrunTimer();
#endif
}
/* ********************************************************************** */
SIGNAL(SIG_UART_RECV)
{
    if(! (UCSRA & (1<<RXC))){
        return;
    }
    cli();
    TIMSK &= ~(1<<TOIE2);
    //if(UCSRA & ((1<<FE)|(1<<DOR)|(1<<PE))){
        //ErrorDisplay(ERRCODE_ReceiveBitErr);
    //}
    RXDataTemp = UDR;
#if 0
    TX485();
    putchar(RXDataTemp);
    RX485();
#endif
    switch(RxMode)
    {
        case 0x00:
            RxFrame.Addr = RXDataTemp;
            RxArrayTemp[0] = RXDataTemp;
            RxMode = 0x01;
            break;
        case 0x01:
            RxFrame.FunCode = RXDataTemp;
            RxArrayTemp[1] = RXDataTemp;
            RxFrame.Data = RxArrayTemp + 2;
            RxFrame.Len = 0;
            RxMode = 0x02;
            break;
        case 0x02:
        case 0x03:
            *(RxFrame.Data++) = RXDataTemp;
            RxFrame.Len++;
            if(RxFrame.Len == (RXARRAYTEMP_MAX-2)){
                RxFrame.Data = RxArrayTemp + 2;
                RxFrame.Len = 0;
                //ErrorDisplay(ERRCODE_RXARRAYTEMP_MAXErr);
            }
            RxMode = 0x03;
            break;
    }
    RxOverrunTimer();
    sei();
}
/* ********************************************************************** */
volatile unsigned char DuanAddr = 0;
volatile unsigned char ADC_timerover =0;
#define ADC_Arrya_Len_MAX   (16)
unsigned int ADC_Arrya_Num;
unsigned int ADC_Arrya[8][ADC_Arrya_Len_MAX];
#define T_LEDRefresh    (10)    //(mS)
#define Timer1ClkSel	(5)    //0:none,1:CLK,2:CLK/8,3:CLK/64,4:CLK/256,5:CLK/1024,6:T0 pin falling edge,7:T0 pin rising edge
#define Timer1Count     (0xFF - (T_LEDRefresh * 1000)/(1024/8))

unsigned char OpenADC_Timer(void)
{
	/* clock select clk/32 */
	TCCR0 = Timer1ClkSel;
	/* enabile Timer0 */
	TIMSK |= 0x01;
	TCNT0 = Timer1Count;
	DuanAddr = 0;
	ADC_timerover =0;
	return 0;
}
SIGNAL(SIG_OVERFLOW0)
{
	ADC_timerover = 1;
	DuanAddr++;
	if(DuanAddr>7){
		DuanAddr = 0;
		ADC_Arrya_Num ++;
	}
	TCNT0 = Timer1Count;
}



void ADCInit(void)
{
    ADMUX =  (1<<REFS0)|    /* 0:Aref,1:AVCC,2:-,3:2.56V */
             (1<<ADLAR)|    /* 0:right adjust,1:left adjust */
             (7<<MUX0);     /* 0~7 channel */
    ADCSRA = (1<<ADEN)|     /* ADC enable */
             (0<<ADSC)|     /* ADC start */
             (0<<ADATE)|    /* ADC Auto Trigger Enable */
             (0<<ADIF)|     /* ADC Interrupt Flag ,1:clear */
             (0<<ADIE)|     /* ADC Interrupt Enable */
             (0<<ADPS0);    /* ADC Prescaler Select Bits */
}

unsigned int ADCOutOne(void)
{
    unsigned int i;
    unsigned long sum = 0;

    #define ADC_COUNT 32
    unsigned int ret = 0;
    unsigned char tmpH,tmpL;

    for(i = 0;i < ADC_COUNT; i++)
    {
        ADCSRA |= (1<<ADSC);  //启动转换
        while(ADCSRA & (1<<ADSC));
        tmpL = ADCL;
        tmpH = ADCH;
        ret =(unsigned int)(tmpH << 2);
        ret = ret + (tmpL>>6 & 0x3);
    sum = sum + ret;
    }
    return sum>>5;

}

void ADCOut(void)
{
    unsigned int temp;
    unsigned int tempMAX = 0,tempMIN = 0;
    unsigned long SUM = 0;
	unsigned char ch;
    unsigned char i;
	
	
	ch = DuanAddr;
	
	if(ADC_Arrya_Num >= ADC_Arrya_Len_MAX){
		ADC_Arrya_Num = 0;
	}
		
	ADMUX &= 0xE0;
	ADMUX |= ch;
	_delay_ms(1);
	tempMAX = ADCOutOne();
	tempMIN = ADCOutOne();

	for(i = 0;i < 34;i++)
	{
		temp = ADCOutOne();
	if(temp < tempMIN)
		tempMIN = temp;
		if(temp > tempMAX)
		tempMAX = temp;
	SUM = SUM + temp;
	}
	SUM = SUM - tempMAX - tempMIN;
	temp = (unsigned int)(SUM>>5);
#if 1
	ADC_Arrya[ch][ADC_Arrya_Num] = temp;

	SUM = 0;
	for(i = 0;i < ADC_Arrya_Len_MAX;i++){
		SUM += ADC_Arrya[ch][i];
	}
	temp = SUM / (long)ADC_Arrya_Len_MAX;
#endif
	V_DATA[ch].F =((float) temp * 5.0)/1024.0;
	ADMUX &= 0xE0;
	ADMUX |= ch+1;
}

void RefreshLED(void)
{

    if(ADC_timerover){
	    ADC_timerover = 0;
        ADCOut();
	}
#if 0
	V_DATA[0].F=1.1;
	V_DATA[1].F=2.1;
	V_DATA[2].F=3.1;
	V_DATA[3].F=4.1;
	V_DATA[4].F=5.1;
	V_DATA[5].F=6.1;
	V_DATA[6].F=7.1;
	V_DATA[7].F=8.1;
#endif
#ifdef TIMERDELAY
    if(TimerOver != 0)
    {
        /*  */
        TimerOver = 0;
        TimerDelayMs(1000);
        /* D2 LED 1S flicker */
        PORTB ^= (1<<0);
		PORTB ^= (1<<1);
		//printk(CHAR_COMPANY);
    if(ErrFlag !=0)
            ErrFlag ++;
    }
#endif
}
void SendDataToPC(FUNCODE Fcode, unsigned char * SendTemp, unsigned int Len)
{

    unsigned char ModbusTemp[RXARRAYTEMP_MAX];
    unsigned short RxCRC16;
    unsigned char i;
    unsigned char count;
    ModbusTemp[0] = DeviceAddr;
    ModbusTemp[1] = Fcode;

    if(Fcode != SET_DEVICEADDR){
        count = 3;
        ModbusTemp[2] = (unsigned char)(Len & 0xFF);
    }else{
        count = 2;
    }
    for(i = 0;i < Len;i++){
        ModbusTemp[i+count] = SendTemp[i];
    }
    RxCRC16 = CRC16(ModbusTemp, Len + count);

    TX485();

    for(i = 0;i < Len + count;i++){
        putchar(ModbusTemp[i]);
    }
    
    putchar((unsigned char) (RxCRC16 & 0xFF));
    putchar((unsigned char) (RxCRC16 >> 8));
    
    _delay_ms(10);
    RX485();

}
unsigned char FrameCRC16IsOK(MODBUSFRAME Temp)
{
    unsigned char i;
    unsigned int CRCtemp;
    unsigned char sch[RXARRAYTEMP_MAX];
    sch[0] = Temp.Addr;
    sch[1] = Temp.FunCode;
    for(i=0;i<(Temp.Len-2);i++,Temp.Data++)
        sch[i+2] = *(Temp.Data);
    CRCtemp = CRC16(sch, Temp.Len);

    if(Temp.CRCL != (unsigned char)CRCtemp)
        return 0;
    if(Temp.CRCH != (unsigned char)(CRCtemp>>8))
        return 0;
    return 1;
}

#if 1
void LoadEEPROM(void)
{

    DeviceAddr   = READ_BYTE_EEP(&ADDR_EEP);
    ADJ_ZERO    = READ_WORD_EEP(&ZERO_EEP);

}
#endif
/* ********************************************************************** */
void SystemInit(void)
{

    
//IO Direction
    DDRA = 0x00;
    DDRC = 0xFF;
    DDRB = 0xFF;
    DDRD = 0xFE;
//
    LoadEEPROM();
//enable interrupt 
    sei();
//Display Open
    //OpenDisplayLED();
//485 enabile
    RX485();
//Open Uart
    USART_Init(UartBaud);
//Set Timer2 clk
    /* clock select clk/128 */
    TCCR2 = Timer2ClkSel;
//Enable ADC
    ADCInit();
	OpenADC_Timer();
//Enable watchdog
    wdt_enable(WDTO_1S);
//
#ifdef TIMERDELAY
    TimerOver = 0;
    TimerDelayMs(1000);
#endif

}

int main(void)
{
    unsigned char SendTemp[40];
    unsigned int tempint;

    unsigned char CRCYesOrNo;
    unsigned char tempch;
    unsigned char ReturnID = 1;
    unsigned int i = 0;
	unsigned char ch;
    /* 0:wait receive frame 1:receive frame finish */
    RxFrameComplete = 0;
    /* Init Receive status mode*/
    RxMode = 0;
    /* Error Flag clear */
    ErrFlag = 0;
	_delay_ms(100);
    /* System init */
    SystemInit();
    //RxOverrunTimer();
    ID.L = 0;
    ID.L = READ_WORD_EEP(&ID0_EEP);
    ID.L <<=16;
    ID.L += READ_WORD_EEP(&ID1_EEP);

    while(1)
    {
        if(!RxFrameComplete)
        {
            RefreshLED();
            wdt_reset();
            continue;
        }
        /* clear receive finish flag */
        RxFrameComplete = 0;
        /* Judge Addr is OK */
        if(RxFrame.Addr != DeviceAddr)
        {
			//putchar(DeviceAddr);
            continue;
        }
        /* Judge CRC16 is OK */
        if(!FrameCRC16IsOK(RxFrame))
        {
            //ErrorDisplay(ERRCODE_CRC16Error);
             CRCYesOrNo = 0x80;
        }else{
             CRCYesOrNo = 0;
        }
        tempint = (unsigned int)RxFrame.Data[0];
        tempint <<= 8;
        tempint |= (unsigned int)RxFrame.Data[1];
#if 1
        switch(RxFrame.FunCode){
            case CommunicationTest:
                SendDataToPC(CommunicationTest, RxFrame.Data, RxFrame.Len - 2);
                break;
            case GET_VALUE:
			    if((tempint < 8) && (tempint + RxFrame.Data[3] <= 8)){
                    /* Get ADC data */
					//V_DATA.F = (float)T_C;
					for(i = tempint<<2 ; i < ((tempint + RxFrame.Data[3])<<2) ;i += 4){
						ch = i>>2;
						SendTemp[i]   = V_DATA[ch].ch[3];
						SendTemp[i+1] = V_DATA[ch].ch[2];
						SendTemp[i+2] = V_DATA[ch].ch[1];
						SendTemp[i+3] = V_DATA[ch].ch[0];
					}
					SendDataToPC(GET_VALUE | CRCYesOrNo, SendTemp , RxFrame.Data[3]<<2);
                }else{
					SendTemp[0] = 0x00;
					SendDataToPC(GET_VALUE | 0x80, SendTemp , 1);
                }
                break;
            case GET_DATA:
                switch(tempint)
                {
                    case ADDR_ZERO:/* Get  Data */
                        SendTemp[0] = 0;
                        SendTemp[1] = ADJ_ZERO;
                        SendDataToPC(GET_DATA | CRCYesOrNo, SendTemp , 2);
                        break;
                    default:
                        SendTemp[0] = 0x00;
                        CRCYesOrNo = 0x80;
                        SendDataToPC(GET_DATA | CRCYesOrNo, SendTemp , 1);
                        break;
                }
                break;
            case SET_DEVICEADDR:
                switch(tempint)
                {
                    case ADDR_DEV:/* change device addr */
                        if(RxFrame.Data[3] == 0x00){
                            break;
                        }
                        tempch = RxFrame.Data[3];
                        //SendTemp[0] = DeviceAddr;
                        SendTemp[0] = RxFrame.Data[0];
                        SendTemp[1] = RxFrame.Data[1];
                        SendTemp[2] = 0x00;
                        SendTemp[3] = tempch;
                        SendDataToPC(SET_DEVICEADDR | CRCYesOrNo, SendTemp , 4);
                        DeviceAddr = tempch;
                        WRITE_BYTE_EEP(DeviceAddr,&ADDR_EEP);
                        break;
                    case ADDR_ZERO:
                        ((char *)&ADJ_ZERO)[1] =  RxFrame.Data[2];
                        ((char *)&ADJ_ZERO)[0] =  RxFrame.Data[3];
                        SendTemp[0] = RxFrame.Data[0];
                        SendTemp[1] = RxFrame.Data[1];
                        SendTemp[2] = (char)(ADJ_ZERO>>8);
                        SendTemp[3] = (char)ADJ_ZERO;
                        SendDataToPC(SET_DEVICEADDR | CRCYesOrNo, SendTemp , 4);
                        WRITE_WORD_EEP(ADJ_ZERO,&ZERO_EEP);
                        break;
                    case ADDR_RETURN_ID:
                        if(RxFrame.Data[3] != 0){
                            ReturnID = 1;
                        }else{
                            ReturnID = 0;
                        }
                        //SendTemp[0] = XiaoShuDian;
                        SendTemp[0] = RxFrame.Data[0];
                        SendTemp[1] = RxFrame.Data[1];
                        SendTemp[2] = 0x00;
                        SendTemp[3] = ReturnID;
                        SendDataToPC(SET_DEVICEADDR | CRCYesOrNo, SendTemp , 4);
                        break;
                    default:
                        SendTemp[0] = 0x00;
                        SendDataToPC(SET_DEVICEADDR | 0x80, SendTemp , 1);
                        break;
                }
                break;
            case GET_ID:
                SendTemp[0] = 0x00;
                switch(tempint)
                {
                    case ADDR_CID:
                        SendTemp[1] = CODE_CID;
                        break;
                    case ADDR_PID:
                        SendTemp[1] = CODE_PID;
                        break;
                    case ADDR_VID:
                        SendTemp[1] = CODE_VID;
                        break;
                    case ADDR_FID:
                        SendTemp[1] = MEASURE_MODE;
                        break;
                    default:
                        SendTemp[0] = 0x00;
                        CRCYesOrNo = 0x80;
                        break;
                }
                SendDataToPC(GET_ID | CRCYesOrNo, SendTemp , 2);
                break;
            case GET_CHAR:
                switch(tempint){
                    case SEL_CHAR_COMPANY:
                        printk(CHAR_COMPANY);
                        break;
                    case SEL_CHAR_DESCRIBE:
                        printk(CHAR_DESCRIBE);
                        break;
                    default:
                        SendTemp[0] = 0x00;
                        SendDataToPC(GET_CHAR | 0x80, SendTemp , 1);
                        break;
                }
                break;
            default:
                RxFrame.FunCode |= 0x80;
                SendDataToPC(RxFrame.FunCode, RxFrame.Data, RxFrame.Len - 2);
                //ErrorDisplay(ERRCODE_FunCodeInvalid);
                //PORTD ^= (1<<6);
                break;
        }//switch end
#endif
    }//while end
    //return 0;
}//main end





