#ifndef _config_h_
#define _config_h_

#include <avr/io.h>
//config
/* ********************************************************************** */
#define CODE_CID        (0x88)
#define CODE_PID        (0x80)
#define CODE_VID        (0x01)

#define ID0        (0x1234)
#define ID1        (0x5678)

#define CHAR_COMPANY    ("EEB\r\n")
#define CHAR_DESCRIBE   ("7ch ADC,0~5:V,6 7:I!\r\n")
#define def_DeviceAddr  (0x10)
#define UartBaud        (9600)
#define Timer2ClkSel    (5)    //0:none,1:CLK,2:CLK/8,3:CLK/32,4:CLK/64,5:CLK/128,6:CLK/256,7:CLK/1024
#define FrameInterval   (0xFF - (10000/(UartBaud/100) * 28)/(128/8))   //3.5 byte
#define RXARRAYTEMP_MAX (40)
#define RX485()         (PORTD &= ~(1<<2))
#define TX485()         (PORTD |= (1<<2))



#define AUTO_ZERO (-75)

#endif//_config_h_
