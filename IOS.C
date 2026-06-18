#include "ios.h"

/*****************************************************************************
* init()
*****************************************************************************/
//Pinek beállítása
void IOSinit(void)
  {
//            76543210        7      6      5      4      3      2      1      0 1-in,0-out
   TRISA  = 0b00111111;	//                              
   PORTA  = 0b00000000;
   TRISB  = 0b00000011;	//                 Pwr2   Pwr1          LED   GPIO2  NIRQ  
   PORTB  = 0b00000000;
   TRISC  = 0b10100000;	//   Rx      Tx   SD_IN   SD_OUT  SCLK  NSEL   SDN    BUZZ
   PORTC  = 0b00000000; //                                                    
   TRISD  = 0b10000000;	//   Jumpi
   PORTD  = 0b00000001; //                                       LNAON  SCL    SDA
   TRISE  = 0b00000110; //                                       Pwr_sw
   PORTE  = 0b00000000; //                                       LCD_Eng LCD_RW LCD_RS

   ADCON1 = 0b00000000;	        //balra igazitva, vss,vdd
   PortCH9Analog;
//   ADCON0 = 0b10000001; //   ANSEL  = 0b00010111;

   ANSELH = 0x00;

   RBPU   = 1;			//PORTB pullup dis

   INTEDG = 0; 			// rb0 falling edge interrupt
   INTE   = 0;			// rb0 int dis
   SDN    = 1;			// 4432 resetben
  }   

