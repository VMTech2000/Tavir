#include "main.h"
#include "types.h"
#include "ios.h"

// idõk: 1nop=200us
//I2C kommunikáció elkezdése
void iicstart(void)		//startbit
{
        D_SCL=0; 	
	D_SDA=0;		//SCL output, SDA output
	SDA=1;			
	nop;nop;		// min 600ns

        SCL=1; 			
        nop;nop;		// min 600ns
        SDA=0;			//Data 0-bol -->1-be
        nop;nop;		// min 600ns
        SCL=0; 			
}

//I2C kommunikáció befejezése
void iicstop(void)		//stop bit
{
        D_SCL=0; D_SDA=0;	//SCL output, SDA output	
	SDA=0;		
        nop;nop;		// min 600ns

        SCL=1;
        nop;nop;		// min 600ns
        SDA=1;			//
        nop;nop;		// min 600ns
        SCL=0; 			
}

void iicputc(unsigned char data)	//byte kuldese adatvonalra
{
        unsigned char i,temp;

        D_SDA=0;	//SDA kimenet
        SCL=0;		// clk=0 ez nem kell ide
        temp=data;
        for(i=0;i<8;i++)	//byte kikuldese
        {
                SDA=((temp&0x80)!=0);	//ha a baloldali (felso) bit 1
					//200ns
                SCL=1;			
	        nop;nop;		// min 600ns

                SCL=0;			//min 1300ns
		nop;
                temp<<=1;
        }
        D_SDA=1;	//SDA bemenet
	nop;nop;nop;nop;nop;		//1300-hoz
        SCL=1;
        nop;nop;		// min 600ns
        SCL=0;
}

/*
unsigned char iicgetc(unsigned char ack)
{
        unsigned char i,temp;

        SCL=0;
        D_SDA=1;
        temp=0;
        for(i=0;i<8;i++)
        {
                temp<<=1;
                temp|=SDA;
                SCL=1;
                DelayUs(2);
                SCL=0;
                DelayUs(2);
        }
        D_SDA=0;
        SDA=ack;
        SCL=1;
        DelayUs(2);
        SCL=0;
        D_SDA=1;
        return(temp);
}

*/