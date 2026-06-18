	/*****************************************************************************
* File Name:    adom - main.c
* Description:  Ado Master fomodul az rf kezelest vegzi
* Modified:     
* ToDo:         -
*	b0	b1	b2	b3	w
*	67	26	64	91	3043
*max368 96      80      16+80   16+80   8192/887
*****************************************************************************/
#include "main.h"
#include "types.h"
#include "ios.h"
#include "timer.h"
#include "async.h"
#include "delay.h"
#include "rf.h"
#include "iic.h"

// 887 esetén:
__CONFIG(0x332A); // brownout-ENA, reset-lab, hs
// 0x3FFF=BROWNOUT=2.1V

//__CONFIG(0x302A); // brownout-dis, reset-lab, hs // +BODEN -t nem lehet bekapcsolni, mert 3,3V-nal nem mukodik a processzor
// 0x03EFF=BROWNOUT=4V

// ha kell debug kiirashoz
//#define PCK_DEBUG

// nem
#define KETIRANYU_COMM
#define CSATORNAK_MAX	12

void MainInit (void);

#define SOROS_PUFF_LEN	18
#define RX_PUFF_LEN 23
#define CSOMAGHOSSZ 23

static bank2 byte rx_puffer[RX_PUFF_LEN];
static bank2 byte soros_puffer[SOROS_PUFF_LEN];
static bank2 byte soros_puff_p;
static bank2 byte ppm_temp;
static bank2 byte Csomag_sent_db;
static bank2 uint RxRssi;
static bank2 byte TxEnable, Buzzer_dallam, adReadCnt, ch9_ad, ch10_ad, ch11_ad, ch12_ad, pwr_sw_ad, pwr_sw_allas;
static bank2 volatile byte bIeAddr, vetelstartmsmulva,telemetria_rx_elinditva,telemetria_vetel_jon,reg73,reg74;
static bank2 volatile byte KalibralasFolyamatban,TereroAllapotJelzes,BuzzBit;
static bank2 volatile byte TelemetriaCsomagErkezettTmr,TereroJelzesDarabszam;
//bank1 byte txpowerproba;
static bank2 byte EEpromWriteEnable;

#define iea_2dircomm	20								// belso eeprom cimek
#define iea_serial		21
#define iea_dac_l		22							// 3 szint 22,23,24
#define iea_dac_h		25							// 3 szint 25,26,27

#define iea_reg73		30							// korrekcio
#define iea_reg74		31							// korrekcio

static volatile byte is1, is2;

static volatile bank2 byte jj,tmr_led;
// const byte *ss1="TurulRc 783 ";
// const byte *ss2="(c) 2010-2011 Gabor Bazso ";

byte bTmrFlags;
  #define TMR_10MS	128
  #define TMR_100MS	64
  #define TMR_1SEC	32
  #define TMR_1MIN	16
  #define TMR_1HOUR	8
  #define TMR_500MS	64

static bank3 byte bUserTmr100ms;
static bank3 byte tmr2div10;
static bank3 byte tmr2div100ms;
static bank3 byte tmr2div1sec;
static bank3 byte tmr2div60sec;
static bank3 byte tmr2div60min;
static bank3 byte tmr2div500;
volatile bank3 byte RF_SYNC3_b;

static bank3 byte SER_DIV23_b;
static bank3 byte hi_lo_jumper;
static bank1 byte PA_DAC_b_lo[4], PA_DAC_b_hi[4]; // a 3. elem csak indulaskor a tulcimzes miatt kell

byte bHoppIndex;
// 867,8-869,12 23csat 60khz/csat (22*0,06mhz), teljese 1.32mhz savszelesseg
// savkozep: 868,46mhz
// összesen 23 csatorna van, 0..22-ig
#define  HCH_CNT 23
byte const HCHANNELS[HCH_CNT] = {			//0..22 kozotti ertekeket tartalmazhat
   6, 12,  2, 14,  4,  0, 13, 17,  1, 20, 
  15,  8, 16, 22,  5, 11, 21,  9, 19,  7, 
  10, 18, 3
  };

static bank2 byte Csomagszamlalo;

// rssi ertekek
static volatile bank2 byte aRssi1, aRssi2;
//static volatile word rssi_summ, rssi_cnt, rssi_j_cnt, rssi_b_cnt;
//static volatile word trssi_summ, trssi_cnt, trssi_j_cnt, trssi_b_cnt;

static volatile bank2 byte aRssi,aRssi_indB;

static volatile bank2 byte ii;

static volatile bank2 byte rf_event;
enum rf_events  { RF_IDLE, RX, TX, INIT};

static volatile bank2 byte Learn_shift;
static volatile bank2 byte FAIL_SAFE_LEARN;

static volatile bank2 byte config, ketiranyukomm;
  #define conf_jumper1	1
  #define conf_jumper2	2
static volatile bank2 bit bite;
static volatile bank2 byte KapcsolatHiba;


/*****************************************************************************
* HoppNextCh
*****************************************************************************/
void HoppNextCh (void)
  {
   byte b;
   if (++bHoppIndex >= HCH_CNT)
     bHoppIndex = 0;
   b = HCHANNELS[bHoppIndex] + SER_DIV23_b;								// eltoljuk a hopping-ot, ezzel sorozatszam fuggo mikor melyik csatornara ugrik
   if (b >= 23) b-=23;
   spi_write(0x79, b); 		
  }

/*****************************************************************************
* Csomagszamlalo novelese - adas es vetel valtasahoz
*****************************************************************************/
void KovetkezoCsomagszam (void)
  {
  if(Csomagszamlalo) Csomagszamlalo--;
  }

/*****************************************************************************
* interrupt rutin
*****************************************************************************/
void interrupt isr (void) @ 0x04
	{ // ************* 20ms-os csomagküldõ vagy fogadó timer ***********************

	 if (TMR1IF)								//timer 1 interrupt 20ms-ra
		{
		 TMR1init();
		 TMR1IF = 0;
		 if( (telemetria_rx_elinditva == 1) && (ketiranyukomm == 0xff))
			{
                         telemetria_rx_elinditva = 0; // kovetkeyo korben adas lesz
//			 rf_event = RX; // tmr indija a vetelt, nem ez
			}
			else
			{
			 if (TxEnable>0) rf_event = TX;
			}
		}

		// ********************* Idõzitõ timer-ek ************************************
	 if ((TMR2IF) && (TMR2IE))  						//timer 2 int. 1ms-ra
		{
		 TMR2IF = 0;
                 if (vetelstartmsmulva) 
                   {
                    vetelstartmsmulva--;
                    if (vetelstartmsmulva == 0) 
                      {
                       rf_event = RX;
                      }
                   }
		 if (tmr2div10)
			{
			 tmr2div10--;
			}
			 else								//10 ms timer
			{
			 tmr2div10 = 9;
			 bTmrFlags |= TMR_10MS;
  			 if (tmr2div100ms) tmr2div100ms--;
			 else								//100 ms timer
				{
				 tmr2div100ms = 9;
				 bTmrFlags |= TMR_100MS;
				 if ((Buzzer_dallam & 1) == 1) 
					{
					if(!BuzzBit) BUZZ = 1; 
					BuzzBit=1;
					} else 
					{
					if(BuzzBit) BUZZ = 0; 
					BuzzBit=0;
					}
				 Buzzer_dallam = Buzzer_dallam >> 1;

				 if (bUserTmr100ms) bUserTmr100ms--;
				 if (tmr2div500) tmr2div500--;
				 else
					{
					 tmr2div500 = 4;
					 bTmrFlags |= TMR_500MS;
					}
				 if (tmr2div1sec) tmr2div1sec--;
					else							//1sec timer
					{
					 tmr2div1sec = 9;
					 bTmrFlags |= TMR_1SEC;
					 if (tmr2div60sec) tmr2div60sec--;
						else							//1min timer
						{
						 tmr2div60sec = 59;
						 bTmrFlags |= TMR_1MIN;
						 if (tmr2div60min) tmr2div60min--;
							else						//1hour timer
							{
							 tmr2div60min = 59;
							 bTmrFlags |= TMR_1HOUR;
							}
						}
					}
				}
			}
		}
   if (TXIF && TXIE)
		{
		 TXIF = 0; 					//
		 asyncTransmitInt();
		}
   if (RCIF)
		{
		 RCIF = 0;
		 asyncReceiveInt();
		}
	}


/*****************************************************************************
* HexcharToByte - '0'..'F' karakter --> 0x00..0x0F byte
*****************************************************************************/
byte HexcharToByte(byte bHexChar)
	{
	 if (('0' <= bHexChar) && (bHexChar <='9')) //0..9
		{
		 return(bHexChar-'0');
		}
	 if (('A' <= bHexChar) && (bHexChar <='F')) //A..F
		{
		 return (bHexChar-55);
		}
     return(0);
	}

/*****************************************************************************
* asyncBcd
*****************************************************************************/
void asyncBcd (byte value)
	{
	 byte b;
	 b = value >> 4;
	 if ( (b > 9) || ((value & 0x0F) > 9))
		return;
	 asyncPutByte(b+'0');
	 b = value & 0x0F;
	 asyncPutByte(b+'0');					//b-ben maradtak az 1-esek
	}

/*****************************************************************************
* ReadIepromB - bajt olvasasa az aktualis iecimrol, eecim leptetessel
*****************************************************************************/
byte ReadIepromB (void)
	{
	 while (WR)
		;
	 EEADR = bIeAddr++;
	 EEPGD  = 0;
	 RD     = 1;
	 return(EEDATA);
	}

/*****************************************************************************
* WriteIepromB - bajt kiirasa az aktualis eecimre, eecim leptetessel
*****************************************************************************/
void WriteIepromB (byte data)
	{
	 while (WR)
		;
	 EEADR = bIeAddr++;
	 EEDATA = data;
	 EEPGD  = 0;
	 WREN   = 1;
	 GIE    = 0;
	 EECON2 = 0x55;
	 EECON2 = 0xAA;
	 WR     = 1;
	 GIE    = 1;
	 WREN   = 0;
	}

/*****************************************************************************
* MCP4706 programozasa
*
* config: vdd a referencia   nincs power down gain=1x
* VREF1:1 VREF0:x            PD1:0 PD0:0      G:0
*
* a power down -t nem hasznaljuk
*
* DAC register:
* 12 bitbol az also 8-at hasznalja
*
* SDA: RD0
* SCL: RD1
*****************************************************************************/

/*****************************************************************************
* write_adc - bajt irasa az adc-be a pa elõfeszt ez állitja be
*****************************************************************************/
void write_dac(byte data)
	{
	 iicstart();
	 iicputc(0b11000000);													// 7 biten az adc cime, es az r/w(write)
	 iicputc(0b00000000);	 												// pd0,pd1, + felso 4 bit ami most 0 vegig, mert 4706 8 bites
	 iicputc(data);														// 8 bit az adc-nek
	 iicstop();
	}
/*
void adasteszt(void)
{
   spi_read(0x03);                      // dummy_read IS1 register
   spi_read(0x04);                      // dummy_read IS2 register
//	ItStatus1 = SpiReadRegister(0x03);												//read the Interrupt Status1 register
//	ItStatus2 = SpiReadRegister(0x04);												//read the Interrupt Status2 register

	//SW reset   
  	//SpiWriteRegister(0x07, 0x80);														//write 0x80 to the Operating & Function Control1 register 
spi_write(0x07, 0x80); 		// reset all registers to default value			// level 1
   	
	//wait for POR interrupt from the radio (while the nIRQ pin is high)
//	while ( NIRQ == 1);  
DelayMs(20); // diff.az.ado.rf.c-ben a vevo olvassa a statust mikor lett vege a resetnek
	
	//read interrupt status registers to clear the interrupt flags and release NIRQ pin
	//ItStatus1 = SpiReadRegister(0x03);												//read the Interrupt Status1 register
	//ItStatus2 = SpiReadRegister(0x04);												//read the Interrupt Status2 register
spi_read(0x03);                      // dummy_read IS1 register
spi_read(0x04);                      // dummy_read IS2 register
	//wait for chip ready interrupt from the radio (while the nIRQ pin is high)
//	while ( NIRQ == 1);  
DelayMs(20); // diff.az.ado.rf.c-ben a vevo olvassa a statust mikor lett vege a resetnek
	//read interrupt status registers to clear the interrupt flags and release NIRQ pin
//	ItStatus1 = SpiReadRegister(0x03);												//read the Interrupt Status1 register
//	ItStatus2 = SpiReadRegister(0x04);												//read the Interrupt Status2 register
spi_read(0x03);                      // dummy_read IS1 register
spi_read(0x04);                      // dummy_read IS2 register


							//set the physical parameters
	//set the center frequency to 915 MHz
//	SpiWriteRegister(0x75, 0x75);														//write 0x75 to the Frequency Band Select register             
//	SpiWriteRegister(0x76, 0xBB);														//write 0xBB to the Nominal Carrier Frequency1 register
//	SpiWriteRegister(0x77, 0x80);  													//write 0x80 to the Nominal Carrier Frequency0 register

	spi_write(0x75, 0x73); 		  // frequency band select/carrier freq = 867,8mhz
   spi_write(0x76, 0x61); 		  // nominal carrier frequency
   spi_write(0x77, 0x80); 		  // nominal carrier frequency

   //spi_write(0x6D, txpowerproba);               // tx power                                        // power = 0=8dbm,28.4ma/12v, 3=+18dbm,34.5ma/12v
							   
	//set the desired TX data rate (9.6kbps)
	spi_write(0x6E, 0x4E);														//write 0x4E to the TXDataRate 1 register
	spi_write(0x6F, 0xA5);														//write 0xA5 to the TXDataRate 0 register
	spi_write(0x70, 0x2C);														//write 0x2C to the Modulation Mode Control 1 register

	//set the desired TX deviatioin (+-45 kHz)
	spi_write(0x72, 0x48);														//write 0x48 to the Frequency Deviation register 

	
							//set the packet structure and the modulation type
	//set the preamble length to 5bytes  
	spi_write(0x34, 0x0A);														//write 0x0A to the Preamble Length register

	//Disable header bytes; set variable packet length (the length of the payload is defined by the
	//received packet length field of the packet); set the synch word to two bytes long
	spi_write(0x33, 0x02 );													//write 0x02 to the Header Control2 register    
	
	//Set the sync word pattern to 0x2DD4
	spi_write(0x36, 0x2D);														//write 0x2D to the Sync Word 3 register
	spi_write(0x37, 0xD4);														//write 0xD4 to the Sync Word 2 register

	//enable the TX packet handler and CRC-16 (IBM) check
	spi_write(0x30, 0x0D);														//write 0x0D to the Data Access Control register
	//enable FIFO mode and GFSK modulation
	spi_write(0x71, 0x63);														//write 0x63 to the Modulation Mode Control 2 register

							//Set the GPIO's to control the RF switch
   //SpiWriteRegister(0x0C, 0x12);														//write 0x12 to the GPIO1 Configuration(set the TX state)
	//SpiWriteRegister(0x0D, 0x15);														//write 0x15 to the GPIO2 Configuration(set the RX state) 
spi_write(0x0b, 0b00010111); 	// GPIO0 ant1 kapcsolokimenet (diversity kapcsolo) 
   spi_write(0x0c, 0b00011101); 	// GPIO1 trx (adás kapcsolo) direct io=1 vetel // diff.az.ado.rf.c-ben
   spi_write(0x0d, 0x00); 		// GPIO2 - power on reset - for MCLK output        // 0a: direct digital output
   spi_write(0x0e, 0x00); 		// GPIO port use default value                     // all direct io
   
							//set the non-default Si4432 registers
	//set VCO and PLL
	spi_write(0x5A, 0x7F); 													//write 0x7F to the VCO Current Trimming register 
	spi_write(0x59, 0x40);														//write 0x40 to the Divider Current Trimming register 	
	//set Crystal Oscillator Load Capacitance register
	spi_write(0x09, 0xD7);														//write 0xD7 to the CrystalOscillatorLoadCapacitance register

	   spi_write(0x71, 0x38);               // csak vivõ!!! és pn9 a forrasa,,,, modulation mode control 2                       // fifo, gfsk, fd[8]=

	//MAIN Loop
//	while(1)
//	{
		//SET THE CONTENT OF THE PACKET
			//set the length of the payload to 8bytes	
			spi_write(0x3E, 8);													//write 8 to the Transmit Packet Length register		
			//fill the payload into the transmit FIFO
			spi_write(0x7F, 0x42);												//write 0x42 ('B') to the FIFO Access register	
			spi_write(0x7F, 0x55);												//write 0x55 ('U') to the FIFO Access register	
			spi_write(0x7F, 0x54);												//write 0x54 ('T') to the FIFO Access register	
			spi_write(0x7F, 0x54);												//write 0x54 ('T') to the FIFO Access register	
			spi_write(0x7F, 0x4F);												//write 0x4F ('O') to the FIFO Access register	
			spi_write(0x7F, 0x4E);												//write 0x4E ('N') to the FIFO Access register	
			spi_write(0x7F, 0x31);												//write 0x31 ('1') to the FIFO Access register	
			spi_write(0x7F, 0x0D);												//write 0x0D (CR) to the FIFO Access register	

			//Disable all other interrupts and enable the packet sent interrupt only.
			//This will be used for indicating the successfull packet transmission for the MCU
 spi_write(0x05, 0x04);												//write 0x04 to the Interrupt Enable 1 register	
	spi_write(0x06, 0x00);												//write 0x03 to the Interrupt Enable 2 register	
 spi_write(0x05, 0x00);												//write 0x04 to the Interrupt Enable 1 register	
	spi_write(0x06, 0x00);												//write 0x03 to the Interrupt Enable 2 register	
			//Read interrupt status regsiters. It clear all pending interrupts and the nIRQ pin goes back to high.
spi_read(0x03);                      // dummy_read IS1 register
spi_read(0x04);                      // dummy_read IS2 register

			//enable transmitter
			//The radio forms the packet and send it automatically.
			spi_write(0x07, 0x09);												//write 0x09 to the Operating Function Control 1 register

			//enable the packet sent interupt only
			//spi_write(0x05, 0x04);												//write 0x04 to the Interrupt Enable 1 register		
			//read interrupt status to clear the interrupt flags
//spi_read(0x03);                      // dummy_read IS1 register
//spi_read(0x04);                      // dummy_read IS2 register

KalibralasFolyamatban = 10;
Buzzer_dallam = 0b00000001;

	//	}
	}

*/

/* soros porton pc/rol erkezo parancsok ertelmezese */
void SorosParancsErtelmezes(void)
{
 byte i,c,jjj,CRC;
					 if (soros_puffer[0] == 'R')									// read parancs érkezett
						{
						 if (soros_puffer[1] == 'S')								// read serial
							{
						 	 strtosen("S:");
							 asyncPutHexByte(RF_SYNC3_b);
							 strtosen(";");
							 asyncPutHexByte(SER_DIV23_b);
							 strtosen(";");
							 for(i=0; i<3; i++)
								{
								 asyncPutHexByte(PA_DAC_b_lo[i]); // strtosen(";");
								}
							 for(i=0; i<3; i++)
								{
								 asyncPutHexByte(PA_DAC_b_hi[i]); // strtosen(";");
								}
							 strtosen(";");
							 asyncPutHexByte(ketiranyukomm);
							 strtosen(";");
							 asyncPutHexByte(reg73);
							 asyncPutHexByte(reg74);

							 bIeAddr = iea_reg73;   i = ReadIepromB();	// korrekcio
							 asyncPutHexByte(i);
							 bIeAddr = iea_reg74;   i = ReadIepromB();	// korrekcio
							 asyncPutHexByte(i);

							 strtosenD("");
							}
						 if (soros_puffer[1] == 'R')								// RR - read analog inputs 
							{
						 	 strtosen("R:");
							 asyncPutHexByte(pwr_sw_ad);
//							 strtosen(";");
							 strtosenD("");
							}
						 if (soros_puffer[1] == 'V')								// RV - read version 
							{
						 	 strtosen("V:");
							 strtosenD(SOFT_VER);
							}
						}
//CRC ellenorzese
					 if (soros_puffer[0] == 'W')									// write parancs érkezett
						{
						 CRC=0;
						 for (jjj=0; jjj<=(soros_puff_p - 3); jjj++) CRC ^= soros_puffer[jjj];
						 c = HexcharToByte(soros_puffer[soros_puff_p - 2]);
						 c <<= 4;
						 c  |= HexcharToByte(soros_puffer[soros_puff_p - 1]);
						 //TXREG=CRC;	 TXREG=c;
						 if (c!=CRC)
							{
							 soros_puffer[1] = '*';
							 Buzzer_dallam = 0b01010101;
							}
						 if (soros_puffer[1] == 'E')								// write enable
							{
							 EEpromWriteEnable=1;
							 Buzzer_dallam = 0b00000001;
							}
						 if (EEpromWriteEnable==0) // HA NEM VOLT ENGEDELYEZVE A WRITE AKKOR INNEN NEM CSINAL SEMMIT
							{
							 soros_puffer[1] = '*';
							 Buzzer_dallam = 0b01010101;
							}
						 if (soros_puffer[1] == 'S') 								// write serial
							{
							 c = HexcharToByte(soros_puffer[2]);
							 c <<= 4;
							 c  |= HexcharToByte(soros_puffer[3]);
							 bIeAddr = iea_serial; 									// serial config mentese
							 WriteIepromB(c);
							 RF_SYNC3_b = c;
							 SER_DIV23_b = RF_SYNC3_b;
							 while (SER_DIV23_b >=23) SER_DIV23_b-=23;				// 0..22-ig lehetseges
						 	 rf_event = INIT;
							 Buzzer_dallam = 0b00000001;
							}
						 if (soros_puffer[1] == 'A')								// write dac
							{
							 c = HexcharToByte(soros_puffer[2]);
							 c <<= 4;
							 c  |= HexcharToByte(soros_puffer[3]);
							 bIeAddr = iea_dac_l; 									// dac l1 config mentese
							 WriteIepromB(c);
							 PA_DAC_b_lo[0] = c;
	 
							 c = HexcharToByte(soros_puffer[4]);
							 c <<= 4;
							 c  |= HexcharToByte(soros_puffer[5]);
							 bIeAddr = iea_dac_l + 1;								// dac l2 config mentese
							 WriteIepromB(c);
							 PA_DAC_b_lo[1] = c;

							 c = HexcharToByte(soros_puffer[6]);
							 c <<= 4;
							 c  |= HexcharToByte(soros_puffer[7]);
							 bIeAddr = iea_dac_l + 2; 								// dac l3 config mentese
							 WriteIepromB(c);
							 PA_DAC_b_lo[2] = c;

							 c = HexcharToByte(soros_puffer[8]);
							 c <<= 4;
							 c  |= HexcharToByte(soros_puffer[9]);
							 bIeAddr = iea_dac_h; 									// dac h1 config mentese
							 WriteIepromB(c);
							 PA_DAC_b_hi[0] = c;
					 
							 c = HexcharToByte(soros_puffer[0x0A]);
							 c <<= 4;
							 c  |= HexcharToByte(soros_puffer[0x0B]);
							 bIeAddr = iea_dac_h + 1; 								// dac h2 config mentese
							 WriteIepromB(c);
							 PA_DAC_b_hi[1] = c;
					 
							 c = HexcharToByte(soros_puffer[0x0C]);
							 c <<= 4;
							 c  |= HexcharToByte(soros_puffer[0x0D]);
							 bIeAddr = iea_dac_h + 2; 								// dac h3 config mentese h+2 cimre
							 WriteIepromB(c);
							 PA_DAC_b_hi[2] = c;
							 Buzzer_dallam = 0b00000001;
							}
						 if (soros_puffer[1] == 'C')									// write config
							{ 
							 c = HexcharToByte(soros_puffer[2]);
							 c <<= 4;
							 c  |= HexcharToByte(soros_puffer[3]);
							 bIeAddr = iea_2dircomm; 								// byte/config mentese
							 WriteIepromB(c);
							 ketiranyukomm = c;
							 Buzzer_dallam = 0b00000001;
							}
						 if (soros_puffer[1] == 'K')									// write korrekcio
							{ 
							 c = HexcharToByte(soros_puffer[2]);
							 c <<= 4;
							 c  |= HexcharToByte(soros_puffer[3]);
							 bIeAddr = iea_reg73; 									// k config mentese
							 WriteIepromB(c);
							 reg73 = c;
	 
							 c = HexcharToByte(soros_puffer[4]);
							 c <<= 4;
							 c  |= HexcharToByte(soros_puffer[5]);
							 bIeAddr = iea_reg74;								// dac l2 config mentese
							 WriteIepromB(c);
							 reg74 = c;
							 Buzzer_dallam = 0b00000001;
                            }

						 if (soros_puffer[1] == 'R')									// RF cal jel kuldes 10mp-ig
							{ 
							// csak vivõ bekapcsolasa:
							RFInit();		 					// RF IC register initial
    						spi_write(0x79, 0); 		
							spi_write(0x73, reg73);					// frq offset * 312,5hz
							spi_write(0x74, reg74); 					// frq offseth 
							//spi_write(0x6D, txpowerproba);               // tx power                                        // power = 0=8dbm,28.4ma/12v, 3=+18dbm,34.5ma/12v
							LNAon = 0;
							if(hi_lo_jumper == 1) 
								write_dac(PA_DAC_b_hi[pwr_sw_allas]); 
									else 
								write_dac(PA_DAC_b_lo[pwr_sw_allas]); 
							Txvivo(CSOMAGHOSSZ);
							// 10mp delay
							KalibralasFolyamatban = 10;
							 Buzzer_dallam = 0b00000001;
                            }

						if (soros_puffer[1] == 'P')									// WP write power test
							{ 
							 c = HexcharToByte(soros_puffer[2]);
							 c <<= 4;
							 c  |= HexcharToByte(soros_puffer[3]);

//							 txpowerproba = c;
	//						 adasteszt();
							 
							 // Buzzer_dallam = 0b00000001;
							}

						}
}

/*****************************************************************************
* main - szoftver belepesi pont
*****************************************************************************/
void main(void)
	{
	 word wmag;
	 byte i;
	 EEpromWriteEnable=0;
	 IOSinit();
	 TimerInit(); 							// tmr1 
	 asyncInit();
	 tmr_led = 0;
     KapcsolatHiba = 0xff;
	 DelayMs(50);
	 strtosenD("");	// csak crlf
	 strtosenD("Turul 783 remote control started");	
	 strtosenD("");	// csak 0a

//	 strtosenD("2009-2012 Copyright (C) Bazso Gabor");
	 SDN = 0;							// 4432 most indul
  
	 config = conf_jumper1 | conf_jumper2;
         vetelstartmsmulva = 0;
         telemetria_rx_elinditva = 0; 
         KalibralasFolyamatban = 0; // a kalib jelet ez a timer allitja le. x1mp
//		 txpowerproba=3;

	 PortCH9Digital;						// ch9-12 csatorna digit
	 bite = false; 
	 if (bite) { CH10=1;}
		else { CH10=0;}
	 for (jj=0; jj<=15; jj++)					//rst csipogas
		 {
		 if (bite) { CH10=1; bite = false;}
		 else { CH10=0; bite = true;}
		 // BUZZ = 1; 
		 // DelayMs(2);
		 // BUZZ = 0; 
		 DelayMs(24);						//1.szint
		 WDT = !WDT;
		 if (jj==5) RFInit();		 			// RF IC register initial
		 if (CH9  == bite) config &= ~conf_jumper1;			// jumper1 nincs radugva
		 if (CH11 == bite) config &= ~conf_jumper2;			// jumper2 nincs radugva
		 }
	 if (config != 0)						// valamelyik config jumper radugva
		{
		 if (config & conf_jumper1) 				// CH9 jumper
			{
			 ketiranyukomm = 0;
			 // strtosenD("2 way communication: off");
			} else
			{
			 ketiranyukomm = 0xff;
			 // strtosenD("2 way communication: on");
			}
		 bIeAddr = iea_2dircomm; 					// ketiranyu config mentese
		 WriteIepromB(ketiranyukomm);
		 BUZZ = 1;
		 for (;;) { nop; }
		}
	 PortCH9Analog; 								// ch9 csatorna visszakapcsolasa
  
	 bIeAddr = iea_2dircomm; 						// 20-as cimtol van tarolva a config
	 ketiranyukomm = ReadIepromB();				// konfig olvasasa eeprombol

	 bIeAddr = iea_serial; 						// config read
	 RF_SYNC3_b = ReadIepromB();					// ado sorozatszama
	 SER_DIV23_b = RF_SYNC3_b;
	 while (SER_DIV23_b >=23) SER_DIV23_b-=23;		// 0..22-ig lehetseges
 
	 bIeAddr = iea_dac_l;   PA_DAC_b_lo[0] = ReadIepromB();	// dac l1 ertek 
	 bIeAddr = iea_dac_l+1; PA_DAC_b_lo[1] = ReadIepromB();	// dac l2 ertek 
	 bIeAddr = iea_dac_l+2; PA_DAC_b_lo[2] = ReadIepromB();	// dac l3 ertek 
         PA_DAC_b_lo[3]=PA_DAC_b_lo[0];
  
	 bIeAddr = iea_dac_h;   PA_DAC_b_hi[0] = ReadIepromB();	// dac h1 ertek 
	 bIeAddr = iea_dac_h+1; PA_DAC_b_hi[1] = ReadIepromB();	// dac h2 ertek 
	 bIeAddr = iea_dac_h+2; PA_DAC_b_hi[2] = ReadIepromB();	// dac h3 ertek 
         PA_DAC_b_hi[3]=PA_DAC_b_hi[0];

  
	 if ((PA_DAC_b_lo[0] == 0xff) || (PA_DAC_b_hi[0] == 0xff))
		{
		 for (i=0; i<3; i++)
			{
			 bIeAddr = iea_dac_h + i; 				// config mentese
			 WriteIepromB(0xD3);					// 170
			 PA_DAC_b_hi[i] = 0xD3;
	   
			 bIeAddr = iea_dac_l + i; 				// config mentese
			 WriteIepromB(0xD0);
			 PA_DAC_b_lo[i] = 0xD0;
			}
		}
	 if(Jumpi == 0) hi_lo_jumper = 1; else hi_lo_jumper = 0;

	 bIeAddr = iea_reg73;   reg73 = ReadIepromB();	// korrekcio
	 bIeAddr = iea_reg74;   reg74 = ReadIepromB();	// korrekcio
	 if (reg74 > 4) // hibas ertek
		{
		reg73=reg74=0;
		}

	 WDT = 0;							//WDG masik proci fele
	 bHoppIndex = 0; 						// 0-as csatornaval kezdunk
	 Csomagszamlalo = 0;						
	 pwr_sw_ad = 0xff;
	 pwr_sw_allas = 3; // ismeretlen ertek
   
//  rssi_summ = rssi_cnt = rssi_j_cnt = rssi_b_cnt = 0;
//  trssi_summ = trssi_cnt = trssi_j_cnt = trssi_b_cnt = 0;
	 TxEnable = 0;
#ifdef PCK_DEBUG
//  strtosen("<");		// reset jelzes kikuldese							// level 2
  asyncPutHexByte(RF_SYNC3_b);
  version = spi_read(0x01);
  asyncPutHexByte(version);		// ic verzio
  strtosen("*");	
#endif  
	 
	 rf_event = INIT;

  
	 for(;;) 							// fõprogram
		{
  
		 asm("clrwdt");
//****************************** parancsok küldése az rf ic-nek *************************************
		if (! KalibralasFolyamatban) // csak akkor mennek az eventek, ha nincs kalibralas folyamatban
		 switch(rf_event)
			{ 
			 case RF_IDLE:
			 break;

			 case INIT:
			 RFInit();		 					// RF IC register initial
			 spi_write(0x79, 0); 						// nincs afc
			 spi_write(0x73, reg73);					// frq offset * 312,5hz
			 spi_write(0x74, reg74); 					// frq offseth 
			 rf_event = RF_IDLE;
			 break;

			 case RX:
//		strtosenD("v");
			 aRssi_indB = 0;						// telemetria veteli terereje
			 LNAon = 1;
			 HoppNextCh();							// uj csatornara ugras
			 Rx(CSOMAGHOSSZ);
			 KovetkezoCsomagszam();
			 rf_event = RF_IDLE;
			 break;

			 case TX:								//0..19 (elsõ 20)
			 LNAon = 0;
		     spi_write(0x1D, 0); 					// adasban ne legyen afc
			 spi_write(0x73, reg73);					// frq offset * 312,5hz
			 spi_write(0x74, reg74); 					// frq offseth 
	//		 spi_write(0x6D, txpowerproba);               // tx power                                        // power = 0=8dbm,28.4ma/12v, 3=+18dbm,34.5ma/12v
			 
			 if(hi_lo_jumper == 1) 
				 write_dac(PA_DAC_b_hi[pwr_sw_allas]); 
				 else 
				 write_dac(PA_DAC_b_lo[pwr_sw_allas]); 
			 HoppNextCh();							// uj csatornara ugras
			 if ((Csomagszamlalo == 0) && (ketiranyukomm == 0xff))		//csomag tipus megadasa
//			 if (((Csomagszamlalo & 0b11) == 0b11) && (ketiranyukomm == 0xff))		//csomag tipus megadasa
                           {
							tx_puffer[0x00] = 1; 					// ppm csomag, a következõ gps lesz
                            telemetria_vetel_jon = 1; 
							Csomagszamlalo = 23; // 22=440ms, 23=480ms / telemetria csomag
                           } 
			 else
							tx_puffer[0x00] = 0; 					// ppm csomag
			 tx_puffer[0x01] = bHoppIndex; 				// az aktualis hopp freq kuldese
			 if (FAIL_SAFE_LEARN > 0)
				{
				 FAIL_SAFE_LEARN --;
				 tx_puffer[00] = 2; 					// vevo mentse el a fail safe ertekeit
				}
			 Tx(CSOMAGHOSSZ);						// adas elinditása
			 KovetkezoCsomagszam();						// nõ a Csomagszamlalo változo értéke
			 rf_event = RF_IDLE;
			 break;

			 default:
			 rf_event = INIT;
			 break;
			} // case switch
//******************************************* rf int-ek ***********************************************
		 if (NIRQ==0)									// RF ic int
			{
			 is1 = spi_read(0x03);							// nirq okok olvasasa
			 is2 = spi_read(0x04);
			 // tx utáni interrupt-ok
			 if (is1 & 0b00000100)							// packet_sent
				{
				 //        RFInit();			 					// RF IC register initial
				 Tx_Pa_Off();
				 write_dac(0); 							// 0 - pa kikapcsolva
				 Csomag_sent_db++;
   //	                 			 if( (Csomagszamlalo == 20) && (ketiranyukomm == 0xff)) 		// ha kiment a csomag, egybõl vételre kapcsolunk ha kell
				 if( (telemetria_vetel_jon == 1) && (ketiranyukomm == 0xff)) 		// ha kiment a csomag, egybõl vételre kapcsolunk ha kell
				   {
		//				    RFInit(); // 15ms nem fer bele
					
				    // spi_write(0x1D, 0x40); 					// vetelben legyen afc
				    // vetelben ne legyen afc
   				    LNAon = 1;							// lna bekapcsolva
				    vetelstartmsmulva = 3; rf_event = RF_IDLE;
                    telemetria_rx_elinditva = 1; // ne fusson ra a tx int-bol
				   } 
                     else  
                   {
//					spi_write(0x07, 0x03); // visszakapcsolunk tune mode ba
                    rf_event = RF_IDLE;
                   }
				}
			 if (is1 & 0b00000010)							// valid packet received
				{
//		strtosenD("b");
				 send_read_address(0x7f);		 				// olvassuk ki a fifo-t
				 for(ii = 0; ii<CSOMAGHOSSZ; ii++) 
					rx_puffer[ii] = send_command();
				 if (rx_puffer[0] == 3)// vevo -> ado iranyu csomag
					{ // adatfeldolgozás
					 // pc-re erkezo soros csomag osszerakasa
					 strtosen("G:");							//GP csomag
					 asyncBcd(rx_puffer[14]);						//4731.1547
					 asyncPutByte('.');							//fmt 47.51NNNN
					 asyncBcd(rx_puffer[15]);
					 asyncBcd(rx_puffer[16]);
					 asyncBcd(rx_puffer[17]);
					 strtosen(",");
					 if (rx_puffer[22] & 0b00000010) 
							asyncPutByte('N');						//N/S
						else asyncPutByte('S');
					 strtosen(",");			
					 if (rx_puffer[22] & 0b00000001) 
						asyncPutByte('1');						//lon elso bit
						else asyncPutByte('0');
					 asyncBcd(rx_puffer[18]);						//1903.3946
					 asyncPutByte('.');							//19.05NNNN
					 asyncBcd(rx_puffer[19]);
					 asyncBcd(rx_puffer[20]);
					 asyncBcd(rx_puffer[21]);
					 strtosen(",");			
					 if (rx_puffer[22] & 0b00000100) 
						asyncPutByte('E');						//E/W
						else asyncPutByte('W');
					 strtosen(";");							//magassag
					 wmag = rx_puffer[12] + ( rx_puffer[13] * 256 );
					 asyncIntToStr5(wmag);			// altitude 0-65.000 meter 5 digiten kiirva
					 strtosen(";");							//speed-sebesseg 9biten max 512 knot, *1,8
					 asyncIntToStr(rx_puffer[10] + (rx_puffer[11] * 256));	
					 strtosen(";");							//course max 180 (360/2)
					 asyncIntToStr( rx_puffer[9] );				

					 strtosen(";");							//holdak szama
					 asyncPutByte((rx_puffer[22] >> 4) + '0');	//holdak szama
					 strtosen(";"); asyncPutHexByte(rx_puffer[0x05]);			//ad1 - hõfok
					 strtosen(";"); asyncPutHexByte(rx_puffer[0x06]);			//ad2 - tápfesz
					 strtosen(";"); asyncPutHexByte(rx_puffer[0x07]);			//ad0 - analog1
					 strtosen(";"); asyncPutHexByte(rx_puffer[0x08]);			//ad0 - analog2

					 strtosen(";"); RxRssi=rx_puffer[0x02]; asyncIntToStr(RxRssi); 	//veteli terero*2 0.5db a lépésköz
					 strtosen(";"); asyncPutByte(rx_puffer[0x03] + '0');		//antenna
					 strtosen(";"); asyncPutHexByte(rx_puffer[0x04]);			//packet db amit levett
					 strtosen(";"); asyncIntToStr(aRssi_indB); 				//adasnal a telemetria veteli terero*2 0.5db a lépésköz

					 strtosenD("");
					 TereroAllapotJelzes = 0;								// nincs hiba
                     if(rx_puffer[0x04]<8) 
					   {
					   TereroAllapotJelzes = 1;			// gyengül a térerõ
					   KapcsolatHiba = 200;				//gyengül
					   }
					 TelemetriaCsomagErkezettTmr = 18;						// 18*100ms a védõidõ a következõ csomagig.
					 spi_write(0x07, 0x03); // visszakapcsolunk tune mode ba
					} 
				}
			 if (is1 & 0b00000001)						// crc error 
				{
//		strtosenD("c");
				rf_event = RF_IDLE;
				}
			 if (is2 & 0b10000000)						// sync word detect * rssi olvasashoz
				{
				 aRssi1 = spi_read(0x28);                      			// diversity rssi-k 
				 aRssi2 = spi_read(0x29);		                      		// diversity rssi-k
				 if (aRssi1 > aRssi2)
					{
					 aRssi = aRssi1;
					} 
					 else
					{
					 aRssi = aRssi2;
					}
				 aRssi_indB = (122-(aRssi/2));
				 aRssi_indB = aRssi_indB * 2;
				 if ((aRssi & 0b00000001)==0) aRssi_indB = aRssi_indB + 1;
				}
			} 									// RF ic int vege


//***************** soros interrupt ********************************************
		while (asyncHaveData()) // soros adat erkezett
			{
			 byte b;
			 b = asyncGetByte();
			 if (b == 220) soros_puff_p = 0x00;										// tav. ertek kezdete
				 else if (b == 221) 													// tav. ertek vege, feldolgozas
				{ 
				 if (soros_puff_p == CSATORNAK_MAX)									// pont annyi adat jott, mint ahany csat van.
					{
					 for (jj=0; jj < CSATORNAK_MAX; jj++)
						{
						 ppm_temp = soros_puffer[jj];
						 tx_puffer[02+jj] = ppm_temp; 								// 02.. 8db ppm beirasa az ado pufferbe
						}
					 if (TxEnable<3) TxEnable ++;
					 if (tx_puffer[01+9]  == 0xff) tx_puffer[01+ 9] =  ch9_ad;		// 0xff jelzi, hogy ott nincs taviranyito
					 if (tx_puffer[01+10] == 0xff) tx_puffer[01+10] = ch10_ad;
					 if (tx_puffer[01+11] == 0xff) tx_puffer[01+11] = ch11_ad;
					 if (tx_puffer[01+12] == 0xff) tx_puffer[01+12] = ch12_ad;
					 // ado 12 csatornas teszt
					 //            for (jj=0; jj < 12; jj++) asyncPutByte(tx_puffer[02+jj]); //2-3.csatorna, 11-12-csat
					 // nemhasznalt csatornakat kozepre allitjuk
					 for (jj=0; jj < CSATORNAK_MAX; jj++) if (tx_puffer[02+jj] == 0xff) tx_puffer[02+jj] = 100; // minden további ami nincs hasznalva középre áll be.
					 tmr_led = 4; LED = 1;// zöld led jelzi, hogy megy-e a hozzákapcsolt táv
					}
				}
				 else if (b == 222) soros_puff_p = 0x00;								// parancs kezdete
				 else if (b == 223) 													// parancs vége, feldolgozas
				{
				 if (soros_puff_p >= 2)												// min 2 karakteres a parancs
					{
					 SorosParancsErtelmezes();
					}
				}
			 else if (soros_puff_p < SOROS_PUFF_LEN) soros_puffer[soros_puff_p++] = b;
			}

		 if (bTmrFlags & TMR_10MS) 					//10ms
			{
			 word wAd;
			 bTmrFlags &= ~TMR_10MS;
			 switch(adReadCnt)
				 { 
				 case 0:
					 ADCON0 = 0b10000001;				//fosc/32, select RA0/an0, ADON=1
				 break;
				 case 1:
					 ADGO_887 = 1;					// start ad conversion
				 break;
				 case 2:
					 wAd = (ADRESH * 200) >> 8;
					 ch9_ad = wAd & 0xff;
					 ADCON0 = 0b10000101;				//fosc/32, select RA1/an1, ADON=1
				 break;	
				 case 3:
					 ADGO_887 = 1;					// start ad conversion
				 break;
				 case 4:
					 wAd = (ADRESH * 200) >> 8;
					 ch10_ad = wAd & 0xff;
					 ADCON0 = 0b10001001;				//fosc/32, select RA2/an2, ADON=1
				 break;
				 case 5:
					 ADGO_887 = 1;					// start ad conversion
				 break;
				 case 6:
					 wAd = (ADRESH * 200) >> 8;
					 ch11_ad = wAd & 0xff;
					 ADCON0 = 0b10010001;				//fosc/32, select an4, ADON=1
				 break;
				 case 7:
					 ADGO_887 = 1;					// start ad conversion
				 break;
				 case 8:
					 wAd = (ADRESH * 200) >> 8;
					 ch12_ad = wAd & 0xff;
					 ADCON0 = 0b10011101;				//fosc/32, select Re2-an7, ADON=1
				 break;
				 case 9:
					 ADGO_887 = 1;					// start ad conversion
				 break;
				 case 10:
					 wAd = (ADRESH * 200) >> 8;
					 pwr_sw_ad = wAd & 0xff;
					 ADCON0 = 0b10000001;				//fosc/32, select RA0/an0, ADON=1
					 adReadCnt = 0;
					 i = 0;
					 if (pwr_sw_ad < 180) i = 1; // 0-61-199, 61 a közép 1kohm-os ellenállással
					 if (pwr_sw_ad < 10) i = 2; 
					 if (i != pwr_sw_allas)
						{
						 pwr_sw_allas  = i;
						 if (i == 0) Buzzer_dallam = 0b00000011;
						 if (i == 1) Buzzer_dallam = 0b00011011;
						 if (i == 2) Buzzer_dallam = 0b11011011;
						 
						 if(hi_lo_jumper) { strtosen("P:NOEU;");} else strtosen("P:EU;");
						 asyncPutByte(i + 49);
 						 strtosenD("");
						}
				 break;
				}
			 adReadCnt++;
			}
		 if (bTmrFlags & TMR_100MS) 					//100ms
			{
			 bTmrFlags &= ~TMR_100MS;
			 if (tmr_led > 0)
				{ // tmr_led = 6 -rol indul
				 tmr_led--;
				 if (tmr_led == 0) LED = 0;
				}
			 if(TelemetriaCsomagErkezettTmr) 
				{
				 TelemetriaCsomagErkezettTmr--;
				 if(!TelemetriaCsomagErkezettTmr)
					{
					 TereroAllapotJelzes = 2; // teljesen elment a tererõ - amikor lefutott a timer, a status flag/ben beallitjuk hogy csipogjon
					 TereroJelzesDarabszam = 20;
					 KapcsolatHiba = 100;
					}
				}
			 if (ketiranyukomm == 0xff) // buzzer csipog ha nincs kapcsolat
                           {
                            if (KapcsolatHiba !=0xff) 
                              {
								if(KapcsolatHiba) // ha nem 0 hanem több
                                  {
								   if (TereroAllapotJelzes == 0) KapcsolatHiba = 1;// nincs hiba
								   if(KapcsolatHiba == 100 )
										{
										Buzzer_dallam = 0b01010101; // nincs térerõ
										if(TereroAllapotJelzes == 2) KapcsolatHiba = 100 + 30; else KapcsolatHiba = 1;
										TereroJelzesDarabszam--;
										if(!TereroJelzesDarabszam) KapcsolatHiba = 1;
										}
    							   if(KapcsolatHiba == 200 )
										{
										Buzzer_dallam = 0b00000001; // gyengül a térerõ
										if(TereroAllapotJelzes == 1) KapcsolatHiba = 200 + 15; else KapcsolatHiba = 1;
										}
								   KapcsolatHiba--; // csökken
    						      }
                              }
                           }
			}

			/*
			if (bTmrFlags & TMR_1MIN)  					//1 min
				{
				bTmrFlags &= ~TMR_1MIN;
				}

			if (bTmrFlags & TMR_1HOUR)  					//1 h
				{
				bTmrFlags &= ~TMR_1HOUR;
				}
			*/
		 if (bTmrFlags & TMR_500MS)  					//500ms
			{
			 bTmrFlags  &= ~TMR_500MS;
			 if(Csomag_sent_db == 0) rf_event = INIT;
			 Csomag_sent_db = 0;
			 if (TxEnable>0) TxEnable --; // 3-rol megy 0-ra, az kb 2.5mp
			}

		 if (bTmrFlags & TMR_1SEC)  					//1s
			{
			 bTmrFlags  &= ~TMR_1SEC;
			 if (TxEnable==0) 
				{
				 Buzzer_dallam = 0b00000001;
				}
			 Learn_shift = (Learn_shift << 1);
			 if (LEARN == 1) Learn_shift = Learn_shift | 1; // 1 ha nincs megnyomva
			 if (Learn_shift == 0b11000000)					//gomb lenyomva 2-3 masodpercig 0x80 volt
				{
				 Buzzer_dallam = 0b10101010;
				 FAIL_SAFE_LEARN = 5;					//fail-safe learn adas inditasa, 25x kuldi
				}
	                 if (KalibralasFolyamatban) 
	                   {
        	            KalibralasFolyamatban--;
	                    if (KalibralasFolyamatban == 0) 
	                      {
						   Buzzer_dallam = 0b00000001;
        	               rf_event = INIT;
	                      }
			   }
			}								//1s vege
/*****************************************************************************
* soros vonal lefagyasvedelem
*****************************************************************************/
		 if ((OERR==1) || (FERR==1)) // tulfutás
			{
			 SPEN = 0; // ferr torlese:
			 SPEN = 1;
			 i = RCREG;
			 i = RCREG;
			 i = RCREG;
			 }
		} // for ciklus vege
	} // main vege


