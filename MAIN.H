/*****************************************************************************
* File Name:   main.h
* Description: 
* ToDo:        -
*****************************************************************************/
#ifndef MAIN_H
 #define MAIN_H
 #include "pic.h"

 #define SOFT_VER	"Turul 783 v1.9"
// a teszt készülék: 0F
// #define RF_SYNC3	0x0A				// készülék sorszáma
// #define SER_DIV23	0x0A				//0..22 kozotti ertek, a sorszám alapján

 static volatile unsigned char bank3  ANSEL   @ 0x188;
 static volatile unsigned char bank3  ANSELH  @ 0x189;
 static volatile unsigned char bank1  WPUB    @ 0x95;
 static volatile bit  ADGO_887 @ (unsigned)&ADCON0*8+1;

// SYNC, BRGH, SPBRG azonos
 static volatile unsigned char bank3	BAUDCTL	@ 0x187;
 static volatile bank3 bit		BRG16	@ (unsigned)&BAUDCTL*8+3;
 static volatile unsigned char bank1	SPBRGH	@ 0x9A;

// static volatile unsigned char  bank3 RF_SYNC3_b, SER_DIV23_b;
//extern bank3 unsigned char RF_SYNC3_b, SER_DIV23_b;
extern volatile bank3 unsigned char RF_SYNC3_b;

#endif


