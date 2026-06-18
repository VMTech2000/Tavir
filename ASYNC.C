/*****************************************************************************
* File Name:    async.c
* Project:      
* Description:  hadrware aszinkron port
* Modified:     
* ToDo:         
*****************************************************************************/
#include "main.h"
#include "ios.h"
#include "types.h"
#include "async.h"

/*****************************************************************************
* modul globalis valtozok
*****************************************************************************/
bank1 byte puffIn[ASYNC_INPUF_LEN];
bank1 byte bInStart;
bank1 byte bInEnd;
bank1 byte bInCnt;

bank3 byte puffOut[ASYNC_OUTPUF_LEN];
bank1 byte bOutStart;
bank1 byte bOutEnd;
bank1 byte bOutCnt;

/*****************************************************************************
* asyncReceiveInt - aszinkron kuldes megszakitas rutin
*****************************************************************************/
void asyncReceiveInt (void)
  {
   byte BejovoBajt;
   BejovoBajt = RCREG;
   if (bInCnt < ASYNC_INPUF_LEN)  //ha fer meg a pufferbe, beleirja
     {
      puffIn[bInEnd++] = BejovoBajt;
      if (bInEnd >= ASYNC_INPUF_LEN)  //forgas
        bInEnd = 0;
      bInCnt++;
     }
  }  

/*****************************************************************************
* asyncHaveData - az aszinkron pufferben van-e adat (true/false)
*****************************************************************************/
byte asyncHaveData(void)
  {
/*
   if (bInCnt > 0)
     return(true);
   return(false);
*/
   return(bInCnt);
  }

/*****************************************************************************
* asyncGetByte - az aszinkron in FIFO-bol byte kiemelese
*****************************************************************************/
byte asyncGetByte(void)
  {
   byte bResult;
   if (bInCnt == 0)
     return (0);
   RCIE = 0;
   bResult = puffIn[bInStart++];
   if (bInStart >= ASYNC_INPUF_LEN)  //forgas
      bInStart = 0;
   bInCnt--;
   RCIE = 1;
   return(bResult);
  }

/*****************************************************************************
* asyncTransmitInt - aszinkron kuldes megszakitas rutin
*****************************************************************************/
  void asyncTransmitInt (void)
    {
     if (bOutCnt) //van meg mit kuldeni
       {
        TXREG = puffOut[bOutStart++];
        if (bOutStart >= ASYNC_OUTPUF_LEN)  //forgas
          bOutStart = 0;
        bOutCnt--;
       }
     else //az elozo volt az utolso byte
       {
        TXIE = 0;
       }
    }

/*****************************************************************************
* asyncOutNotFull - az aszinkron kimeno puffer nincsen tele
*****************************************************************************/
/*
byte asyncOutNotFull(void)
  {
   if (bOutCnt < ASYNC_OUTPUF_LEN)
     return(true);
   return(false);
  }
*/
/*****************************************************************************
* asyncOutpufEmpty - az aszinkron kimeno puffer ures-e (true/false)
*****************************************************************************/
/*
byte asyncOutpufEmpty(void)
  {
   if (bOutCnt == 0)
     return(true);
   return(false);
  }
*/
/*****************************************************************************
* asyncPutByte - aszinkron kuldo eljaras (puffer/TXREG)
*****************************************************************************/
  void asyncPutByte (byte ByteToSend) //valos 19200
    {
     while (bOutCnt >= ASYNC_OUTPUF_LEN)  //ha tele a puffer, var
       ;
     if (TXIE) //kuldes van folyamatban --> pufferbe kerul
       {
        TXIE = 0;  //megszakitas ne usson be rosszkor
        puffOut[bOutEnd++] = ByteToSend;
        if (bOutEnd >= ASYNC_OUTPUF_LEN)  //forgas
          bOutEnd = 0;
        bOutCnt++;
       }
     else
       {
        TXREG = ByteToSend;
       }
     TXIE = 1;
    }

/*****************************************************************************
* asyncPutHexChar - 0..0x0F byte kikuldese '0'..'F' karakterkent
*****************************************************************************/
/*
void asyncPutHexChar(byte bHalfByte)
  {
   byte bChar;
   if (bHalfByte <= 9) //0..9
     {
      bChar = bHalfByte + 48;
     }
   else //0x0A..0x0F
     {
      bChar = bHalfByte + 55;
     }
   asyncPutByte(bChar);
  }
*/
/*****************************************************************************
* asyncPutHexByte - byte kikuldese '00'..'FF' karakterkent
*****************************************************************************/

void asyncPutHexByte(byte bByte)
  {
   byte bChar,bHalfByte;

   bHalfByte = bByte >> 4;
   if (bHalfByte <= 9) //0..9
     {
      bChar = bHalfByte + 48;
     }
   else //0x0A..0x0F
     {
      bChar = bHalfByte + 55;
     }
   asyncPutByte(bChar);

   bHalfByte = bByte & 0x0F;
   if (bHalfByte <= 9) //0..9
     {
      bChar = bHalfByte + 48;
     }
   else //0x0A..0x0F
     {
      bChar = bHalfByte + 55;
     }
   asyncPutByte(bChar);
  }


/*****************************************************************************
* asyncPutBcdByte - 0x00..0x63 byte kikuldese '00'..'99' karakterkent
*****************************************************************************/
/*
void asyncPutBcdByte(byte bByte)
  {
   byte bTizesek,bChar;
   bTizesek = bByte / 10;
   bChar = bTizesek + '0';
   asyncPutByte(bChar); //tizesek
   bChar = bByte - (bTizesek * 10) + '0';
   asyncPutByte(bChar); //egyesek
  }
*/

/*****************************************************************************
* asyncPutStr - max 80 len. string kuldese soroson, vege = 0x00 vagy '|'
*****************************************************************************/
void strtosen(byte const *stri)
  {
   byte l=0;
   while((stri[l] != 0x00) && (l<80)) 
     {
      asyncPutByte(stri[l++]);
     }
  }

/*****************************************************************************
* strtosenD - max 80 len. string kuldese soroson, vege = 0x00 vagy '|'
* kukuld plusszban 0x0D, 0x0A chart.
*****************************************************************************/
void strtosenD(byte const *stri)
{
  byte l=0;
  while((stri[l] != 0x00) && (l<80)) 
    {
     asyncPutByte(stri[l++]);
    }
//  asyncPutByte(0x0D);
  asyncPutByte(0x0A);
}

/*****************************************************************************
* asyncIntToStr - szam kikuldese sztringkent, csak hasznos szamjegyek
*****************************************************************************/
  void asyncIntToStr(uint szam)
    {
     byte bontva[5];
     uint tizh = 10000;//4 digitet ir ki
     byte i,j;
     for (i=0; i<5; i++)
       bontva[i] = '0';
     i = 0;
     while (szam > 0)
       {
        while (szam >= tizh)
          {
           szam-=tizh;
           (bontva[i])++;
          }
        if (tizh >= 10) 
          tizh /= 10;
        i++;
       }
     i = 0;
//     while (bontva[i] == '0')
//       i++;
     i=2; // 3digitet ir ki
     for (j=i; j<5; j++)
       asyncPutByte(bontva[j]);
    }

/*****************************************************************************
* asyncIntToStr2 - szam kikuldese sztringkent, csak hasznos szamjegyek
*****************************************************************************/
/*
  void asyncIntToStr2(uint szam)
    {
     byte bontva[5];
     uint tizh = 10000;//4 digitet ir ki
     byte i,j;
     for (i=0; i<5; i++)
       bontva[i] = '0';
     i = 0;
     while (szam > 0)
       {
        while (szam >= tizh)
          {
           szam-=tizh;
           (bontva[i])++;
          }
        if (tizh >= 10) 
          tizh /= 10;
        i++;
       }
     i = 0;
//     while (bontva[i] == '0')
//       i++;
     i=4; // 5digitet ir ki
     for (j=i; j<5; j++)
       asyncPutByte(bontva[j]);
    }
*/
/*****************************************************************************
* asyncIntToStr5 - szam kikuldese sztringkent, csak hasznos szamjegyek
*****************************************************************************/
  void asyncIntToStr5(uint szam)
    {
     byte bontva[5];
     uint tizh = 10000;//4 digitet ir ki
     byte i,j;
     for (i=0; i<5; i++)
       bontva[i] = '0';
     i = 0;
     while (szam > 0)
       {
        while (szam >= tizh)
          {
           szam-=tizh;
           (bontva[i])++;
          }
        if (tizh >= 10) 
          tizh /= 10;
        i++;
       }
     i = 0;
//     while (bontva[i] == '0')
//       i++;
     i=0; // 5digitet ir ki
     for (j=i; j<5; j++)
       asyncPutByte(bontva[j]);
    }

/*****************************************************************************
* asyncInit - szoft. aszinkron inicializalasa
*****************************************************************************/
void asyncInit(void)
  {
   byte dummy;

   bInStart = bInEnd = bInCnt =0;
   bOutStart = bOutEnd = bOutCnt =0;

   TX9 = 0;
   RX9 = 0;

   BRGH  = 1; BRG16 = 1;
   SPBRG = 42;//116.3kbaud bps=20 000 000/(4*(n+1)=116,279
   SPBRGH = 0;

   SYNC = 0; //async kommunikacio
   SPEN = 1; // enable soros port
   CREN = 1;

   dummy=RCSTA;
   dummy=RCREG;
   dummy=RCREG;

   PEIE = 1; //periferialis megszak.
   RCIE = 1; //receive int.
   TXIE = 0; //transmit int.
   TXEN = 1; // adó engedélyezése

  }

