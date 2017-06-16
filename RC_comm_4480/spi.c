
#include    <pic18.h>
#include "spi.h"





void WriteSPI(unsigned char i)
{
	unsigned char zzz;

    SSPBUF = i;              
	
	zzz=0;
	while(zzz<10) zzz++;

	while(!BF);

}


unsigned char  WriteSPI_ADDR(unsigned char addr,unsigned char thisdata)
{
	CS=0;
	WriteSPI(addr | WRITE_CMD);
	WriteSPI(thisdata);
	CS=1;
	return(SSPBUF);
}


unsigned char  ReadSPI_ADDR(unsigned char addr)
{
	CS=0;
	WriteSPI(addr | READ_CMD);
	WriteSPI(0x00);
	CS=1;
	return(SSPBUF);
}



void InitSPI(void)
{
	TRISC3=0;   //sck
	TRISC4=1;   //sdi
	TRISC5=0;   //sdo
	TRISA5=0;   //ss
	SSPSTAT = 0xC0;         //SPI Bus mode 0,0
	SSPCON1 = 0x21;         //Enable SSP, Fosc/16

	CKE=0;  //0
	CKP=1;

	LATC4=1;
	RC4=1;
}






unsigned char  SPI_Play(unsigned char sel_voice)
{
	return(WriteSPI_ADDR(ADD00_DEC_CHAP,sel_voice));
}


unsigned char  SPI_Stop_Play(void)
{
	return(WriteSPI_ADDR(ADD00_AUDIO_CON,0x02));
}


unsigned char  SPI_Play_Status_Chk(void)
{
	unsigned char sta;

	sta=ReadSPI_ADDR(ADD00_SBC_CON);
	if(sta & 0x80){
		WriteSPI_ADDR(ADD00_LED_CON,0xf0);
		return(1);
	}
	else{
		WriteSPI_ADDR(ADD00_LED_CON,0x0);
		return(0);
	}
}




