
#include    <pic18.h>
#include "spi.h"





void WriteSPI(unsigned char i)
{
	unsigned char zzz;

    SSPBUF = i;              
	
	zzz=0;
	while(zzz<10) zzz++;

	while(!BF); // BF 플래그가 셋 되면 외부에서 데이타가 수신된 것이다. 
				// 따라서 BF 가 셋되면 다음을 수행한다. 

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



unsigned char  Write_AddicoreRFID(unsigned char addr,unsigned char thisdata)
{
	CS=0;
	WriteSPI((addr<<1)&0x7E);
	WriteSPI(thisdata);
	CS=1;
	return(SSPBUF);
}

unsigned char  Read_AddicoreRFID(unsigned char addr)
{
	CS=0;
	WriteSPI(((addr<<1)&0x7E) | 0x80);
	WriteSPI(0x00);
	CS=1;
	return(SSPBUF);
}



/**
 * Writes a byte to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
unsigned char PCD_WriteRegister(unsigned char reg,	
										byte value
										)		
{
	CS=0;	
	WriteSPI((reg<<1)&0x7E);
	WriteSPI(value);
	CS=1;
	return(SSPBUF);
} // End PCD_WriteRegister()


/**
 * Writes a number of bytes to the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
unsigned char PCD_WriteRegister_A(	unsigned char reg,	///< The register to write to. One of the PCD_Register enums.
									byte count,			///< The number of bytes to write to the register
									byte *values		///< The values to write. Byte array.
								) {
	byte index;
								
	CS=0;
	WriteSPI((reg<<1)&0x7E);						// MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
	for (index = 0; index < count; index++) {
		WriteSPI(values[index]);
	}
	CS=1;
	return(SSPBUF);
} // End PCD_WriteRegister()


unsigned char PCD_ReadRegister( unsigned char reg		)		
{
	CS=0;
	WriteSPI(((reg<<1)&0x7E) | 0x80);
	WriteSPI(0x00);
	CS=1;
	return(SSPBUF);
} 


/**
 * Reads a number of bytes from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
unsigned char PCD_ReadRegister_A(	unsigned char reg,	///< The register to read from. One of the PCD_Register enums.
								byte count,			///< The number of bytes to read
								byte *values,		///< Byte array to store the values in.
								byte rxAlign		///< Only bit positions rxAlign..7 in values[0] are updated.
								) {	
	byte index = 0;
	byte mask;
	byte value;

								
	if (count == 0) {
		return 0;
	}
	//Serial.print(F("Reading ")); 	Serial.print(count); Serial.println(F(" bytes from register."));

	CS=0;
	count--;								// One read is performed outside of the loop
	WriteSPI(((reg<<1)&0x7E) | 0x80);					// Tell MFRC522 which address we want to read
	if (rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
		// Create bit mask for bit positions rxAlign..7
		mask = (0xFF << rxAlign) & 0xFF;
		// Read value and tell that we want to read the same address again.
		WriteSPI(((reg<<1)&0x7E) | 0x80);
		// Apply mask to both current value of values[0] and the new data in value.
		values[0] = (values[0] & ~mask) | (SSPBUF & mask);
		index++;
	}
	while (index < count) {
		WriteSPI(((reg<<1)&0x7E) | 0x80);	// Read value and tell that we want to read the same address again.
		values[index] = SSPBUF;
		index++;
	}
	WriteSPI(0);			// Read the final byte. Send 0 to stop reading.
	values[index] = SSPBUF;
	CS=1;
	return(SSPBUF);
} // End PCD_ReadRegister()



void InitSPI(void)
{
	TRISC3=0;   //sck
	TRISC4=1;   //sdi
	TRISC5=0;   //sdo
	TRISA5=0;   //ss
	SSPSTAT = 0xC0;         //SPI Bus mode 0,0
	SSPCON1 = 0x21;         //Enable SSP, Fosc/16

	CKE=0;  // SPI Clock Select bit
	CKP=1; 	// Clock Polarity Select bit 
			// 1 = Idle state for clock is a high level
			// 0 = Idle state for clock is a low level

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




