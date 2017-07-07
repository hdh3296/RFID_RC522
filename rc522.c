
#ifndef __RC522_H_
#define	__RC522_H_

#include "RC_comm_4480\spi.h"
#include "rc522.h"


/*
 * Function Name: AntennaOn
 * Description: Open antennas, each time you start or shut down the natural barrier between the transmitter should be at least 1ms interval
 * Input: None
 * Return value: None
 */
void AntennaOn(void)
{
	byte temp;

	temp = Read_AddicoreRFID(TxControlReg);
	if (!(temp & 0x03))
	{
		SetBitMask(TxControlReg, 0x03);
	}
}


/*
 * Function Name: AddicoreRFID_Reset
 * Description: Perform soft reset of AddicoreRFID Module
 * Input: None
 * Return value: None
 */
void AddicoreRFID_Reset(void)
{
    Write_AddicoreRFID(CommandReg, PCD_SOFTRESET);
}

/*
 * Function Name: AddicoreRFID_Init
 * Description: Initialize the AddicoreRFID module
 * Input: None
 * Return value: None
*/
void AddicoreRFID_Init(void)
{

		AddicoreRFID_Reset();           // Soft reset the AddicoreRFID

	 	
	// Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
    Write_AddicoreRFID(TModeReg, 0x8D);		// Tauto=1; f(Timer) = 6.78MHz/TPreScaler
    Write_AddicoreRFID(TPrescalerReg, 0x3E);	//TModeReg[3..0] + TPrescalerReg
    Write_AddicoreRFID(TReloadRegL, 30);           
    Write_AddicoreRFID(TReloadRegH, 0);
	
	Write_AddicoreRFID(TxAutoReg, 0x40);		// 100%ASK
	Write_AddicoreRFID(ModeReg, 0x3D);		// CRC Initial value 0x6363	???

	// ClearBitMask(Status2Reg, 0x08);		// MFCrypto1On=0
	// Write_AddicoreRFID(RxSelReg, 0x86);		// RxWait = RxSelReg[5..0]
	// Write_AddicoreRFID(RFCfgReg, 0x7F);   		// RxGain = 48dB

	AntennaOn();		//Open the antenna
}



void SetBitMask(byte reg, byte mask)  
{
    byte tmp;
    tmp = Read_AddicoreRFID(reg);
    Write_AddicoreRFID(reg, tmp | mask);  // set bit mask
}

void ClearBitMask(byte reg, byte mask)  
{
    byte tmp;
    tmp = Read_AddicoreRFID(reg);
    Write_AddicoreRFID(reg, tmp & (~mask));  // clear bit mask
} 



/*
 * Function Name: AddicoreRFID_Request
 * Description: Find cards, read the card type number
 * Input parameters: reqMode - find cards way
 *			 TagType - Return Card Type
 *			 	0x4400 = Mifare_UltraLight
 *				0x0400 = Mifare_One(S50)
 *				0x0200 = Mifare_One(S70)
 *				0x0800 = Mifare_Pro(X)
 *				0x4403 = Mifare_DESFire
 * Return value: the successful return MI_OK
 */
byte AddicoreRFID_Request(byte reqMode, byte *TagType)
{
	byte status;  

	Write_AddicoreRFID(BitFramingReg, 0x07);	
	
	TagType[0] = reqMode;
	status = AddicoreRFID_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &_RxBits);
	
	if ((status != MI_OK) || (_RxBits != 0x10))
	{    
		status = MI_ERR;
	}
  
	return status;
}



/*
 * Function Name: AddicoreRFID_ToCard
 * Description: RC522 and ISO14443 card communication
 * Input Parameters: command - MF522 command word,
 *			 sendData--RC522 sent to the card by the data
 *			 sendLen--Length of data sent	 
 *			 backData--Data returned from the card
 *			 backLen--Returned data bit length
 * Return value: the successful return MI_OK
 */

byte n;
byte AddicoreRFID_ToCard(byte command, byte *sendData, byte sendLen, 
												byte *backData, uint *backLen)
{
    byte status = MI_ERR;
    byte irqEn = 0x00;
    byte waitIRq = 0x00;
    byte lastBits;
    
    uint i;

    switch (command)
    {
        case PCD_MFAUTHENT:		//Certification cards close
		{
			irqEn = 0x12;
			waitIRq = 0x10;
			break;
		}
		case PCD_TRANSCEIVE:	//Transmit FIFO data
		{
			irqEn = 0x77;
			waitIRq = 0x30;
			break;
		}
		default:
			break;
    }
   
    Write_AddicoreRFID(ComIrqReg, irqEn|0x80);	//Interrupt request
    ClearBitMask(ComIrqReg, 0x80);			//Clear all interrupt request bit
    SetBitMask(FIFOLevelReg, 0x80);			//FlushBuffer=1, FIFO Initialization
    
	Write_AddicoreRFID(CommandReg, PCD_IDLE);	//NO action; Cancel the current command???

	// Writing data to the FIFO
    for (i=0; i<sendLen; i++)
    {   
		Write_AddicoreRFID(FIFODataReg, sendData[i]);    
	}

	// Execute the command
	Write_AddicoreRFID(CommandReg, command);
    if (command == PCD_TRANSCEIVE)
    {    
		SetBitMask(BitFramingReg, 0x80);		//StartSend=1,transmission of data starts  
	}   
    
	// Waiting to receive data to complete
	i = 2000;	//i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
    do 
    {
		//ComIrqReg[7..0]
		//Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
        n = Read_AddicoreRFID(ComIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitIRq));

    ClearBitMask(BitFramingReg, 0x80);			//StartSend=0
	
    if (i != 0)
    {    
        if(!(Read_AddicoreRFID(ErrorReg) & 0x1B))	//BufferOvfl Collerr CRCErr ProtecolErr
        {
            status = MI_OK;
            if (n & irqEn & 0x01)
            {   
				status = MI_NO_TAG_ERR;			//??   
			}

            if (command == PCD_TRANSCEIVE)
            {
               	n = Read_AddicoreRFID(FIFOLevelReg);
              	lastBits = Read_AddicoreRFID(ControlReg) & 0x07;
                if (lastBits)
                {   
					*backLen = (n-1)*8 + lastBits;   
				}
                else
                {   
					*backLen = n*8;   
				}

                if (n == 0)
                {   
					n = 1;    
				}
                if (n > MAX_LEN)
                {   
					n = MAX_LEN;   
				}
				
				//Reading the received data in FIFO
                for (i=0; i<n; i++)
                {   
					backData[i] = Read_AddicoreRFID(FIFODataReg);    
				}
            }
        }
        else
        {   
			status = MI_ERR;  
		}
        
    }
	
    //SetBitMask(ControlReg,0x80);           //timer stops
    //Write_AddicoreRFID(CommandReg, PCD_IDLE); 

    return status;
}

/*
 * Function Name: AddicoreRFID_Anticoll
 * Description: Anti-collision detection, reading selected card serial number card
 * Input parameters: serNum - returns 4 bytes card serial number, the first 5 bytes for the checksum byte
 * Return value: the successful return MI_OK
 */
byte AddicoreRFID_Anticoll(byte *serNum)
{
    byte status;
    byte i;
	byte serNumCheck=0;
    //uint unLen;

    //ClearBitMask(Status2Reg, 0x08);		//TempSensclear
    //ClearBitMask(CollReg,0x80);			//ValuesAfterColl
	Write_AddicoreRFID(BitFramingReg, 0x00);		//TxLastBists = BitFramingReg[2..0]
 
    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = AddicoreRFID_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &_RxBits);

    if (status == MI_OK)
	{
		//Check card serial number
		for (i=0; i<4; i++)
		{   
		 	serNumCheck ^= serNum[i];
		}
		if (serNumCheck != serNum[i])
		{   
			status = MI_ERR;    
		}
    }

    //SetBitMask(CollReg, 0x80);		//ValuesAfterColl=1

	//memcpy(serNum, str, 5);
    return status;
} 

/*
 * Function Name: CalulateCRC
 * Description: CRC calculation with MF522
 * Input parameters: pIndata - To read the CRC data, len - the data length, pOutData - CRC calculation results
 * Return value: None
 */
void CalulateCRC(byte *pIndata, byte len, byte *pOutData)
{
    byte i, n;

    ClearBitMask(DivIrqReg, 0x04);			//CRCIrq = 0
    SetBitMask(FIFOLevelReg, 0x80);			//Clear the FIFO pointer
    //Write_AddicoreRFID(CommandReg, PCD_IDLE);

	//Writing data to the FIFO	
    for (i=0; i<len; i++)
    {   
		Write_AddicoreRFID(FIFODataReg, *(pIndata+i));   
	}
    Write_AddicoreRFID(CommandReg, PCD_CALCCRC);

	//Wait CRC calculation is complete
    i = 0xFF;
    do 
    {
        n = Read_AddicoreRFID(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));			//CRCIrq = 1

	//Read CRC calculation result
    pOutData[0] = Read_AddicoreRFID(CRCResultRegL);
    pOutData[1] = Read_AddicoreRFID(CRCResultRegM);
}


/*
 * Function Name: AddicoreRFID_Halt
 * Description: Command card into hibernation
 * Input: None
 * Return value: None
 */
void AddicoreRFID_Halt(void)
{
	byte status;
    uint unLen;
    byte buff[4]; 

    buff[0] = PICC_HALT;
    buff[1] = 0;
    CalulateCRC(buff, 2, &buff[2]);
 
    status = AddicoreRFID_ToCard(PCD_TRANSCEIVE, buff, 4, buff,&unLen);
}


/*
 * Function Name: AddicoreRFID_Read
 * Description: Read block data
 * Input parameters: blockAddr - block address; recvData - read block data
 * Return value: the successful return MI_OK
 */
byte AddicoreRFID_Read(byte blockAddr, byte *recvData)
{
    byte status;
    uint unLen;

    recvData[0] = PICC_READ;
    recvData[1] = blockAddr;
    CalulateCRC(recvData,2, &recvData[2]);
    status = AddicoreRFID_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

    if ((status != MI_OK) || (unLen != 0x90))
    {
        status = MI_ERR;
    }

    
    return status;
}



/*
 * Function Name: AddicoreRFID_Write
 * Description: Write block data
 * Input parameters: blockAddr - block address; writeData - to 16-byte data block write
 * Return value: the successful return MI_OK
 */
byte AddicoreRFID_Write(byte blockAddr, byte *_writeData)
{
    byte status;
    uint recvBits;
    byte i;
	byte buff[18]; 
    
    buff[0] = PICC_WRITE;
    buff[1] = blockAddr;
    CalulateCRC(buff, 2, &buff[2]);
    status = AddicoreRFID_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
    {   
		status = MI_ERR;   
	}
        
    if (status == MI_OK)
    {
        for (i=0; i<16; i++)		//Data to the FIFO write 16Byte
        {    
        	buff[i] = *(_writeData+i);   
        }
        CalulateCRC(buff, 16, &buff[16]);
        status = AddicoreRFID_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);
        
		if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
        {   
			status = MI_ERR;   
		}
    }
    
    return status;
}




/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////


/**
 * Initializes the MFRC522 chip.
 */
void PCD_Init() 
{	
	// Reset baud rates
	PCD_WriteRegister(TxModeReg, 0x00);
	PCD_WriteRegister(RxModeReg, 0x00);
	// Reset ModWidthReg
	PCD_WriteRegister(ModWidthReg, 0x26);

	// When communicating with a PICC we need a timeout if something goes wrong.
	// f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
	// TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
	PCD_WriteRegister(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
	PCD_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_WriteRegister(TReloadRegL, 0xE8);
	
	PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
}


/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void PCD_AntennaOn(void) 
{
	byte value;
	
	value =	PCD_ReadRegister(TxControlReg);
	if ((value & 0x03) != 0x03) {
		PCD_WriteRegister(TxControlReg, value | 0x03);
	}
} 



/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 * 
 * @return bool
 */

volatile byte bbb = 0x44;
byte PICC_IsNewCardPresent(void) 
{
	byte bufferATQA[2];
	byte bufferSize = 2;
	byte result;

	// Reset baud rates
	PCD_WriteRegister(TxModeReg, 0x00);
	PCD_WriteRegister(RxModeReg, 0x00);
	// Reset ModWidthReg
	PCD_WriteRegister(ModWidthReg, 0x26);

	result = PICC_RequestA(bufferATQA, bufferSize);
	bbb = result;
	
	return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte PICC_RequestA(	byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
											byte bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
										) {
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()


/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
byte PICC_REQA_or_WUPA(	byte command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
												byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
												byte bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
											) {
	byte validBits;
	byte status;
	
	if (bufferSize < 2) {	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
	}
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits, 0, false);
	if (status != STATUS_OK) {
		return status;
	}
	if (bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // End PICC_REQA_or_WUPA()


/**
 * Clears the bits given in mask from register reg.
 */
void PCD_ClearRegisterBitMask(	unsigned char reg,	///< The register to update. One of the PCD_Register enums.
										byte mask			///< The bits to clear.
									  ) {
	byte tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp & (~mask));		// clear bit mask
} // End PCD_ClearRegisterBitMask()


/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte PCD_TransceiveData(	byte *sendData,		///< Pointer to the data to transfer to the FIFO.
													byte sendLen,		///< Number of bytes to transfer to the FIFO.
													byte *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
													byte backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
													byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
													byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
													byte checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated. Default false
								 ) {
	byte waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, 
													rxAlign, checkCRC);
} // End PCD_TransceiveData()


/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte PCD_CommunicateWithPICC(	byte command,		///< The command to execute. One of the PCD_Command enums.
														byte waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
														byte *sendData,		///< Pointer to the data to transfer to the FIFO.
														byte sendLen,		///< Number of bytes to transfer to the FIFO.
														byte *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
														byte backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
														byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
														byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
														byte checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
									 ) {
	unsigned int i;
	unsigned char status;
									 
	// Prepare values for BitFramingReg
	byte txLastBits = validBits ? *validBits : 0;
	byte bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	
	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
	PCD_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	PCD_WriteRegister(FIFOLevelReg, 0x80);				// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister_A(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO
	PCD_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
	PCD_WriteRegister(CommandReg, command);				// Execute the command
	if (command == PCD_Transceive) {
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}
	
	// Wait for the command to complete.
	// In PCD_Init() we set the TAuto flag in TModeReg. This means the timer automatically starts when the PCD stops transmitting.
	// Each iteration of the do-while-loop takes 17.86μs.
	// TODO check/modify for other architectures than Arduino Uno 16bit
	
	for (i = 2000; i > 0; i--) {
		byte n = PCD_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
			return STATUS_TIMEOUT;
		}
	}
	// 35.7ms and nothing happend. Communication with the MFRC522 might be down.
	if (i == 0) {
		return STATUS_TIMEOUT;
	}
	
	// Stop now if any errors except collisions were detected.
	byte errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}
  
	byte _validBits = 0;
	
	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		byte n = PCD_ReadRegister(FIFOLevelReg);	// Number of bytes in the FIFO
		if (n > backLen) {
			return STATUS_NO_ROOM;
		}
		backLen = n;											// Number of bytes returned
		PCD_ReadRegister_A(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = PCD_ReadRegister(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}
	
	// Tell about collisions
	if (errorRegValue & 0x08) {		// CollErr
		return STATUS_COLLISION;
	}
	
	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) {
		// In this case a MIFARE Classic NAK is not OK.
		if (backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		byte controlBuffer[2];
		status = PCD_CalculateCRC(&backData[0], backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) {
			return status;
		}
		if ((backData[backLen - 2] != controlBuffer[0]) || (backData[backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}
	
	return STATUS_OK;
} // End PCD_CommunicateWithPICC()



/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte PCD_CalculateCRC(	byte *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
												byte length,	///< In: The number of bytes to transfer.
												byte *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
					 ) {
	unsigned int i;
					 
	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop any active command.
	PCD_WriteRegister(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	PCD_WriteRegister(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister_A(FIFODataReg, length, data);	// Write data to the FIFO
	PCD_WriteRegister(CommandReg, PCD_CalcCRC);		// Start the calculation
	
	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73μs.
	// TODO check/modify for other architectures than Arduino Uno 16bit

	// Wait for the CRC calculation to complete. Each iteration of the while-loop takes 17.73us.
	for (i = 5000; i > 0; i--) {
		// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
		byte n = PCD_ReadRegister(DivIrqReg);
		if (n & 0x04) {									// CRCIRq bit set - calculation done
			PCD_WriteRegister(CommandReg, PCD_Idle);	// Stop calculating CRC for new content in the FIFO.
			// Transfer the result from the registers to the result buffer
			result[0] = PCD_ReadRegister(CRCResultRegL);
			result[1] = PCD_ReadRegister(CRCResultRegH);
			return STATUS_OK;
		}
	}
	// 89ms passed and nothing happend. Communication with the MFRC522 might be down.
	return STATUS_TIMEOUT;
} // End PCD_CalculateCRC()


/**
 * Sets the bits given in mask in register reg.
 */
void PCD_SetRegisterBitMask(	unsigned char reg,	///< The register to update. One of the PCD_Register enums.
										byte mask			///< The bits to set.
									) { 
	byte tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp | mask);			// set bit mask
} // End PCD_SetRegisterBitMask()



#endif	
