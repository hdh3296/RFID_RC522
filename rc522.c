
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

	result = PICC_RequestA(bufferATQA, &bufferSize);
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
											byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
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
												byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
											) {
	byte validBits;
	byte status;
	
	if (*bufferSize < 2) {	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
	}
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits, 0, false);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
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
													byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
													byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
													byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
													byte checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated. Default false
								 ) {	// byte *validBits = nullptr, byte rxAlign = 0, bool checkCRC = false);
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
														byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
														byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
														byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
														byte checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
									 ) {
	unsigned int i;
	unsigned char status;
	byte n;
	byte controlBuffer[2];
	byte txLastBits;
	byte bitFraming;
	byte errorRegValue;
	byte _validBits;
									 
	// Prepare values for BitFramingReg
	if (validBits)
		txLastBits = *validBits;
	else
		txLastBits = 0;
	bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	
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
		n = PCD_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
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
	errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}
  
	_validBits = 0;
	
	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) {
		n = PCD_ReadRegister(FIFOLevelReg);	// Number of bytes in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;											// Number of bytes returned
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
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		
		status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
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

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 * 
 * @return bool
 */
byte PICC_ReadCardSerial(void) {
	byte result;

	result = PICC_Select(&uid, 0);
	return (result == STATUS_OK);
} // End 


/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 * 
 * A PICC UID consists of 4, 7 or 10 bytes.
 * Only 4 bytes can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID bytes		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte PICC_Select(	Uid *uid,			///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
						byte validBits		///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
						) {
	static bit uidComplete;
	static bit selectDone;
	static bit useCascadeTag;
	byte cascadeLevel = 1;
	byte result;
	byte count;
	byte index;
	byte uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	byte buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	byte bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	byte rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	byte txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
	byte *responseBuffer;
	byte responseLength;
	
	// Description of buffer structure:
	//		Byte 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
	//		Byte 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits. 
	//		Byte 2: UID-data or CT		See explanation below. CT means Cascade Tag.
	//		Byte 3: UID-data
	//		Byte 4: UID-data
	//		Byte 5: UID-data
	//		Byte 6: BCC					Block Check Character - XOR of bytes 2-5
	//		Byte 7: CRC_A
	//		Byte 8: CRC_A
	// The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
	//
	// Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
	//		UID size	Cascade level	Byte2	Byte3	Byte4	Byte5
	//		========	=============	=====	=====	=====	=====
	//		 4 bytes		1			uid0	uid1	uid2	uid3
	//		 7 bytes		1			CT		uid0	uid1	uid2
	//						2			uid3	uid4	uid5	uid6
	//		10 bytes		1			CT		uid0	uid1	uid2
	//						2			CT		uid3	uid4	uid5
	//						3			uid6	uid7	uid8	uid9
	
	// Sanity checks
	if (validBits > 80) {
		return STATUS_INVALID;
	}
	
	// Prepare MFRC522
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
	// Repeat Cascade Level loop until we have a complete UID.
	uidComplete = false;
	while (!uidComplete) {
		// Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;
			
			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;
			
			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;
			
			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}
		
		// How many UID bits are known in this Cascade Level?
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		// Copy the known bits from uid->uidByte[] to buffer[]
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		byte bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			byte maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		// Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}
		
		// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
		selectDone = false;
		while (!selectDone) {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}
			
			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			
			// Transmit the buffer and receive the response.
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, false);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				byte valueOfCollReg = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				byte collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits = collisionPos;
				count			= (currentLevelKnownBits - 1) % 8; // The bit to modify
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << count);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision 
					// We continue below outside the while.
				}
				else { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
					currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
		} // End of while (!selectDone)
		
		// We do not check the CBB - it was constructed by us above.
		
		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}
		
		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)
	
	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
}



/**
 * Dumps debug info about the selected PICC to Serial.
 * On success the PICC is halted after dumping the data.
 * For MIFARE Classic the factory default key of 0xFFFFFFFFFFFF is tried.  
 *
 * @DEPRECATED Kept for bakward compatibility
 */
void PICC_DumpToSerial(Uid *uid	///< Pointer to Uid struct returned from a successful PICC_Select().
								) {
	MIFARE_Key key;
	byte piccType;
	byte i;
	
	// Dump UID, SAK and Type
	PICC_DumpDetailsToSerial(uid);
	
	// Dump contents
	piccType = PICC_GetType(uid->sak);
	switch (piccType) {
		case PICC_TYPE_MIFARE_MINI:
		case PICC_TYPE_MIFARE_1K:
		case PICC_TYPE_MIFARE_4K:	// 정상적이면 여기 온다. 
			// All keys are set to FFFFFFFFFFFFh at chip delivery from the factory.
			for (i = 0; i < 6; i++) {
				key.keyByte[i] = 0xFF;
			}
			PICC_DumpMifareClassicToSerial(uid, piccType, &key);
			break;
			
		case PICC_TYPE_MIFARE_UL:
			PICC_DumpMifareUltralightToSerial();
			break;
			
		case PICC_TYPE_ISO_14443_4:
		case PICC_TYPE_MIFARE_DESFIRE:
		case PICC_TYPE_ISO_18092:
		case PICC_TYPE_MIFARE_PLUS:
		case PICC_TYPE_TNP3XXX:
			Serial_println(
			//"Dumping memory contents not implemented for that PICC type."
			);
			break;
			
		case PICC_TYPE_UNKNOWN:
		case PICC_TYPE_NOT_COMPLETE:
		default:
			break; // No memory dump here
	}
	
	Serial_println();
	PICC_HaltA(); // Already done if it was a MIFARE Classic PICC.
}



/**
 * Dumps card info (UID,SAK,Type) about the selected PICC to Serial.
 *
 * @DEPRECATED kept for backward compatibility
 */
void PICC_DumpDetailsToSerial(Uid *uid	///< Pointer to Uid struct returned from a successful PICC_Select().
									) {
	byte i;									
	// UID
	Serial_print(//"Card UID:"
					);
	for (i = 0; i < uid->size; i++) {
		if(uid->uidByte[i] < 0x10)
			Serial_print(//" 0"
							);
		else
			Serial_print(//" "
						);
		Serial_print(//uid->uidByte[i]
							//, HEX
							);
	} 
	Serial_println();
	
	// SAK
	Serial_print(//"Card SAK: "
					);
	if(uid->sak < 0x10)
		Serial_print(//"0"
						);
	Serial_println(//uid->sak
					//, HEX
					);
	
	// (suggested) PICC type
	PICC_Type piccType = PICC_GetType(uid->sak);
	Serial_print(//"PICC type: "
					);
//	Serial_println(PICC_GetTypeName(piccType));
} // End PICC_DumpDetailsToSerial()



/**
 * Translates the SAK (Select Acknowledge) to a PICC type.
 * 
 * @return PICC_Type
 */
byte PICC_GetType(byte sak		///< The SAK byte returned from PICC_Select().
							) {
	// http://www.nxp.com/documents/application_note/AN10833.pdf 
	// 3.2 Coding of Select Acknowledge (SAK)
	// ignore 8-bit (iso14443 starts with LSBit = bit 1)
	// fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
	sak &= 0x7F;
	switch (sak) {
		case 0x04:	return PICC_TYPE_NOT_COMPLETE;	// UID not complete
		case 0x09:	return PICC_TYPE_MIFARE_MINI;
		case 0x08:	return PICC_TYPE_MIFARE_1K;
		case 0x18:	return PICC_TYPE_MIFARE_4K;
		case 0x00:	return PICC_TYPE_MIFARE_UL;
		case 0x10:
		case 0x11:	return PICC_TYPE_MIFARE_PLUS;
		case 0x01:	return PICC_TYPE_TNP3XXX;
		case 0x20:	return PICC_TYPE_ISO_14443_4;
		case 0x40:	return PICC_TYPE_ISO_18092;
		default:	return PICC_TYPE_UNKNOWN;
	}
} // End PICC_GetType()


/**
 * Dumps memory contents of a MIFARE Classic PICC.
 * On success the PICC is halted after dumping the data.
 */
void PICC_DumpMifareClassicToSerial(	Uid *uid,			///< Pointer to Uid struct returned from a successful PICC_Select().
												PICC_Type piccType,	///< One of the PICC_Type enums.
												MIFARE_Key *key		///< Key A used for all sectors.
											) {
	byte no_of_sectors = 0;
	int i;
	
	switch (piccType) {
		case PICC_TYPE_MIFARE_MINI:
			// Has 5 sectors * 4 blocks/sector * 16 bytes/block = 320 bytes.
			no_of_sectors = 5;
			break;
			
		case PICC_TYPE_MIFARE_1K: // 정상적이면 여기 온다. 
			// Has 16 sectors * 4 blocks/sector * 16 bytes/block = 1024 bytes.
			no_of_sectors = 16;
			break;
			
		case PICC_TYPE_MIFARE_4K:
			// Has (32 sectors * 4 blocks/sector + 8 sectors * 16 blocks/sector) * 16 bytes/block = 4096 bytes.
			no_of_sectors = 40;
			break;
			
		default: // Should not happen. Ignore.
			break;
	}
	
	// Dump sectors, highest address first.
	if (no_of_sectors) {
		Serial_println(//"Sector Block   0  1  2  3   4  5  6  7   8  9 10 11  12 13 14 15  AccessBits"
						);
		for (i = no_of_sectors - 1; i >= 0; i--) {
			PICC_DumpMifareClassicSectorToSerial(uid, key, i);
		}
	}
	PICC_HaltA(); // Halt the PICC before stopping the encrypted session.
	PCD_StopCrypto1();
} // End PICC_DumpMifareClassicToSerial()


/**
 * Dumps memory contents of a MIFARE Ultralight PICC.
 */
void PICC_DumpMifareUltralightToSerial() {
	byte status;
	byte byteCount;
	byte buffer[18];
	byte i;
	
	Serial_println(//"Page  0  1  2  3"
					);
	// Try the mpages of the original Ultralight. Ultralight C has more pages.
	for (byte page = 0; page < 16; page +=4) { // Read returns data for 4 pages at a time.
		// Read pages
        byteCount = 18; // byteCount = sizeof(buffer);
//		status = MIFARE_Read(page, buffer, &byteCount);
		if (status != STATUS_OK) {
			Serial_print(//"MIFARE_Read() failed: "
							);
//			Serial_println(GetStatusCodeName(status));
			break;
		}
		// Dump data
		for (byte offset = 0; offset < 4; offset++) {
			i = page + offset;
			if(i < 10)
				Serial_print(//"  "
								); // Pad with spaces
			else
				Serial_print(//" "
								); // Pad with spaces
			Serial_print(//i
							);
			Serial_print(//"  "
							);
			for (byte index = 0; index < 4; index++) {
				i = 4 * offset + index;
				if(buffer[i] < 0x10)
					Serial_print(//" 0"
									);
				else
					Serial_print(//" "
									);
				Serial_print(//buffer[i]
								//, HEX
								);
			}
			Serial_println();
		}
	}
} // End PICC_DumpMifareUltralightToSerial()


/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */ 
byte PICC_HaltA() {
	byte result;
	byte buffer[4];
	
	// Build command buffer
	buffer[0] = PICC_CMD_HLTA;
	buffer[1] = 0;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}
	
	// Send the command.
	// The standard says:
	//		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
	//		HLTA command, this response shall be interpreted as 'not acknowledge'.
	// We interpret that this way: Only STATUS_TIMEOUT is a success.
	result = PCD_TransceiveData(buffer, sizeof(buffer), nullptr, 0, 0, 0, 0);
	if (result == STATUS_TIMEOUT) {
		return STATUS_OK;
	}
	if (result == STATUS_OK) { // That is ironically NOT ok in this case ;-)
		return STATUS_ERROR;
	}
	return result;
} // End PICC_HaltA()

void Serial_print()
{
	byte a;
	a = 1;
}

void Serial_printl()
{
	byte a;
	a = 1;
}

void Serial_println()
{
	byte a;
	a = 1;

}


/**
 * Dumps memory contents of a sector of a MIFARE Classic PICC.
 * Uses PCD_Authenticate(), MIFARE_Read() and PCD_StopCrypto1.
 * Always uses PICC_CMD_MF_AUTH_KEY_A because only Key A can always read the sector trailer access bits.
 */
void PICC_DumpMifareClassicSectorToSerial(Uid *uid,			///< Pointer to Uid struct returned from a successful PICC_Select().
													MIFARE_Key *key,	///< Key A for the sector.
													byte sector			///< The sector to dump, 0..39.
													) {
	byte status;
	byte firstBlock;		// Address of lowest address to dump actually last block dumped)
	byte no_of_blocks;		// Number of blocks in sector
	bool isSectorTrailer;	// Set to true while handling the "last" (ie highest address) in the sector.
	int8_t blockOffset;
	int32_t value;

	// Dump blocks, highest address first.
	byte byteCount;
	byte buffer[18];
	byte blockAddr;
	

	
	// The access bits are stored in a peculiar fashion.
	// There are four groups:
	//		g[3]	Access bits for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
	//		g[2]	Access bits for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
	//		g[1]	Access bits for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
	//		g[0]	Access bits for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
	// Each group has access bits [C1 C2 C3]. In this code C1 is MSB and C3 is LSB.
	// The four CX bits are stored together in a nible cx and an inverted nible cx_.
	byte c1, c2, c3;		// Nibbles
	byte c1_, c2_, c3_;		// Inverted nibbles
	bool invertedError;		// True if one of the inverted nibbles did not match
	byte g[4];				// Access bits for each of the four groups.
	byte group;				// 0-3 - active group for access bits
	bool firstInGroup;		// True for the first block dumped in the group

	int8_t blockOffset;
	byte index;


	isSectorTrailer = true;
	invertedError = false;	// Avoid "unused variable" warning.
	
	// Determine position and size of sector.
	if (sector < 32) { // Sectors 0..31 has 4 blocks each
		no_of_blocks = 4;
		firstBlock = sector * no_of_blocks;
	}
	else if (sector < 40) { // Sectors 32-39 has 16 blocks each
		no_of_blocks = 16;
		firstBlock = 128 + (sector - 32) * no_of_blocks;
	}
	else { // Illegal input, no MIFARE Classic PICC has more than 40 sectors.
		return;
	}
		

	for (blockOffset = no_of_blocks - 1; blockOffset >= 0; blockOffset--) {
		blockAddr = firstBlock + blockOffset;
		// Sector number - only on first line
		if (isSectorTrailer) {
			if(sector < 10)
				Serial_print(//F("   ")
								); // Pad with spaces
			else
				Serial_print(//F("  ")
								); // Pad with spaces
			Serial_print(//sector
							);
			Serial_print(//F("   ")
							);
		}
		else {
			Serial_print(//F("       ")
							);
		}
		// Block number
		if(blockAddr < 10)
			Serial_print(//F("   ")
							); // Pad with spaces
		else {
			if(blockAddr < 100)
				Serial_print(//F("  ")
							); // Pad with spaces
			else
				Serial_print(//F(" ")
							); // Pad with spaces
		}
		Serial_print(//blockAddr
						);
		Serial_print(//F("  ")
						);
		// Establish encrypted communications before reading the first block
		if (isSectorTrailer) {
			status = PCD_Authenticate(PICC_CMD_MF_AUTH_KEY_A, firstBlock, key, uid);
			if (status != STATUS_OK) {
				Serial_print(//F("PCD_Authenticate() failed: ")
								);
				Serial_println(//GetStatusCodeName(status)
								);
				return;
			}
		}
		// Read block
		byteCount = sizeof(buffer);
		status = MIFARE_Read(blockAddr, buffer, &byteCount);
		if (status != STATUS_OK) {
			Serial_print(//F("MIFARE_Read() failed: ")
							);
			Serial_println(//GetStatusCodeName(status)
							);
			continue;
		}
		// Dump data
		for (index = 0; index < 16; index++) {
			if(buffer[index] < 0x10)
				Serial_print(//F(" 0")
								);
			else
				Serial_print(//F(" ")
								);
			Serial_print(//buffer[index], HEX
								);
			if ((index % 4) == 3) {
				Serial_print(//F(" ")
								);
			}
		}
		// Parse sector trailer data
		if (isSectorTrailer) {
			c1  = buffer[7] >> 4;
			c2  = buffer[8] & 0xF;
			c3  = buffer[8] >> 4;
			c1_ = buffer[6] & 0xF;
			c2_ = buffer[6] >> 4;
			c3_ = buffer[7] & 0xF;
			invertedError = (c1 != (~c1_ & 0xF)) || (c2 != (~c2_ & 0xF)) || (c3 != (~c3_ & 0xF));
			g[0] = ((c1 & 1) << 2) | ((c2 & 1) << 1) | ((c3 & 1) << 0);
			g[1] = ((c1 & 2) << 1) | ((c2 & 2) << 0) | ((c3 & 2) >> 1);
			g[2] = ((c1 & 4) << 0) | ((c2 & 4) >> 1) | ((c3 & 4) >> 2);
			g[3] = ((c1 & 8) >> 1) | ((c2 & 8) >> 2) | ((c3 & 8) >> 3);
			isSectorTrailer = false;
		}
		
		// Which access group is this block in?
		if (no_of_blocks == 4) {
			group = blockOffset;
			firstInGroup = true;
		}
		else {
			group = blockOffset / 5;
			firstInGroup = (group == 3) || (group != (blockOffset + 1) / 5);
		}
		
		if (firstInGroup) {
			// Print access bits
			Serial_print(//F(" [ ")
							);
			Serial_print(//(g[group] >> 2) & 1, DEC); Serial.print(F(" ")
							);
			Serial_print(//(g[group] >> 1) & 1, DEC); Serial.print(F(" ")
							);
			Serial_print(//(g[group] >> 0) & 1, DEC
							);
			Serial_print(//F(" ] ")
							);
			if (invertedError) {
				Serial_print(//F(" Inverted access bits did not match! ")
								);
			}
		}
		
		if (group != 3 && (g[group] == 1 || g[group] == 6)) { // Not a sector trailer, a value block
//			value = (int32_t(buffer[3])<<24) | (int32_t(buffer[2])<<16) | (int32_t(buffer[1])<<8) | int32_t(buffer[0]);
			Serial_print(//F(" Value=0x")); Serial.print(value, HEX
							);
			Serial_print(//F(" Adr=0x")); Serial.print(buffer[12], HEX
							);
		}
		Serial_println();
	}
	
	return;
}


/**
 * Executes the MFRC522 MFAuthent command.
 * This command manages MIFARE authentication to enable a secure communication to any MIFARE Mini, MIFARE 1K and MIFARE 4K card.
 * The authentication is described in the MFRC522 datasheet section 10.3.1.9 and http://www.nxp.com/documents/data_sheet/MF1S503x.pdf section 10.1.
 * For use with MIFARE Classic PICCs.
 * The PICC must be selected - ie in state ACTIVE(*) - before calling this function.
 * Remember to call PCD_StopCrypto1() after communicating with the authenticated PICC - otherwise no new communications can start.
 * 
 * All keys are set to FFFFFFFFFFFFh at chip delivery.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise. Probably STATUS_TIMEOUT if you supply the wrong key.
 */
byte PCD_Authenticate(byte command,		///< PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
											byte blockAddr, 	///< The block number. See numbering in the comments in the .h file.
											MIFARE_Key *key,	///< Pointer to the Crypteo1 key to use (6 bytes)
											Uid *uid			///< Pointer to Uid struct. The first 4 bytes of the UID is used.
											) {
	byte waitIRq = 0x10;		// IdleIRq
	
	// Build command buffer
	byte sendData[12];
	sendData[0] = command;
	sendData[1] = blockAddr;
	for (byte i = 0; i < MF_KEY_SIZE; i++) {	// 6 key bytes
		sendData[2+i] = key->keyByte[i];
	}
	// Use the last uid bytes as specified in http://cache.nxp.com/documents/application_note/AN10927.pdf
	// section 3.2.5 "MIFARE Classic Authentication".
	// The only missed case is the MF1Sxxxx shortcut activation,
	// but it requires cascade tag (CT) byte, that is not part of uid.
	for (byte i = 0; i < 4; i++) {				// The last 4 bytes of the UID
		sendData[8+i] = uid->uidByte[i+uid->size-4];
	}
	
	// Start the authentication.
	return PCD_CommunicateWithPICC(PCD_MFAuthent, waitIRq, &sendData[0], sizeof(sendData), 0, 0, 0, 0, 0);
}


/**
 * Reads 16 bytes (+ 2 bytes CRC_A) from the active PICC.
 * 
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 * 
 * For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
 * The MF0ICU1 returns a NAK for higher addresses.
 * The MF0ICU1 responds to the READ command by sending 16 bytes starting from the page address defined by the command argument.
 * For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
 * A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.
 * 
 * The buffer must be at least 18 bytes because a CRC_A is also returned.
 * Checks the CRC_A before returning STATUS_OK.
 * 
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
byte MIFARE_Read(	byte blockAddr, 	///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
						byte *buffer,		///< The buffer to store the data in
						byte *bufferSize	///< Buffer size, at least 18 bytes. Also number of bytes returned if STATUS_OK.
						) {
	byte result;
	
	// Sanity check
	if (buffer == nullptr || *bufferSize < 18) {
		return STATUS_NO_ROOM;
	}
	
	// Build command buffer
	buffer[0] = PICC_CMD_MF_READ;
	buffer[1] = blockAddr;
	// Calculate CRC_A
	result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
	if (result != STATUS_OK) {
		return result;
	}
	
	// Transmit the buffer and receive the response, validate CRC_A.
	return PCD_TransceiveData(buffer, 4, buffer, bufferSize, nullptr, 0, true);
}


/**
 * Used to exit the PCD from its authenticated state.
 * Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
 */
void PCD_StopCrypto1() 
{
	// Clear MFCrypto1On bit
	PCD_ClearRegisterBitMask(Status2Reg, 0x08); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
} // End PCD_StopCrypto1()


#endif	
