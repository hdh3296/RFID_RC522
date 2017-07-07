


#define    	uchar   unsigned char
#define    	uint    unsigned int
#define		byte	char

////////////////////////////////////////////////////////////////
// AddicoreRFID Registers                   
////////////////////////////////////////////////////////////////
/* The information below regarding the MFRC522 registers is from 
   the MFRC522 Manufacturer's Datasheet (Rev. 3.6 — 14 December 2011), Table 20
*/
//Page 0: Command and Status
#define     Reserved00          0x00  // reserved for future use
#define     CommandReg          0x01  // starts and stops command execution
#define     ComIEnReg           0x02  // enable and disable the passing of interrupt requests to IRq pin
#define     DivlEnReg           0x03  // enable and disable interrupt request control bits
#define     ComIrqReg           0x04  // interrupt request bits
#define     DivIrqReg           0x05  // interrupt request bits
#define     ErrorReg            0x06  // error bits showing the error status of the last command executed
#define     Status1Reg          0x07  // communication status bits
#define     Status2Reg          0x08  // receiver and transmitter status bits
#define     FIFODataReg         0x09  // input and output of 64 byte FIFO buffer
#define     FIFOLevelReg        0x0A  // number of bytes stored in the FIFO buffer
#define     WaterLevelReg       0x0B  // level for FIFO underflow and overflow warning
#define     ControlReg          0x0C  // miscellaneous control registers
#define     BitFramingReg       0x0D  // adjustments for bit-oriented frames
#define     CollReg             0x0E  // bit position of the first bit-collision detected on the RF interface
#define     Reserved01          0x0F  // reserved for future use
//Page 1: Command     
#define     Reserved10          0x10  // reserved for future use
#define     ModeReg             0x11  // defines general modes for transmitting and receiving
#define     TxModeReg           0x12  // defines transmission data rate and framing
#define     RxModeReg           0x13  // defines reception data rate and framing
#define     TxControlReg        0x14  // controls the logical behavior of the antenna driver pins TX1 and TX2
#define     TxAutoReg           0x15  // controls the setting of the transmission modulation
#define     TxASKReg			 0x15  

#define     TxSelReg            0x16  // selects the internal sources for the antenna driver
#define     RxSelReg            0x17  // selects internal receiver settings
#define     RxThresholdReg      0x18  // selects thresholds for the bit decoder
#define     DemodReg            0x19  // defines demodulator settings
#define     Reserved11          0x1A  // reserved for future use
#define     Reserved12          0x1B  // reserved for future use
#define     MfTxReg             0x1C  // controls some MIFARE communication transmit parameters
#define     MfRxReg             0x1D  // controls some MIFARE communication receive parameters
#define     Reserved13          0x1E  // reserved for future use
#define     SerialSpeedReg      0x1F  // selects the speed of the serial UART interface
//Page 2: Configuration  
#define     Reserved20          0x20  // reserved for future use
#define     CRCResultRegM       0x21  // shows the MSB values of the CRC calculation
#define		CRCResultRegH		0x21
#define     CRCResultRegL       0x22  // shows the LSB values of the CRC calculation
#define     Reserved21          0x23  // reserved for future use
#define     ModWidthReg         0x24  // controls the ModWidth setting
#define     Reserved22          0x25  // reserved for future use
#define     RFCfgReg            0x26  // configures the receiver gain
#define     GsNReg              0x27  // selects the conductance of the antenna driver pins TX1 and TX2 for modulation
#define     CWGsPReg	        0x28  // defines the conductance of the p-driver output during periods of no modulation
#define     ModGsPReg           0x29  // defines the conductance of the p-driver output during periods of modulation
#define     TModeReg            0x2A  // defines settings for the internal timer
#define     TPrescalerReg       0x2B  // defines settings for the internal timer
#define     TReloadRegH         0x2C  // defines the higher 8 bits of the 16-bit timer reload value
#define     TReloadRegL         0x2D  // defines the lower 8 bits of the 16-bit timer reload value
#define     TCounterValueRegH   0x2E  // shows the higher 8 bits of the 16-bit timer value
#define     TCounterValueRegL   0x2F  // shows the lower 8 bits of the 16-bit timer value
//Page 3: Test Registers    
#define     Reserved30          0x30  // reserved for future use
#define     TestSel1Reg         0x31  // general test signal configuration
#define     TestSel2Reg         0x32  // general test signal configuration and PRBS control
#define     TestPinEnReg        0x33  // enables pin output driver on pins D1 to D7
#define     TestPinValueReg     0x34  // defines the values for D1 to D7 when it is used as an I/O bus
#define     TestBusReg          0x35  // shows the status of the internal test bus
#define     AutoTestReg         0x36  // controls the digital self test
#define     VersionReg          0x37  // shows the software version
#define     AnalogTestReg       0x38  // controls the pins AUX1 and AUX2
#define     TestDAC1Reg         0x39  // defines the test value for TestDAC1
#define     TestDAC2Reg         0x3A  // defines the test value for TestDAC2
#define     TestADCReg          0x3B  // shows the value of ADC I and Q channels
#define     Reserved31          0x3C  // reserved for production tests
#define     Reserved32          0x3D  // reserved for production tests
#define     Reserved33          0x3E  // reserved for production tests
#define     Reserved34		    0x3F  // reserved for production tests

//AddicoreRFID Primary Coupling Device (PCD according to the ISO14443) Commands
#define PCD_IDLE                0x00  // no action, cancels current command execution
#define PCD_MEM                 0x01  // stores 25 bytes into the internal buffer
#define PCD_GENRANDOMID         0x02  // generates a 10-byte random ID number
#define PCD_CALCCRC             0x03  // activates the CRC calculation or performs a self test
#define PCD_TRANSMIT            0x04  // Transmit data
#define PCD_NOCMDCHANGE         0x07  // no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
#define PCD_RECEIVE             0x08  // activates the receiver circuits (receive data)
#define PCD_TRANSCEIVE          0x0C  // transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission (transmit and receive data)
#define PCD_MFAUTHENT           0x0E  // performs the MIFARE standard authentication as a reader (authentication)
#define PCD_SOFTRESET           0x0F  // resets the MFRC522

// AddicoreRFID Proximity Integrated Circuit Card (PICC) Commands
#define PICC_REQIDL             0x26  // search the antenna area. PCD does not enter hibernation
#define PICC_REQALL             0x52  // find all the cards in antenna area
#define PICC_ANTICOLL           0x93  // anti-collision
#define PICC_SElECTTAG          0x93  // Select card
#define PICC_AUTHENT1A          0x60  // authentication with key A
#define PICC_AUTHENT1B          0x61  // authentication with key B
#define PICC_READ               0x30  // reads one memory block
#define PICC_WRITE              0xA0  // writes one memory block
#define PICC_DECREMENT          0xC0  // decrements the contents of a block and stores the result in the internal data register
#define PICC_INCREMENT          0xC1  // increments the contents of a block and stores the result in the internal data register
#define PICC_RESTORE            0xC2  // reads the contents of a block into the internal data register
#define PICC_TRANSFER           0xB0  // writes the contents of the internal data register to a block
#define PICC_HALT               0x50  // Sleep the card

// AddicoreRFID PICC Responses
#define MI_ACK                  0xA   // The 4-bit acknowledgement returned from a MIFARE Classic PICC
#define MI_ATQA                 0x04 //The 16-bit ATQA (Answer To Request acc. to ISO/IEC 14443-4) response from PICC
#define MI_SAK                  0x08  // The 8-bit SAK (Select Acknowledge, Type A) response from PICC

// AddicoreRFID Default Definable Pins
#define DEFAULT_CHIP_SELECT_PIN         10
#define DEFAULT_RESET_PIN               5

//AddicoreRFID error codes
#define MI_OK                   0
#define MI_NO_TAG_ERR           1
#define MI_ERR                  2

//Maximum length of the array
#define MAX_LEN 16


#define false	0
#define true	1


// Return codes from the functions in this class. Remember to update GetStatusCodeName() if you add more.
// last value set to 0xff, then compiler uses less ram, it seems some optimisations are triggered
typedef enum {
	STATUS_OK				,	// Success
	STATUS_ERROR			,	// Error in communication
	STATUS_COLLISION		,	// Collission detected
	STATUS_TIMEOUT			,	// Timeout in communication.
	STATUS_NO_ROOM			,	// A buffer is not big enough.
	STATUS_INTERNAL_ERROR	,	// Internal error in the code. Should not happen ;-)
	STATUS_INVALID			,	// Invalid argument.
	STATUS_CRC_WRONG		,	// The CRC_A does not match
	STATUS_MIFARE_NACK		= 0xff	// A MIFARE PICC responded with NAK.
} StatusCode;


// Commands sent to the PICC.
typedef enum {
	// The commands used by the PCD to manage communication with several PICCs (ISO 14443-3, Type A, section 6.4)
	PICC_CMD_REQA			= 0x26,		// REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
	PICC_CMD_WUPA			= 0x52,		// Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
	PICC_CMD_CT				= 0x88,		// Cascade Tag. Not really a command, but used during anti collision.
	PICC_CMD_SEL_CL1		= 0x93,		// Anti collision/Select, Cascade Level 1
	PICC_CMD_SEL_CL2		= 0x95,		// Anti collision/Select, Cascade Level 2
	PICC_CMD_SEL_CL3		= 0x97,		// Anti collision/Select, Cascade Level 3
	PICC_CMD_HLTA			= 0x50,		// HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
	PICC_CMD_RATS           = 0xE0,     // Request command for Answer To Reset.
	// The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
	// Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
	// The read/write commands can also be used for MIFARE Ultralight.
	PICC_CMD_MF_AUTH_KEY_A	= 0x60,		// Perform authentication with Key A
	PICC_CMD_MF_AUTH_KEY_B	= 0x61,		// Perform authentication with Key B
	PICC_CMD_MF_READ		= 0x30,		// Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
	PICC_CMD_MF_WRITE		= 0xA0,		// Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
	PICC_CMD_MF_DECREMENT	= 0xC0,		// Decrements the contents of a block and stores the result in the internal data register.
	PICC_CMD_MF_INCREMENT	= 0xC1,		// Increments the contents of a block and stores the result in the internal data register.
	PICC_CMD_MF_RESTORE		= 0xC2,		// Reads the contents of a block into the internal data register.
	PICC_CMD_MF_TRANSFER	= 0xB0,		// Writes the contents of the internal data register to a block.
	// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
	// The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
	PICC_CMD_UL_WRITE		= 0xA2		// Writes one 4 byte page to the PICC.
} PICC_Command;


// MFRC522 commands. Described in chapter 10 of the datasheet.
typedef enum { 
	PCD_Idle				= 0x00, 	// no action, cancels current command execution
	PCD_Mem 				= 0x01, 	// stores 25 bytes into the internal buffer
	PCD_GenerateRandomID	= 0x02, 	// generates a 10-byte random ID number
	PCD_CalcCRC 			= 0x03, 	// activates the CRC coprocessor or performs a self-test
	PCD_Transmit			= 0x04, 	// transmits data from the FIFO buffer
	PCD_NoCmdChange 		= 0x07, 	// no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
	PCD_Receive 			= 0x08, 	// activates the receiver circuits
	PCD_Transceive			= 0x0C, 	// transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
	PCD_MFAuthent			= 0x0E, 	// performs the MIFARE standard authentication as a reader
	PCD_SoftReset			= 0x0F		// resets the MFRC522
} PCD_Command;


uint _RxBits;		  // The number of received data bits

byte AddicoreRFID_Request(byte reqMode, byte *TagType);
byte AddicoreRFID_ToCard(byte command, byte *sendData, byte sendLen, 
									byte *backData, uint *backLen);
void AntennaOn(void);
void AddicoreRFID_Reset(void);
void AddicoreRFID_Init(void);
void SetBitMask(byte reg, byte mask);
extern byte AddicoreRFID_Anticoll(byte *serNum);
extern void CalulateCRC(byte *pIndata, byte len, byte *pOutData);
extern void AddicoreRFID_Halt(void);
byte AddicoreRFID_Write(byte blockAddr, byte *_writeData);
byte AddicoreRFID_Read(byte blockAddr, byte *recvData);

void PCD_Init(void);
void PCD_AntennaOn(void);
byte PICC_IsNewCardPresent(void);
byte PICC_RequestA(	byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
											byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
										);

byte PICC_REQA_or_WUPA(	byte command, 		///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
												byte *bufferATQA,	///< The buffer to store the ATQA (Answer to request) in
												byte *bufferSize	///< Buffer size, at least two bytes. Also number of bytes returned if STATUS_OK.
											);

void PCD_ClearRegisterBitMask(	unsigned char reg,	///< The register to update. One of the PCD_Register enums.
										byte mask			///< The bits to clear.
									  );

byte PCD_TransceiveData(	byte *sendData,		///< Pointer to the data to transfer to the FIFO.
													byte sendLen,		///< Number of bytes to transfer to the FIFO.
													byte *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
													byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
													byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
													byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
													byte checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated. Default false
								 );
byte PCD_CommunicateWithPICC(	byte command,		///< The command to execute. One of the PCD_Command enums.
														byte waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
														byte *sendData,		///< Pointer to the data to transfer to the FIFO.
														byte sendLen,		///< Number of bytes to transfer to the FIFO.
														byte *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
														byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
														byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
														byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
														byte checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
									 );

byte PCD_CalculateCRC(	byte *data,		///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
												byte length,	///< In: The number of bytes to transfer.
												byte *result	///< Out: Pointer to result buffer. Result is written to result[0..1], low byte first.
					 );

void PCD_SetRegisterBitMask(	unsigned char reg,	///< The register to update. One of the PCD_Register enums.
										byte mask			///< The bits to set.
									);


