
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

	 	
	//Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
    Write_AddicoreRFID(TModeReg, 0x8D);		//Tauto=1; f(Timer) = 6.78MHz/TPreScaler
    Write_AddicoreRFID(TPrescalerReg, 0x3E);	//TModeReg[3..0] + TPrescalerReg
    Write_AddicoreRFID(TReloadRegL, 30);           
    Write_AddicoreRFID(TReloadRegH, 0);
	
	Write_AddicoreRFID(TxAutoReg, 0x40);		//100%ASK
	Write_AddicoreRFID(ModeReg, 0x3D);		//CRC Initial value 0x6363	???

	//ClearBitMask(Status2Reg, 0x08);		//MFCrypto1On=0
	//Write_AddicoreRFID(RxSelReg, 0x86);		//RxWait = RxSelReg[5..0]
	//Write_AddicoreRFID(RFCfgReg, 0x7F);   		//RxGain = 48dB

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

#endif	
