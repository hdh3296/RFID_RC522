
extern	void InitSPI(void);
/*
extern	void WriteSPI(unsigned char i);
extern	unsigned char ByteWriteSPI(unsigned char i);
extern  unsigned char WordWriteSPI(unsigned char addr,unsigned char thisdata);
*/

unsigned char  WriteSPI_ADDR(unsigned char addr,unsigned char thisdata);
unsigned char  ReadSPI_ADDR(unsigned char addr);

unsigned char  SPI_Play(unsigned char sel_voice);
unsigned char  SPI_Stop_Play(void);
unsigned char  SPI_Play_Status_Chk(void);


#define   	WRITE_CMD    0x00         
#define   	READ_CMD     0x40         

#define   	CS     LATA5         


#define		ADD00_SYS_CON				0x01
#define		ADD00_SBC_CON				0x02
#define		ADD00_DEC_CHAP				0x03
#define		ADD00_VOL_CON				0x04
#define		ADD00_AUDIO_CON				0x05
#define		ADD00_LED_CON				0x07
#define		ADD00_SBC_DATA				0x0A
#define		ADD00_INDEX_CHAP			0x0B
#define		ADD00_MAN_ID				0x11
#define		ADD00_Mem_TYP				0x12
#define		ADD00_Mem_CAPA				0x13
#define		ADD00_Total_CHAP_Num		0x14
#define		ADD00_SF_CRC				0x15
#define		ADD00_CRC_Result			0x16
