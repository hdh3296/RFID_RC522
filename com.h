

extern	void    init_comms(void);
extern  void 	USART0_TXC(void);
extern  void 	USART0_RXC(void);
extern	void	Com1TxStart(void);


#define RUNLED 			LATC5	//0
#define RXLED       	LATC4 	//0
#define TXLED       	LATD3	//0

#define TX_EN       	LATD2	//0



//////////////////////////////////////////////////
//////////////////////////////////////////////////

#define         EOT             0x04
#define         ETX             0x03
#define         ENQ             0x05
#define         ACK             0x06


#define         STX_CHK         0
#define         ADDRESS_CHK     1
#define         COMMAND_CHK     2
#define         DATACNT_CHK     3
#define         DATA_CHK        4
#define         CHKSUM_CHK      5
#define         ETX_CHK         6
#define         RX_ERROR        7
#define         RX_GOOD         8
#define         TX_SET          9
#define         CHKSUM_LOW_CHK  10
#define         CHKSUM_HIGH_CHK 11
#define         TX_ALL          12

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////




#define	COM1_MAX_TX_BUF		100
#define	COM1_MAX_RX_BUF		100

extern	unsigned 	char	Com1TxBuffer[COM1_MAX_TX_BUF];
extern	unsigned 	char	Com1RxBuffer[COM1_MAX_RX_BUF];
extern	unsigned 	char	Com1TxCnt;
extern	unsigned 	char	Com1TxThisPt;

extern	unsigned 	char   	Com1RxStatus;
extern	unsigned 	char   	Com1RxCurCnt;
extern	unsigned 	char 	Com1SerialTime;
extern	unsigned 	int		Com1BaudRate;

