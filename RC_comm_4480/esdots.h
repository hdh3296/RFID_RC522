
//#include "you_can.h" 


#include        "iodef.h"
//#include        "..\can_4480\you_can.h"

//extern   unsigned    char  LockMove;   ////////////     
//extern	 bit   LockOnce; //????????        




extern   unsigned    char  DisableFlr[4];            

extern   unsigned    char  CurStatus;       
extern   unsigned    char  CurFloor;      
extern   unsigned    char  BefCurFloor;  
extern   unsigned    char  Arrowshift;    
extern   unsigned    char  Model;         
extern   unsigned    char  htpt;
extern   unsigned    char  shift;
extern   unsigned    char  shiftTime;
extern   unsigned    char  shiftDataBuf[16];
extern   unsigned    char  MyAddress;        
extern   unsigned    char  MyLogAdr;        
extern   unsigned    char  src;
extern   unsigned    char  firstdata;
extern   unsigned    char  seconddata;
extern   unsigned    char  TimeOutBd;
extern   unsigned    char  MainTimer;
extern   unsigned    char  msec100;
extern   unsigned    char  UpButtonTime;
extern   unsigned    char  DnButtonTime;


extern   unsigned    int   SecTimer;
extern   unsigned    char  BlinkTime;
extern   unsigned    char  UpHallElev;
extern   unsigned    char  DnHallElev;
extern   unsigned    char  chartTime;




extern   unsigned    char  	ErrDspOnTimer;
extern   unsigned    char  	UpKeyCalu[MAX_ELEV];
extern   unsigned    char  	DnKeyCalu[MAX_ELEV];
extern	 unsigned 	 char  	Equal[MAX_ELEV];
extern   unsigned    char  	MoveTime[MAX_ELEV];
extern   unsigned    char  	SubDoor;
extern   unsigned    char  	TopFloor;
////////////////extern   unsigned    char  	TwoDoorDrive;
extern   unsigned    char  	CallMeAdr;

extern	 unsigned    char  MaxHighButton;
extern	 unsigned    char  MaxLowButton;
extern	 unsigned    char  MaxLongFlr;
extern	 unsigned    char  MaxButton;


extern   unsigned 	int  	xIdPt;
extern   unsigned 	char 	xtmpadr;
extern   unsigned 	char 	xTmpCurFloor;
extern   unsigned 	char 	xtmptop;
extern   unsigned 	char 	xtmpstart;
extern   unsigned 	char 	xtmpnext;
extern   unsigned 	char 	xtmphigh;
extern   unsigned 	char 	xtmplow;
extern   unsigned 	char 	xdestflr;
extern   unsigned 	char 	xyoufloor;
extern	 unsigned 	char 	Priority;
//extern	unsigned    char    SlavePrkCounter;


///////////////////
extern   unsigned    char dsplamp0,dsplamp1;

extern   const   unsigned        char    seg1[];
extern   const   unsigned        char    seg0[];

#define  F_SEG 0x71
#define  P_SEG 0xf3
#define  L_SEG 0x38
//////////////////////////


extern	bit   	UpMove;
extern	bit   	DnMove;
extern	bit   	FloorChange;
extern	bit   	ManualToggle;
extern	bit   	CarMove;
extern	bit   	KeyClr;
extern	bit     OpenLampSet;
extern	bit   	KeyClrExt;
extern	bit   	Auto;
extern	bit   	FDsp;
extern	bit   	Parking;
extern	bit   	Vip;
extern	bit   	Fire;
extern	bit   	ShiftOn;
extern	bit   	Open;
extern	bit   	Close;
extern	bit   	CurKey;
extern	bit   	UpKeyBit;
extern	bit   	DnKeyBit;
extern	bit   	Emg;
extern	bit   	StopBit;
extern	bit   	Full;
extern	bit   	OverLoad;
extern	bit   	UpDnBlinkBit;
extern	bit   	BlinkOn;
extern	bit   	StatusDspOnOff;
extern	bit   	ChangeSetup;
extern	bit   	UpKeyOn;
extern	bit   	UpKeyLong;
extern	bit   	UpKeyOneTouch;
extern	bit   	DnKeyOn;
extern	bit   	DnKeyLong;
extern	bit   	DnKeyOneTouch;
extern	bit   	Up_Key_Valid;
extern	bit   	Up_Key_Clear;
extern	bit   	Dn_Key_Valid;
extern	bit   	Dn_Key_Clear;
extern 	bit   	LowOn;
extern 	bit   	HighOn;
extern 	bit   	HostCallMe;
//extern 	bit   	SetupBit;     
extern	bit   	CmpSetBit;
extern	bit		CmpSetBit;    
extern  bit     MaskSetBit;   
extern  bit     HallLampUpBit;
extern  bit     HallLampDnBit;
extern  bit   	TogUpKeyBitSet;                     
extern  bit   	TogUpKeyBitReset;                     
extern  bit   	TogDnKeyBitSet;                     
extern  bit   	TogDnKeyBitReset;                     
extern  bit		b_vip;                    
extern  bit   	b_MyFamily;                    
extern  bit   	bIamXSubDoor;
extern  bit		bCloseReady;                    
extern  bit		bHALL_LAMP_UP;                    

				
extern	bit		bCardKeyValid;


extern  bit		bTestBit;                    

extern  bit		bToggleOn;
extern  bit   	bCurOpenKey;
extern  bit   	bUserLamp4;


extern  bit   	bCmdUpKey;
extern  bit   	bCmdDnKey;


extern	bit		bSlaveParking;
extern	bit		bParkingBd;
extern	bit		bParkingCmdValid;
extern	bit		bParkingIgnor;

extern	unsigned char ParkingKeyTime;
extern	unsigned char ParkingCmdCnt;

extern	bit		bErrorCar; 
extern	bit		bNew_Law;

