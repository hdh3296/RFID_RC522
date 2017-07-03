/*****************************************
project :       elev    voice
name    :       you sang yong
date    :       1999,9,21
******************************************/

#include    <pic18.h>

#include        "RC_system_com\memory_map.h"
#include        "RC_comm_4480\iodef.h"
#include        "RC_comm_4480\comdata.h"
#include        "RC_comm_4480\setup.h"
#include        "RC_comm_4480\spi.h"

#include        "Voice_Ext_IO_8.h"
#include		"rc522.h"

#include		"com.h"

#define	TRMT	TRMT1	
#define	OERR	OERR1
#define	TXEN	TXEN1
#define	SPEN	SPEN1
#define	FERR	FERR1



#define NormalBoard_DoorSlow	1 // ¿œπ›«¸ ∫∏µÂ∑Œ µµæÓ ΩΩ∑ŒøÏ ¡¶æÓ ±‚¥… ªÁøÎ Ω√(∏±∑π¿Ã √‚∑¬), »Æ¿ÂIO∫∏µÂ ªÁøÎΩ√ø°¥¬ µ∆ƒ¿Œ ∏∑æ∆æﬂ «—¥Ÿ. 


#define START_FL	FLOOR_B7
#define END_FL		FLOOR_B
#define VOICE_ADR  	A_VOI
#define VOICE_ON               0
#define VOICE_OFF              1

//√˛ ¡§∫∏ ¡§¿«
#define         DINGDONG_MENT          0
#define         UP_MENT                1
#define         DOWN_MENT              2
#define         HWAJAE_MENT            3
#define         OVERLOAD_MENT          4
#define         EMERGENCY_MENT         5
#define         CLOSE_MENT             6
#define         POWER_DOWN_MENT        7
#define         FLOOR_B7               8
#define         FLOOR_B6                FLOOR_B7+1  //9
#define         FLOOR_B5                FLOOR_B7+2	//10
#define         FLOOR_B4                FLOOR_B7+3	//11
#define         FLOOR_B3                FLOOR_B7+4	//12
#define         FLOOR_B2                FLOOR_B7+5	//13
#define         FLOOR_B1                FLOOR_B7+6	//14
#define         FLOOR_F1                 FLOOR_B7+7	//15
#define         FLOOR_F2                 FLOOR_B7+8	//16
#define         FLOOR_F3                 FLOOR_B7+9	//17
#define         FLOOR_F4                 FLOOR_B7+10	//18
#define         FLOOR_F5                 FLOOR_B7+11	//19
#define         FLOOR_F6                 FLOOR_B7+12	//20
#define         FLOOR_F7                 FLOOR_B7+13	//21
#define         FLOOR_F8                 FLOOR_B7+14	//22
#define         FLOOR_F9                 FLOOR_B7+15	//23
#define         FLOOR_F10                FLOOR_B7+16	//24
#define         FLOOR_F11                FLOOR_B7+17	//25
#define         FLOOR_F12                FLOOR_B7+18	//26
#define         FLOOR_F13                FLOOR_B7+19	//27
#define         FLOOR_F14                FLOOR_B7+20	//28
#define         FLOOR_F15                FLOOR_B7+21	//29
#define         FLOOR_F16                FLOOR_B7+22	//30
#define         FLOOR_F17                FLOOR_B7+23	//31
#define         FLOOR_F18                FLOOR_B7+24	//32
#define         FLOOR_F19                FLOOR_B7+25	//33
#define         FLOOR_F20                FLOOR_B7+26	//34
#define         FLOOR_F21                FLOOR_B7+27	//35
#define         FLOOR_F22                FLOOR_B7+28	//36
#define         FLOOR_F23                FLOOR_B7+29	//37
#define         FLOOR_F24                FLOOR_B7+30	//38
#define         FLOOR_F25                FLOOR_B7+31	//39
#define         FLOOR_F26                FLOOR_B7+32	//40
#define         FLOOR_F27                FLOOR_B7+33	//41
#define         FLOOR_F28                FLOOR_B7+34	//42
#define         FLOOR_F29                FLOOR_B7+35	//43
#define         FLOOR_F30                FLOOR_B7+36	//44
#define         FLOOR_F31                FLOOR_B7+37	//45
#define         FLOOR_F32                FLOOR_B7+38	//46
#define         FLOOR_G                 FLOOR_B7+39 //47
#define         FLOOR_M                 FLOOR_B7+40	//48
#define         FLOOR_L                 FLOOR_B7+41	//49
#define         FLOOR_PH                FLOOR_B7+42	//50
#define         FLOOR_B                	FLOOR_B7+43	//51
#define         CARBTN_B7          		FLOOR_B7+44	//52
#define         CARBTN_B6          		FLOOR_B7+45	//53
#define         CARBTN_B5          		FLOOR_B7+46	//54
#define         CARBTN_B4          		FLOOR_B7+47	//55
#define         CARBTN_B3          		FLOOR_B7+48	//56
#define         CARBTN_B2          		FLOOR_B7+49 //57
#define         CARBTN_B1          		FLOOR_B7+50	//58
#define         CARBTN_F1          		FLOOR_B7+51	//59
#define         CARBTN_F2          		FLOOR_B7+52	//60
#define         CARBTN_F3          		FLOOR_B7+53	//61
#define         CARBTN_F4          		FLOOR_B7+54	//62
#define         CARBTN_F5          		FLOOR_B7+55	//63
#define         CARBTN_F6          		FLOOR_B7+56	//64
#define         CARBTN_F7          		FLOOR_B7+57	//65
#define         CARBTN_F8          		FLOOR_B7+58	//66
#define         CARBTN_F9          		FLOOR_B7+59	//67
#define         CARBTN_F10         		FLOOR_B7+60	//68
#define         CARBTN_F11          	FLOOR_B7+61	//69
#define         CARBTN_F12          	FLOOR_B7+62	//70
#define         CARBTN_F13          	FLOOR_B7+63	//71
#define         CARBTN_F14          	FLOOR_B7+64	//72
#define         CARBTN_F15          	FLOOR_B7+65	//73
#define         CARBTN_F16          	FLOOR_B7+66	//74
#define         CARBTN_F17          	FLOOR_B7+67	//75
#define         CARBTN_F18          	FLOOR_B7+68	//76
#define         CARBTN_F19          	FLOOR_B7+69	//77
#define         CARBTN_F20         		FLOOR_B7+70	//78
#define         CARBTN_F21          	FLOOR_B7+71	//79
#define         CARBTN_F22          	FLOOR_B7+72	//80
#define         CARBTN_F23          	FLOOR_B7+73	//81
#define         CARBTN_F24          	FLOOR_B7+74	//82
#define         CARBTN_F25          	FLOOR_B7+75	//83
#define         CARBTN_F26          	FLOOR_B7+76	//84
#define         CARBTN_F27          	FLOOR_B7+77 //85
#define         CARBTN_F28          	FLOOR_B7+78 //86
#define         CARBTN_F29          	FLOOR_B7+79 //87
#define         CARBTN_F30          	FLOOR_B7+80 //88
#define         CARBTN_F31          	FLOOR_B7+81 //89
#define         CARBTN_F32          	FLOOR_B7+82 //90
#define         PARKING_MENT      		FLOOR_B7+83 //91
#define         OPEN_MENT      		    FLOOR_B7+84	//92
#define         CANCLE_MENT      		FLOOR_B7+85	//93
#define         PUSH_MENT      		    FLOOR_B7+86	//94
#define         SILENCE_MENT            FLOOR_B7+87 //95
#define         SONG_MENT               FLOOR_B7+88 //96
#define         BEEP_MENT               FLOOR_B7+89 //97 ªﬂ º“∏Æ
#define			CARBTN_L				FLOOR_B7+90 //98	
#define			OPPOSITE_OPEN_MENT		FLOOR_B7+91 //99
/* --> ¡÷¿« : √˛µµ¬¯ ∏‡∆Æø° ¥Î«ÿº≠¥¬ µ˘µø ¡∂∞«¿ª ≥÷æÓ ¡‡æﬂ «œπ«∑Œ <--*/
#define         FLOOR_P                	FLOOR_B7+92	//100 ∆ƒ≈∑ Floor
#define         CARBTN_M                FLOOR_B7+93	//101 

	
	
#define NO_MENT 0xff // ∏‡∆Æ æ¯¿Ω




//IO ¡§¿«
#define   		_DIPSW1        RD4     //in , ≈◊Ω∫∆Æ ∏µÂ º≥¡§
#define   		_DIPSW2        RD5     //in , ƒ´πˆ∆∞ ¿Ωº∫ º≥¡§
#define   		_DIPSW3        RD6     //in , Local º≥¡§
#define   		_DIPSW4        RD7     //in ,    "

#ifdef	CPU45K80
#define   		_VOICE_ACT      LATE0   //out
#else
#define   		_VOICE_ACT      LATA4   //out
#endif

#define   		_BATTERY       LATC0   //out
#define   		_POWER_DOWN    RC2     //in
#define   		_VOICE_DOWNLOAD_PIN  RC1     //in


#ifndef FALSE
#define FALSE               0
#endif

#ifndef TRUE
#define TRUE                1
#endif

#ifndef OFF
#define OFF                    0
#endif

#ifndef ON
#define ON                     1
#endif

#ifndef BAT_ON
#define BAT_ON	1	// High ¿œ ∂ß = ON
#endif
#ifndef BAT_OFF
#define BAT_OFF	0
#endif


//
#define ELE_nCURFLOOR          RcvBuf[IdPt + S0_FLOOR] // √˛ºˆ∏¶ 10¡¯ºˆ∑Œ «•«ˆ

#define ELE_bOPEN           (((RcvBuf[IdPt + S1_STATE] & S1_OPEN) != 0x00)?		1:0)
#define ELE_bCLOSE          (((RcvBuf[IdPt + S1_STATE] & S1_CLOSE) != 0x00)?		1:0)
#define ELE_bOVERLOAD       (((RcvBuf[IdPt + S1_STATE] & S1_OVERLOAD) != 0x00)?		1:0)
#define ELE_bEMG            (((RcvBuf[IdPt + S1_STATE] & S1_EMG) != 0x00)?		1:0)
#define ELE_bBAT            (((RcvBuf[IdPt + S1_STATE] & S1_BAT) != 0x00)?		1:0)
#define ELE_bAUTO           (((RcvBuf[IdPt + S1_STATE] & S1_AUTO) != 0x00)?		1:0)
#define ELE_bMANUAL         (((RcvBuf[IdPt + S1_STATE] & S1_MANUAL) != 0x00)?		1:0)
#define ELE_bUP             (((RcvBuf[IdPt + S1_STATE] & S1_UP) != 0x00)?		1:0)

#define ELE_bDOWN           (((RcvBuf[IdPt + S2_STATE] & S2_DN) != 0x00)?		1:0)
#define ELE_bCAR_MOVE       (((RcvBuf[IdPt + S2_STATE] & S2_CAR_MOVE) != 0x00)?		1:0)
#define ELE_bEXT_BUT_CLR    (((RcvBuf[IdPt + S2_STATE] & S2_EXT_BUT_CLR) != 0x00)?		1:0)
#define ELE_bFLOW           (((RcvBuf[IdPt + S2_STATE] & S2_FLOW) != 0x00)?		1:0) // ¿Ωº∫Ω√¿€
#define ELE_bFIRE           (((RcvBuf[IdPt + S2_STATE] & S2_FIRE) != 0x00)?		1:0)
#define ELE_bCAR_BUT_CLR    (((RcvBuf[IdPt + S2_STATE] & S2_CAR_BUT_CLR) != 0x00)?		1:0)
#define ELE_bOPEN_LAMP_ON   (((RcvBuf[IdPt + S2_STATE] & S2_OPEN_LAMP_ON) != 0x00)?		1:0)
#define ELE_bLAMP_USER      (((RcvBuf[IdPt + S2_STATE] & S2_LAMP_USER) != 0x00)?		1:0)

#define ELE_bVIP            (((RcvBuf[IdPt + S3_STATE] & S3_VIP) != 0x00)?		1:0)
#define ELE_bPARKING        (((RcvBuf[IdPt + S3_STATE] & S3_PARKING) != 0x00)?		1:0)
#define ELE_bSHIFT          (((RcvBuf[IdPt + S3_STATE] & S3_SHIFT) != 0x00)?		1:0)
#define ELE_bUPDN_VO        (((RcvBuf[IdPt + S3_STATE] & S3_UPDN_VO) != 0x00)?		1:0)
#define ELE_bOPEN_SUB       (((RcvBuf[IdPt + S3_STATE] & S3_OPEN_SUB) != 0x00)?		1:0)
#define ELE_bFULL           (((RcvBuf[IdPt + S3_STATE] & S3_FULL) != 0x00)?		1:0)
#define ELE_bCURKEY         (((RcvBuf[IdPt + S3_STATE] & S3_CURKEY) != 0x00)?		1:0)
#define ELE_bSTOP           (((RcvBuf[IdPt + S3_STATE] & S3_STOP) != 0x00)?		1:0)

#define ELE_bCAR_MOST_SERVICE   (((RcvBuf[IdPt + S4_STATE] & S4_CAR_MOST_SERVICE) != 0x00)?		1:0)
#define ELE_bFAMILY_SERVICE     (((RcvBuf[IdPt + S4_STATE] & S4_FAMILY_SERVICE) != 0x00)?		1:0)
//#define ELE_bARRIVE             (RcvBuf[IdPt + S4_STATE] & S4_ARRIVE) != 0x00)?		1:0)
#define ELE_bWAIT_FLR_SERVICE   (((RcvBuf[IdPt + S4_STATE] & S4_WAIT_FLR_SERVICE) != 0x00)?		1:0)
#define ELE_bPARKING_READY      (((RcvBuf[IdPt + S4_STATE] & S4_PARKING_READY) != 0x00)?		1:0)
#define ELE_bHALLLANTERN_UP     (((RcvBuf[IdPt + S4_STATE] & S4_HALLLANTERN_UP) != 0x00)?		1:0)
#define ELE_bHALLLANTERN_DN     (((RcvBuf[IdPt + S4_STATE] & S4_HALLLANTERN_DN) != 0x00)?		1:0)

#define ELE_DSP1         RcvBuf[IdPt + DSP1]
#define ELE_DSP2         RcvBuf[IdPt + DSP2]

#define ELE_mSYSSTATUS  RcvBuf[IdPt + SL_mSysStatus]
// I/O  ¿‘∑¬ : // 0x0 ¿Ã∏È? ¿‘∑¬ ¡¢¡°ø° Ω≈»£ ON / ¿‘∑¬ ∫Ò∆Æ ∞™¿∫ Ω«¡¶ ¿‘∑¬¿Ã æ»µÈæÓ ø√∂ß (¡Ô, ¿‘∑¬ LEDø° ∫“¿Ã æ» µÈæÓ ø√∂ß ∫Ò∆Æ 1∞™¿Ã ≈ÎΩ≈¿∏∑Œ ø¬¥Ÿ.) / ≈ÎΩ≈ ∫Ò∆Æ 0 = ¿‘∑¬ ON 
#define ELE_bIN_EMG  	(((RcvBuf[IdPt + SL_IN_EMG] & 0x01) == 0x0)?			0:1)
#define ELE_bIN_PRK 	(((RcvBuf[IdPt + SL_IN_EMG] & (0x01 << 1)) == 0x0)?		0:1)
#define ELE_bIN_AT  	(((RcvBuf[IdPt + SL_IN_EMG] & (0x01 << 2)) == 0x0)?		1:0)
#define ELE_bIN_UB  	(((RcvBuf[IdPt + SL_IN_EMG] & (0x01 << 3)) == 0x0)?		1:0)
#define ELE_bIN_DB  	(((RcvBuf[IdPt + SL_IN_EMG] & (0x01 << 4)) == 0x0)?		1:0)

#define ELE_bIN_RG  	(((RcvBuf[IdPt + SL_IN_GR] & 0x01) == 0x0)?				0:1)
#define ELE_bIN_BAT 	(((RcvBuf[IdPt + SL_IN_GR] & (0x01 << 1)) == 0x0)?		1:0)
#define ELE_bIN_PAS  	(((RcvBuf[IdPt + SL_IN_GR] & (0x01 << 2)) == 0x0)?		1:0)
#define ELE_bIN_FIR  	(((RcvBuf[IdPt + SL_IN_GR] & (0x01 << 3)) == 0x0)?		1:0)
#define ELE_bIN_WAT  	(((RcvBuf[IdPt + SL_IN_GR] & (0x01 << 4)) == 0x0)?		1:0)

#define ELE_bIN_FR1  	(((RcvBuf[IdPt + SL_IN_FIRE] & 0x01) == 0x0)?				1:0)
#define ELE_bIN_FR2 	(((RcvBuf[IdPt + SL_IN_FIRE] & (0x01 << 1)) == 0x0)?		1:0)


// I/O  √‚∑¬ / √‚∑¬¿∫ √‚∑¬ LEDø° ∫“¿Ã µÈæÓ ø√∂ß, ¡Ô √‚∑¬¿Ã ≥™∞• ∂ß √‚∑¬ ≈ÎΩ≈ ∫Ò∆Æ¥¬ 1∑Œ º¬ µ»¥Ÿ. / ≈ÎΩ≈ ∫Ò∆Æ 1 = √‚∑¬ ON
#define ELE_bIN_FAN  	(((RcvBuf[IdPt + SL_OUT_FAN] & 0x01) == 0x0)?				0:1)
#define ELE_bIN_LIT 	(((RcvBuf[IdPt + SL_OUT_FAN] & (0x01 << 1)) == 0x0)?		0:1)
#define ELE_bIN_BUZ  	(((RcvBuf[IdPt + SL_OUT_FAN] & (0x01 << 2)) == 0x0)?		0:1)
#define ELE_bIN_BEL  	(((RcvBuf[IdPt + SL_OUT_FAN] & (0x01 << 3)) == 0x0)?		0:1)
#define ELE_bIN_REL  	(((RcvBuf[IdPt + SL_OUT_FAN] & (0x01 << 4)) == 0x0)?		0:1)
//
#define ELE_bIN_RELAY  	(((RcvBuf[IdPt + SL_S5_STATE_37] & 0x01) == 0x0)?		0:1)

#define ELE_bOUT_OP  	(((RcvBuf[IdPt + SL_OUT_OP] & 0x01) == 0x0)?	0:1)
#define ELE_bOUT_CL 	(((RcvBuf[IdPt + SL_OUT_OP] & (0x01 << 1)) == 0x0)?		0:1)



//mSysStatus
#define msysDOOROPEN    57
#define msysDOORCLOSE   58



#define BEEP_DELAY_TIME	4000 // ¥‹¿ß msec


volatile const unsigned char FloorChar[] = {"B5B4B3B2B10102030405060708091011121314151617181920212223242526272829303132"};

unsigned char new_buf[64];	                                	    /*unlike the old_text this is not a CONST -> stored in data RAM */
far unsigned char * source_ptr_1 = (far unsigned char *)new_buf;    /*pointers to data*/
far unsigned char * dest_ptr_1 = (far unsigned char *)FloorChar;
unsigned char sizex = 64;


unsigned    char    TestMentDelayTimer;
unsigned    char    TmpCurVoice;
unsigned 	char 	CurVoice;

unsigned	char	CurFloorVoice;
unsigned    char    RunPgm;
unsigned    char    HwajaeVoiceCnt;
unsigned    char    OverLoadVoiceCnt;
unsigned    char    MyAddress;
unsigned    char    MyLogAdr;
unsigned    char    src;
unsigned    char    firstdata;
unsigned    char    seconddata;
unsigned    char    TimeOutBd = 0;
unsigned    char    CallMeAdr = 0;
unsigned    char    FloorXCnt;

/*
Error[000]   : _SetupDisplay (C:\micom\project\elevcan\dot_lamp\setup.obj)
Error[000]   : _DspModel (C:\micom\project\elevcan\dot_lamp\setup.obj)
*/

unsigned    char  shiftTime;
unsigned    char  MainTimer = 0;
unsigned    char  UpButtonTime = 0;
unsigned    char  DnButtonTime = 0;
unsigned    char  DspModel = 0;

unsigned    char     UpDnVoiceTimer = 0;
unsigned    char     abctimer = 0;

unsigned    char     nBefFlr = 0;
unsigned    char     nBefFlrTime = 0;

unsigned    char  YourDoor;
unsigned    char  YourDoor0;
unsigned    char  YourDoor1;
unsigned    char  YourDoor2;
unsigned    char  YourDoor3;

unsigned    char  CurCarKey[4] = {0, 0, 0, 0}; // «ˆ¿Á µÓ∑œµ» ƒ´≈∞ ƒ› ªÛ≈¬ ¿˙¿Â
unsigned    char  BefCarKey[4] = {0, 0, 0, 0};


bit FDsp;
bit UpKeyBit;
bit DnKeyBit;
bit BatteryRun;
bit Flow_Active;
bit valid_key;
bit active_key;
bit bVoicePlaying; // «ˆ¿Á ¿Ωº∫ πÊº€ ¡ﬂ¿Œ¡ˆ æ∆¥—¡ˆ
bit bCloseVoice;
bit bSetCarBtnVoice;

typedef struct
{
    unsigned char CarKey[4];
} tag_Elevator;
tag_Elevator	Elevator0, Elevator1, Elevator2, Elevator3;

typedef unsigned char UCHAR;

unsigned int    IdPt;
bit bAfterCancel;
bit bSetAfterCancel;
bit bSetSong;
bit bBeepEnab;

// Voice Play Seqence
typedef enum
{
    DINGDONG_READY_SEQ,
    CURVOICE_READY_SEQ,
    DINGDONG_PLAY_SEQ,
    ALARM_PLAYING_SEQ,
    CURFLOORVOICE_PLAY_SEQ,
    CURVOICE_PLAY_SEQ,
    CURVOICE_PLAYING_SEQ,
    END_CHK_SEQ,
    END_SEQ,
    CALCEL_READY_SEQ,
    CALCEL_PLAY_SEQ,
    Default_SEQ
} tag_Sequence;
tag_Sequence	PlaySeq;

bit bDingdong;
unsigned int BeefDelayTimer = 0;

bit bSetOppositeDoor;
bit bOppositeDoor_Enab;

unsigned int RealayTestTimer = 0;

bit bHajaeMentEn;



// «‘ºˆ¿« ø¯«¸
extern void	WaitDownLoader(void);
extern void    SetupDisplayES15(unsigned char mdata, unsigned char mode);
extern void  SetupDisplay(unsigned char ft, unsigned char sd);
extern unsigned char SyncButtonCheck(unsigned char src);
extern void    SetCarKeyCancel();
extern unsigned char DspCharRdWr();
extern void    HextoASCIIByte();
extern void    TestVoicePlay();
extern unsigned char    GetCarCallMent(unsigned char);
extern unsigned char	GetVoice_CarCall(unsigned char, unsigned char*, unsigned char*);
extern  unsigned char    GetVoice_Floor(unsigned char, unsigned char);
extern unsigned char	GetVoice_OpenCloseUpDn(unsigned char);
extern unsigned char    GetVoice_State(unsigned char, unsigned char);
extern unsigned char    GetVoice_OverLoad(unsigned char, unsigned char);
extern void    PortInit();
extern void InitVoice();
void SetVoice(void);
extern void SetDipSW();
extern unsigned char VoiceBusy();
extern unsigned char GetFloorMent();
extern unsigned char   GetVoice_Song(unsigned char);
extern unsigned char  GetVoice_BeepByBuz(unsigned char);






//##################################//
// ∏ﬁ¿Œ «‘ºˆ				    	//
//##################################//

void main(void)
{
    di();
    Initial();
    PortInit();
    Timer0Init();
    InitSPI();
    Tx1ConfirmCnt = 0;
    Tx0ConfirmCnt = 0;

	init_comms();
    ei();

    LoadSetupValue();  //120927 :SetupCheck() ?? ??
    CmpSetBit = 0;          //new
    MaskSetBit = 0;         //new

    di();
    CAN_Init();
    ei();

    InitVoice();

	
    //SetDipSW();

	TestVoicePlay();



	SetVoice(); // ∞¢¡æ º¬∆√ ø©∫Œ 
    if (bSetSong) bSetCarBtnVoice = FALSE;


	Ext_IO_8_Init();

// ¿œπ›«¸ ∫∏µÂø°º≠ »Æ¿ÂIO ∫∏µÂ∑Œ ≥—æÓ∞°∏Èº≠ ∏±∑π¿Ã IO∞° √‚∑¬ø°º≠ ¿‘∑¬¿∏∑Œ πŸ≥¢æ˙±‚ ∂ßπÆø° ....
#if   defined(NormalBoard_DoorSlow) 
	TRISC0	=	0x0;    // VOICE ¿œπ› ∫∏µÂ ¿Œ ∞ÊøÏ, ∏±∑π¿Ã √‚∑¬¿∏∑Œ º≥¡§ 
#endif


    while (1)
    {
        CLRWDT();

		Ext_IO_8_Func();


        // Voice Downlod ¡ﬂ...
        if (_VOICE_DOWNLOAD_PIN)
        {
            WaitDownLoader();
        }

        if (ELE_bIN_RELAY)
		{
            _BATTERY = BAT_ON;
		}	
        else
		{
            _BATTERY = BAT_OFF;
		}

        DspCharRdWr(); // CAR CALL ¿Ωº∫¿ª ¿ß«— ø§∏Æ∫£¿Ã≈Õ ∞¢√˛ µΩ∫«√∑π¿Ã ∞™ ¿˙¿Â
        SetCarKeyCancel(); // CAR CALL √Îº“ ∞™ º¬∆√

        TmpCurVoice = NO_MENT;
        // Beef ∏‡∆Æ ∞¸∑√
        if (ELE_bIN_BUZ)
        {
            TmpCurVoice = GetVoice_OverLoad(TmpCurVoice, CurVoice);
            TmpCurVoice = GetVoice_Floor(TmpCurVoice, GetFloorMent());

            if (BeefDelayTimer > BEEP_DELAY_TIME)
            {
                if (bBeepEnab)
                {
                    TmpCurVoice = GetVoice_BeepByBuz(TmpCurVoice);
                }

                if (TmpCurVoice == BEEP_MENT)	BeefDelayTimer = 0;
            }
        }
        else
        {
            TmpCurVoice = GetVoice_State(TmpCurVoice, CurVoice);
            if (bDingdong == FALSE) TmpCurVoice = GetVoice_OpenCloseUpDn(TmpCurVoice);
            TmpCurVoice = GetVoice_Floor(TmpCurVoice, GetFloorMent());
            if (bSetCarBtnVoice) TmpCurVoice = GetVoice_CarCall(TmpCurVoice, CurCarKey, BefCarKey);
            if (bSetSong) TmpCurVoice = GetVoice_Song(TmpCurVoice);
        }


        if (TmpCurVoice != NO_MENT)
        {
            CurVoice = TmpCurVoice;
            _VOICE_ACT = VOICE_ON;
            if (bVoicePlaying)
            {
                SPI_Stop_Play(); // ¿œ¥‹, ±‚¡∏ πÊº€ ¡ﬂ¿Ã¥¯ ¿Ωº∫ ¡ﬂ¡ˆ !
                if ((CurVoice >= START_FL) && (CurVoice <= END_FL))
                {
                    PlaySeq = DINGDONG_READY_SEQ;
                    CurFloorVoice = CurVoice;
                }
                else
                {
                    PlaySeq = CURVOICE_READY_SEQ;
                }
            }
            else
            {
				/* --> ¡÷¿« ¿Ã ¡∂∞«æ»ø° µÈæÓ∞°æﬂ µ˘µø¿Ã ≥™ø¬¥Ÿ. <--*/
                if (((CurVoice >= START_FL) && (CurVoice <= END_FL)) || (CurVoice == FLOOR_P))   // √˛ µµ¬¯ !
                {
                    PlaySeq = DINGDONG_PLAY_SEQ;
                    CurFloorVoice = CurVoice;
                }
                else
                {
                    PlaySeq = CURVOICE_PLAY_SEQ;
                }
            }
        }


        switch (PlaySeq)
        {
        case DINGDONG_READY_SEQ:
            if (bVoicePlaying == FALSE)
            {
                PlaySeq = DINGDONG_PLAY_SEQ;
            }
            break;
        case DINGDONG_PLAY_SEQ:
            SPI_Play(DINGDONG_MENT); // µµ¬¯ æÀ∏≤ µ˘µø !
            bDingdong = TRUE;
            PlaySeq = ALARM_PLAYING_SEQ;
            break;
        case ALARM_PLAYING_SEQ:
            if (bVoicePlaying)
            {
                PlaySeq = CURVOICE_READY_SEQ;
            }
            break;
        case CURVOICE_READY_SEQ:
            if (bVoicePlaying == FALSE)
            {
                if (CurVoice & 0x80)
                    PlaySeq = END_CHK_SEQ;
                else
                    PlaySeq = CURVOICE_PLAY_SEQ;

                if (bDingdong)
                    PlaySeq = CURFLOORVOICE_PLAY_SEQ;
            }
            break;
        case CURFLOORVOICE_PLAY_SEQ:
            SPI_Play(CurFloorVoice);
            bDingdong = FALSE;
			if(bSetOppositeDoor)	bOppositeDoor_Enab = TRUE;
            PlaySeq = CURVOICE_PLAYING_SEQ;
            break;
        case CURVOICE_PLAY_SEQ:
            SPI_Play(CurVoice); // µµ¬¯ '∏Ó √˛¿‘¥Ÿ','πÆ¿Ãø≠∏≥¿Ã¥Ÿ','¥›»¸¥œ¥Ÿ' µÓ æ»≥ªπÊº€ √‚∑¬
            PlaySeq = CURVOICE_PLAYING_SEQ;
            break;
        case CURVOICE_PLAYING_SEQ:
            if (bVoicePlaying)
            {
                PlaySeq = END_CHK_SEQ;
                if (bAfterCancel)
                {
                    bAfterCancel = FALSE;
                    PlaySeq = CALCEL_READY_SEQ;
                }
            }
            break;
        case END_CHK_SEQ:
            if (bVoicePlaying == FALSE)
            {
                PlaySeq = END_SEQ;
                _VOICE_ACT = VOICE_OFF;
            }
            break;
        case CALCEL_READY_SEQ:
            if (bVoicePlaying == FALSE)
            {
                PlaySeq = CALCEL_PLAY_SEQ;
            }
            break;
        case CALCEL_PLAY_SEQ:
            SPI_Play(CANCLE_MENT); // µµ¬¯ æÀ∏≤ µ˘µø !
            PlaySeq = CURVOICE_PLAYING_SEQ;
            break;
        }


        if (VoiceBusy())
        {
            bVoicePlaying = TRUE;
        }
        else
        {
            if (PlaySeq == END_SEQ)
            {
                CurVoice = NO_MENT;
                bBeepEnab = TRUE;
                bDingdong = FALSE;
            }
            bVoicePlaying = FALSE;
        }
    }
}


//##################################//
// ¿Œ≈Õ∑¥∆Æ «‘ºˆ			    	//
//##################################//

void interrupt isr(void)
{
//    unsigned char i;


    if (TMR0IF)
    {

        TMR0IF = 0 ;
        TMR0L = MSEC_L;
        TMR0H = MSEC_H;

        TestMentDelayTimer++;
        shiftTime++;

        if (BeefDelayTimer < 0xffff) BeefDelayTimer++;
		if (RealayTestTimer < 0xffff) RealayTestTimer++;

        abctimer++;
        if (abctimer > 100)
        {
            abctimer = 0;
            if (UpDnVoiceTimer < 200)	UpDnVoiceTimer++;
            if (nBefFlrTime < 200)	nBefFlrTime++;

            if (FloorXCnt < 200)	FloorXCnt++;


        }

		Ext_IO_8_Timer_1msec();

    }


#ifdef	CPU45K80
    if (PIR5 > 0)
    {
        CanInterrupt();
    }
#else
    if (PIR3 > 0)
    {
        CanInterrupt();
    }
#endif


/* 232 ÏãúÎ¶¨Ïñº */ 

	if((RCIE)&&(RCIF))										/*receive interrupt routine*/
	{
        RCIF=0;
        USART0_RXC();
	}	

	if((TXIE)&&(TXIF))										/*transmit interrupt routine*/
	{
        TXIF=0;
        USART0_TXC();
	}	


	if(OERR) {
      	TXEN=0;
      	TXEN=1;
      	SPEN=0;
      	SPEN=1;
		CREN1=0;
    }

	if( !CREN1)	CREN1=1;
	


}


//##################################//
// ¿œπ› «‘ºˆµÈ                      				//
//##################################//


unsigned char GetFloorMent(void)
{
    unsigned char tmMent;
    unsigned char cDotDsp, bValid;

    bValid = TRUE;
    // Dot1 √º≈© (10¿« ¿⁄∏Æ)
    cDotDsp = ELE_DSP1;
    if (cDotDsp == 'B')
    {
        tmMent = FLOOR_F1;
    }
    else if (cDotDsp == 'P')
    {
        tmMent = FLOOR_F1;
    }
    else if (cDotDsp == 'G')
    {
        tmMent = FLOOR_F1;
    }
    else if (cDotDsp == '0')
    {
        tmMent = FLOOR_B1;
    }
    else if (cDotDsp == '1')
    {
        tmMent = 10 + FLOOR_B1;
    }
    else if (cDotDsp == '2')
    {
        tmMent = 20 + FLOOR_B1;
    }
    else if (cDotDsp == '3')
    {
        tmMent = 30 + FLOOR_B1;
    }
    else
    {
        //tmMent = (RcvBuf[IdPt] | 0x80);
        bValid = FALSE;
        tmMent = 0xff;
    }

    // Dot2 √º≈©(1¿« ¿⁄∏Æ)
    if (bValid)
    {
        cDotDsp = ELE_DSP2;
        if (cDotDsp == '0')
        {
            if (tmMent < (10 + FLOOR_B1))
            {
                tmMent = 0xff;
            }
        }
        else if (cDotDsp == 'F')
        {
            if (ELE_DSP1 == 'G')
            {
                tmMent = FLOOR_G;
            }
            else
            {
                tmMent = (tmMent + 4);
            }
        }
        else if (cDotDsp == 'G')
        {
            tmMent = FLOOR_G;
        }
        else if (cDotDsp == 'M')
        {
            tmMent = FLOOR_M;
        }
        else if (cDotDsp == 'H')
        {
            tmMent = FLOOR_PH;
        }
        else if (cDotDsp == 'L')
        {
            tmMent = FLOOR_L;
        }
        else if (cDotDsp == 'B')
        {
            tmMent = FLOOR_B;
        }
        else if (cDotDsp == 'P')
        {
            tmMent = FLOOR_P;
        }		
        else if ((cDotDsp >= '1') && (cDotDsp <= '9'))
        {
            if (tmMent == FLOOR_F1)
                tmMent = tmMent - (cDotDsp - '0');
            else
                tmMent = tmMent + (cDotDsp - '0');
        }
        else
        {
            tmMent = 0xff;
        }
    }
    return tmMent;
}


unsigned char    GetVoice_Floor(unsigned char xTmpCurVoice, unsigned char xCurFloorMent)
{
    static unsigned char befFloorMent = 0;
    static bit bmentSaveEnab;

    if (xTmpCurVoice != NO_MENT)
        return xTmpCurVoice;

    if (xCurFloorMent != 0xfe)
    {
        if (ELE_bAUTO)   // ¿⁄µø !
        {
            if (ELE_bFLOW)
            {
                if (bmentSaveEnab && (xCurFloorMent != befFloorMent))
                {
                    xTmpCurVoice = befFloorMent = xCurFloorMent;
                    bmentSaveEnab = FALSE;
                    bBeepEnab = FALSE;
                }
            }
            else
            {
                bmentSaveEnab = TRUE;
            }
        }
        else   // ¿⁄µø¿Ã æ∆¥— ∞ÊøÏ !
        {
            if (xCurFloorMent != befFloorMent)
            {
                xTmpCurVoice = befFloorMent = xCurFloorMent;
                bBeepEnab = FALSE;
            }
        }
    }

    return xTmpCurVoice;
}

// πÆ¿Ã ø≠∏≥¥œ¥Ÿ, ¥›»¸¥œ¥Ÿ, ø√∂Û∞©¥œ¥Ÿ, ≥ª∑¡∞©¥œ¥Ÿ.
unsigned char   GetVoice_OpenCloseUpDn(unsigned char xTmpCurVoice)
{
    static unsigned char xUpDnVoice = 0x0;
    static bit xbOpened = FALSE;
    static bit xbUpDned = FALSE;

    if (xTmpCurVoice != NO_MENT)
        return xTmpCurVoice;

    if (ELE_bPARKING_READY)
        return xTmpCurVoice;

    if (ELE_bAUTO)   // ¿⁄µø !
    {
        // Up, Down Ω√, ¿Ωº∫ √≥∏Æ
        if (ELE_bOPEN && (UpDnVoiceTimer > 40) 
			&& xbOpened && (xbUpDned == FALSE))
        {
            if (ELE_bUP)
            {
                xTmpCurVoice = UP_MENT; // ø√∂Û∞©¥œ¥Ÿ !
                xbUpDned = TRUE;
            }
            else if (ELE_bDOWN)
            {
                xTmpCurVoice = DOWN_MENT; // ≥ª∑¡ ∞©¥œ¥Ÿ !
                xbUpDned = TRUE;
            }
        }
        // Door Open Ω√, ¿Ωº∫ √≥∏Æ
        if (ELE_bCAR_MOVE)
        {
            UpDnVoiceTimer = 0;
            xbUpDned = FALSE;
        }      

		// open √‚∑¬¿Ã ≥™∞•∂ß open ∏‡∆Æ ≥™∞°µµ∑œ ºˆ¡§ 2017-03-09
		if ( (ELE_bOUT_OP) && (xbOpened == FALSE) && !bVoicePlaying )	//open
		{
			if (bOppositeDoor_Enab) 
			{
				bOppositeDoor_Enab = FALSE;
				xTmpCurVoice = OPPOSITE_OPEN_MENT; // π›¥Î∆Ì πÆ¿Ã ø≠∏≥¥œ¥Ÿ.
			}
			else	
			{
				xTmpCurVoice = OPEN_MENT; // πÆ¿Ã ø≠∏≥¥œ¥Ÿ !
			}
			xbOpened = TRUE;
			UpDnVoiceTimer = 0;
		}
		else if (xbOpened && ELE_bOUT_CL)   // close : πÆ¿Ã ¿ÃπÃ ø≠∑¡¿÷¥¬ ªÛ≈¬ø°º≠ close √‚∑¬¿Ã ≥™ø¿∏È 
		{
			xTmpCurVoice = CLOSE_MENT; // πÆ¿Ã ¥›»¸¥œ¥Ÿ !
			xbOpened = FALSE;
		}

		
    }
    else   // ¿⁄µø¿Ã æ∆¥“ ∂ß !
    {
        if (ELE_bCLOSE)   // πÆ¿Ã ¥›«˚¿ª ∂ß !
        {
            if (xbUpDned == FALSE)
            {
                if (ELE_bUP && (xUpDnVoice != UP_MENT))
                {
                    xTmpCurVoice = xUpDnVoice = UP_MENT;
                    xbUpDned = TRUE;
                }
                else if (ELE_bDOWN && (xUpDnVoice != DOWN_MENT))
                {
                    xTmpCurVoice = xUpDnVoice = DOWN_MENT;
                    xbUpDned = TRUE;
                }
            }
        }
        else
        {
            xUpDnVoice = 0;
            xbUpDned = FALSE;
        }
    }

    return(xTmpCurVoice);
}


unsigned char   GetVoice_Song(unsigned char xBefVoice)
{
    unsigned char tmCurVoice;
    static bit bSong = FALSE;

    if (xBefVoice != NO_MENT)
        return xBefVoice;

    if (ELE_bCAR_MOVE && ELE_bAUTO)
    {
        if (ELE_bFLOW) bSong = FALSE;
        if (!VoiceBusy() && bSong && ELE_bUP)
        {
            tmCurVoice = SONG_MENT;
            return tmCurVoice;
        }
        else if (!VoiceBusy() && bSong && ELE_bDOWN)
        {
            tmCurVoice = SONG_MENT;
            return tmCurVoice;
        }
    }
    else
    {
        bSong = TRUE;
    }
    return xBefVoice;
}


unsigned char  GetVoice_BeepByBuz(unsigned char xTmpCurVoice)
{

    if (xTmpCurVoice != 0xff) // ¿ÃπÃ ¥Ÿ∏• ¿Ωº∫¿Ã µÓ∑œµ«æÓ ¿÷¥Ÿ∏È !
        return xTmpCurVoice;

    if (ELE_bIN_BUZ) // ∫Œ¿˙ √‚∑¬
    {
        if (CurVoice != BEEP_MENT)
            xTmpCurVoice = BEEP_MENT;
    }

    return xTmpCurVoice;
}




unsigned char    GetVoice_State(UCHAR xTmpCurVoice, UCHAR xCurVoice)
{
    static unsigned char EmergencyVoiceCnt = 0;

    if (xTmpCurVoice != NO_MENT)
        return xTmpCurVoice;

    // ∆ƒ≈∑
    if (ELE_bPARKING_READY  && (xCurVoice != PARKING_MENT))
    {
        xTmpCurVoice = PARKING_MENT;
    }
    if (ELE_bPARKING  && (xCurVoice == PARKING_MENT))
    {
        if (bVoicePlaying)
        {
            SPI_Stop_Play();
            bVoicePlaying = FALSE;
        }
    }

    // ø¿πˆ∑ŒµÂ
    if ((ELE_bOPEN || ELE_bOPEN_SUB) && ELE_bOVERLOAD && (xCurVoice != OVERLOAD_MENT))
    {
        xTmpCurVoice = OVERLOAD_MENT;
    }

    // »≠¿Á 
	if (!ELE_bFIRE)
	{
		bHajaeMentEn = TRUE;
	}
	
	if (ELE_bIN_FR1)
	{
		bHajaeMentEn = FALSE;
	}	
	
    if (ELE_bFIRE && (xCurVoice != HWAJAE_MENT) && bHajaeMentEn)
    {
        xTmpCurVoice = HWAJAE_MENT;
    }

    // ∫ÒªÛ
    // [151006] ¿⁄µøΩ√ø°∏∏ EMG MENT √‚∑¬ µ«µµ∑œ ºˆ¡§
    if (ELE_bEMG  && (xCurVoice != EMERGENCY_MENT) && ELE_bIN_AT)
    {
        if (EmergencyVoiceCnt < 10)	EmergencyVoiceCnt++;
        if (EmergencyVoiceCnt < 6)
            xTmpCurVoice = EMERGENCY_MENT;
    }
    if (ELE_bEMG == FALSE)
        EmergencyVoiceCnt = 0;

    // πÂµ•∏Æ
    if (ELE_bBAT  && (xCurVoice != POWER_DOWN_MENT))
    {
        xTmpCurVoice = POWER_DOWN_MENT;
    }

    return xTmpCurVoice;

}

unsigned char    GetVoice_OverLoad(UCHAR xTmpCurVoice, UCHAR xCurVoice)
{
    static unsigned char EmergencyVoiceCnt = 0;

    if (xTmpCurVoice != 0xff)
        return xTmpCurVoice;

    // ø¿πˆ∑ŒµÂ
    if ((ELE_bOPEN || ELE_bOPEN_SUB) && ELE_bOVERLOAD)
    {
        bBeepEnab = FALSE;
        if (xCurVoice != OVERLOAD_MENT)
            xTmpCurVoice = OVERLOAD_MENT;
    }

    return xTmpCurVoice;

}



//ƒ´πˆ∆∞ ¿Ωº∫ √≥∏Æ
unsigned char   GetVoice_CarCall(UCHAR xTmpCurVoice, UCHAR *xCurkey, UCHAR *xBefkey)
{
    unsigned char j;
    unsigned char iFloor; // 0~31
    unsigned char bitKey;

    if (xTmpCurVoice != NO_MENT)
        return xTmpCurVoice;
    if (!ELE_bAUTO || ELE_bFLOW)
        return xTmpCurVoice;
    if (!ELE_bUP && !ELE_bDOWN)
    {	
		if (!ELE_bVIP)
        	return xTmpCurVoice;
    }
    if (CurVoice != 0xff)
        return xTmpCurVoice;

    bitKey = 0x01;
    j = 0;
    for (iFloor = 0; iFloor < 32; iFloor++)
    {
        if (xCurkey[j] != xBefkey[j])   //8∞≥¿« ƒ´≈∞ ªÛ≈¬ ∫Ò±≥
        {
            // «— ∞≥¿« ƒ´≈∞ ªÛ≈¬ ∫Ò±≥
            if ((xCurkey[j] & bitKey) &&  !(xBefkey[j] & bitKey)) // ƒ´ƒ› µÓ∑œ¿Ã∏È? (¬¸ && !∞≈¡˛)
            {
                if (iFloor != (ELE_nCURFLOOR - 1)) // ƒ´ƒ› µÓ∑œ √˛¿Ã «ˆ¿Á √˛∞˙ ∞∞¡ˆ æ ¿ª ∂ß¥¬ √‚∑¬¿ª ≥ª∫∏≥Ω¥Ÿ. 
                {
                    xBefkey[j] = (xBefkey[j] | bitKey);
                    xTmpCurVoice = GetCarCallMent(iFloor);
                }
                break;
            }
            else if (!(xCurkey[j] & bitKey) && (xBefkey[j] & bitKey)) //ƒ´ƒ› √Îº“¿Ã∏È? (!∞≈¡˛ && ¬¸)
            {
                xBefkey[j] = (xBefkey[j] & ~bitKey);

                if (bSetAfterCancel)
                {
                    bAfterCancel = TRUE;
                    xTmpCurVoice = GetCarCallMent(iFloor);
                }
                else
                {
                    xTmpCurVoice = CANCLE_MENT;
                }
                break;
            }

            bitKey = (bitKey << 1);
            if (bitKey == 0)
            {
                j++;
                bitKey = 0x01;
            }
        }
        else
        {
            j++;
            bitKey = 0x01;
            iFloor = (iFloor + 7);
        }
    }
    return xTmpCurVoice;
}



// ƒ´ƒ› ≈∞ ªÛ≈¬ ∞™¿ª CurVoideø° ¿˙¿Â«—¥Ÿ.
unsigned char    GetCarCallMent(unsigned char Call_Floor)
{
    unsigned char   cDotDsp, i, bValid;
    UCHAR tmMent;

    bValid = TRUE;
    // Dot1
    i = (Call_Floor * 2);
    cDotDsp = FloorChar[i + 0];
    if (cDotDsp == 'B')
    {
        tmMent = CARBTN_F1;
    }
    else if (cDotDsp == '0')
    {
        tmMent = CARBTN_B1;
    }
    else if (cDotDsp == '1')
    {
        tmMent = 10 + CARBTN_B1;
    }
    else if (cDotDsp == '2')
    {
        tmMent = 20 + CARBTN_B1;
    }
    else if (cDotDsp == '3')
    {
        tmMent = 30 + CARBTN_B1;
    }
    else
    {
        bValid = FALSE;
        tmMent = 0xff;
    }
    // Dot2
    if (bValid)
    {
        cDotDsp = FloorChar[i + 1];
		if (cDotDsp == 'L')
		{
			tmMent = CARBTN_L;
		}
		else if (cDotDsp == 'M')
		{
			tmMent = CARBTN_M;
		}		
        else if (cDotDsp == '0')
        {
            if (tmMent < (10 + CARBTN_B1))
                tmMent = 0xff;
        }
        else if (cDotDsp == 'F')
        {
            tmMent = (tmMent + 4);
        }
        else if ((cDotDsp >= '1') && (cDotDsp <= '9'))
        {
            if (tmMent == CARBTN_F1) // ¡ˆ«œ√˛¿Ã∏È?
                tmMent = tmMent - (cDotDsp - '0');
            else // ¡ˆªÛ√˛¿Ã∏È?
                tmMent = tmMent + (cDotDsp - '0');
        }
        else
        {
            tmMent = 0xff;
        }
    }
    return tmMent;
}



void    SetCarKeyCancel(void)
{
    unsigned char bBefBtnClr;
    unsigned char i, j, k;

    bBefBtnClr = OFF;

    if (!ELE_bUP && !ELE_bDOWN)
    {
		if (!ELE_bVIP)
        	bBefBtnClr = ON;
    }
	
    if (ELE_bCAR_BUT_CLR)
    {
        bBefBtnClr = ON;
    }

    if (bBefBtnClr == ON)
    {
        CurCarKey[0] = Elevator0.CarKey[0] = Elevator1.CarKey[0]
                                             = Elevator2.CarKey[0] = Elevator3.CarKey[0] = BefCarKey[0] = 0;
        CurCarKey[1] = Elevator0.CarKey[1] = Elevator1.CarKey[1]
                                             = Elevator2.CarKey[1] = Elevator3.CarKey[1] = BefCarKey[1] = 0;
        CurCarKey[2] = Elevator0.CarKey[2] = Elevator1.CarKey[2]
                                             = Elevator2.CarKey[2] = Elevator3.CarKey[2] = BefCarKey[2] = 0;
        CurCarKey[3] = Elevator0.CarKey[3] = Elevator1.CarKey[3]
                                             = Elevator2.CarKey[3] = Elevator3.CarKey[3] = BefCarKey[3] = 0;
    }

    if (ELE_bOPEN || ELE_bOPEN_SUB)
    {
        k = (ELE_nCURFLOOR - 1);
        i = (k / 8);
        j = (k % 8);
        k = 0x01;
        k = (k << j);

        BefCarKey[i] = (BefCarKey[i] & ~k);
        CurCarKey[i] = (CurCarKey[i]  & ~k);
        Elevator0.CarKey[i] = (Elevator0.CarKey[i] & ~k);
        Elevator1.CarKey[i] = (Elevator1.CarKey[i] & ~k);
        Elevator2.CarKey[i] = (Elevator2.CarKey[i] & ~k);
        Elevator3.CarKey[i] = (Elevator3.CarKey[i] & ~k);
    }
}


// ¿Ωº∫ ¥ŸøÓ∑ŒµÂ
void	WaitDownLoader(void)
{

    SSPEN = 0;

    SSPCON1 = 0;

    TRISC3 = 1; //sck
    TRISC4 = 1; //sdi
    TRISC5 = 1; //sdo
    TRISA5 = 1; //ss

    do
    {
        CLRWDT();
    }
    while (_VOICE_DOWNLOAD_PIN);


    while (1);

}


void    SetupDisplayES15(unsigned char mdata, unsigned char mode)
{
}

void  SetupDisplay(unsigned char ft, unsigned char sd)
{
}


unsigned char SyncButtonCheck(unsigned char src)
{
    unsigned char i;

    if (RxEidBuffer & HOST_BOARD)   // Host !
    {
        if (SrcAddress  == LocalNumber)	return(1);
        else							return(0);
    }
    else   // Slave !
    {
        if (SrcAddress  == LocalNumber)
        {
            i = (unsigned char)RxEidBuffer;
            if ((i == 0x80) || (i == 0x81) || (i == 0x82) || (i == 0x83))   // CAR Board
            {
                return(1);
            }
            else
            {
                return(0);
            }
        }
        else	return(0);
    }
}


// CarKey µÓ∑œ
void    MyLampCheck(void)
{
    unsigned int i, j;
    i = 0;
    j = 0;

    i = (unsigned char)RxEidBuffer;

    switch (i)   // Car Address
    {
    case    0x80:
        YourDoor0 = EqualDataBuf[2]; // mCarOpCl
        Elevator0.CarKey[0] = EqualDataBuf[3];
        Elevator0.CarKey[1] = EqualDataBuf[4];
        Elevator0.CarKey[2] = EqualDataBuf[5];
        Elevator0.CarKey[3] = EqualDataBuf[6];
        j = 1;
        break;
    case    0x81:
        YourDoor1 = EqualDataBuf[2];
        Elevator1.CarKey[0] = EqualDataBuf[3];
        Elevator1.CarKey[1] = EqualDataBuf[4];
        Elevator1.CarKey[2] = EqualDataBuf[5];
        Elevator1.CarKey[3] = EqualDataBuf[6];
        j = 1;
        break;
    case    0x82:
        YourDoor2 = EqualDataBuf[2];
        Elevator2.CarKey[0] = EqualDataBuf[3];
        Elevator2.CarKey[1] = EqualDataBuf[4];
        Elevator2.CarKey[2] = EqualDataBuf[5];
        Elevator2.CarKey[3] = EqualDataBuf[6];
        j = 1;
        break;
    case    0x83:
        YourDoor3 = EqualDataBuf[2];
        Elevator3.CarKey[0] = EqualDataBuf[3];
        Elevator3.CarKey[1] = EqualDataBuf[4];
        Elevator3.CarKey[2] = EqualDataBuf[5];
        Elevator3.CarKey[3] = EqualDataBuf[6];
        j = 1;
        break;
    }

    if (j == 1)
    {
        YourDoor  = YourDoor0   | YourDoor1   | YourDoor2   | YourDoor3;
        CurCarKey[0] = Elevator0.CarKey[0] | Elevator1.CarKey[0] |
                       Elevator2.CarKey[0] | Elevator3.CarKey[0];
        CurCarKey[1] = Elevator0.CarKey[1] | Elevator1.CarKey[1] |
                       Elevator2.CarKey[1] | Elevator3.CarKey[1];
        CurCarKey[2] = Elevator0.CarKey[2] | Elevator1.CarKey[2] |
                       Elevator2.CarKey[2] | Elevator3.CarKey[2];
        CurCarKey[3] = Elevator0.CarKey[3] | Elevator1.CarKey[3] |
                       Elevator2.CarKey[3] | Elevator3.CarKey[3];
    }
}


// CAR CALL ¿Ωº∫¿ª ¿ß«— ø§∏Æ∫£¿Ã≈Õ ∞¢√˛ µΩ∫«√∑π¿Ã ∞™ ¿˙¿Â
unsigned char DspCharRdWr(void)
{
    unsigned char   CurFlrNum;

    if (ELE_nCURFLOOR > 32)	return(0);
    if (ELE_nCURFLOOR < 1)	return(0);

    if (nBefFlr == ELE_nCURFLOOR)
    {
        nBefFlrTime = 0;
    }
    else
    {
        if (nBefFlrTime > 3)
        {
            nBefFlrTime = 0;
            nBefFlr = ELE_nCURFLOOR;
            CurFlrNum = ELE_nCURFLOOR - 1;
            CurFlrNum = (CurFlrNum * 2);
            if ((ELE_DSP1 != FloorChar[CurFlrNum]) || (ELE_DSP2 != FloorChar[CurFlrNum + 1]))
            {
                new_buf[CurFlrNum + 0] = ELE_DSP1;
                new_buf[CurFlrNum + 1] = ELE_DSP2;
                flash_write(source_ptr_1, sizex, dest_ptr_1);
            }
        }
    }
    return(0);
}




void    HextoASCIIByte(void)
{
    unsigned char   i;

    i = (src & 0xf0) >> 4;
    if (i < 0x0a)   i = i + '0';
    else         i = i + '7';
    firstdata = i;

    i = (src & 0x0f);
    if (i < 0x0a)   i = i + '0';
    else         i = i + '7';
    seconddata = i;
}


uchar str[MAX_LEN];
volatile char mytest;
volatile char mytest1,mytest2,mytest3;



void  Serial2Check(void)
{
	unsigned char i;

    if(Com1RxStatus==RX_GOOD){      
		//for(i=0;i<(Com1RxBuffer[3]+4);i++)	Can1TxBuf[i]=Com1RxBuffer[i];
		//Can1TxDataTxPointer=2;	
		//Can1TxDataTotalCnt= (Com1RxBuffer[3]+4);
        //Com1RxStatus=STX_CHK; 
		//RXLED = !RXLED;
    }                         
}





volatile uchar checksum1;
void    TestVoicePlay(void)
{
    unsigned bBusy;
	uchar status;
	

    _VOICE_ACT = VOICE_OFF; // Î¶¨ÏÖã ÌïòÏù¥ : RFID Ï†ïÏÉÅ ÎèôÏûë 

	AddicoreRFID_Init();

    for (;1;)
    {
        CLRWDT();

		Serial2Check();

		if(Com1RxStatus == TX_SET){
			if(Com1SerialTime > 3){
				Com1SerialTime=0;
				Com1RxStatus=STX_CHK; 
				Com1TxThisPt=0;
				Com1TxCnt=0;
				TXIE=0;
				TX_EN=0;
			}
		}

		// RFID ÌÉúÍ∑∏Ïùò ÌÉÄÏûÖÏùÑ Î¶¨ÌÑ¥
		status = AddicoreRFID_Request(PICC_REQIDL, str);    
		if (status == MI_OK)    // MIFARE Ïπ¥ÎìúÏùºÎïåÎßå ÏûëÎèô
		{
			mytest = str[0];
			mytest1 = str[1];
		}	

		// RFID Ï∂©ÎèåÎ∞©ÏßÄ, RFID ÌÉúÍ∑∏Ïùò IDÍ∞í(ÏãúÎ¶¨ÏñºÎÑòÎ≤Ñ) Îì± Ï†ÄÏû•Îêú Í∞íÏùÑ Î¶¨ÌÑ¥Ìï®. 4Byte
    	status = AddicoreRFID_Anticoll(str);
    	if (status == MI_OK)      // MIFARE Ïπ¥ÎìúÏùºÎïåÎßå ÏûëÎèô
    	{			
         	mytest = str[0];
			mytest1 = str[1];			
			mytest2 = str[2];
			mytest3 = str[3];
			checksum1 = str[0] ^ str[1] ^ str[2] ^ str[3];

			Com1TxStart();	
    	}

		AddicoreRFID_Halt();  // ÎèôÏûë Ï§ëÏßÄ ÏãúÌÇ§Îäî Í±¥Îç∞ ÎèôÏûë ÏïàÌïòÎäîÍ±∞ Í∞ôÎã§. 
    }

}



/*******************************************************
        manual play and record
*******************************************************/

void    PortInit(void)
{
    TRISE2 = 1;	  // port E data direction...(output)
    TRISE1 = 1;
    TRISE0 = 1;

// TRISD Register
    TRISD7 = 1;	// port D data direction....(output)
    TRISD6 = 1;
    TRISD5 = 1;
    TRISD4 = 1;
    TRISD3 = 1;
    TRISD2 = 1;
    TRISD1 = 1;
    TRISD0 = 1;

// TRISC Register
    TRISC7 = 1;	// port C data direction
    TRISC6 = 1;
    TRISC5 = 1;
    TRISC4 = 1;
    TRISC3 = 1;
    TRISC2 = 1;
    TRISC1 = 1;
    TRISC0 = 1;

// TRISB Register
    TRISB7 = 1;	// port B data direction
    TRISB6 = 1;
    TRISB5 = 1;
    TRISB4 = 1;
    TRISB3 = 1; //can rx
    TRISB2 = 1; //can tx
    TRISB1 = 1; //in
    TRISB0 = 1; //in

// TRISA Register
    TRISA7 = 1;	// port A data direction
    TRISA6 = 1;
    TRISA5 = 1;

    TRISA3 = 1;
    TRISA2 = 1;
    TRISA1 = 1;
    TRISA0 = 1;

    TRISC0 = 0;

// LATE Register
    LATE2 = 1;	// port E data latch
    LATE1 = 1;
    LATE0 = 1;

// LATD Register
    LATD7 = 1;	// port D data latch
    LATD6 = 1;
    LATD5 = 1;
    LATD4 = 1;
    LATD3 = 1;
    LATD2 = 1;
    LATD1 = 1;
    LATD0 = 1;


// LATC Register
    LATC7 = 1;	// port C data latch
    LATC6 = 1;
    LATC5 = 1;
    LATC4 = 1;
    LATC3 = 1;
    LATC2 = 1;
    LATC1 = 1;
    LATC0 = 1;

// LATB Register
    LATB7 = 1;	// port B data latch
    LATB6 = 1;
    LATB5 = 1;
    LATB4 = 1;
    LATB3 = 1;
    LATB2 = 1;
    LATB1 = 1;
    LATB0 = 1;

// LATA Register
    LATA7 = 1;	// port A data latch
    LATA6 = 1;
    LATA5 = 1;
    LATA3 = 1;
    LATA2 = 1;
    LATA1 = 1;
    LATA0 = 1;

#ifndef		CPU45K80
    TRISA4 = 0;
    LATA4 = 1;
#else
    TRISE0 = 0;
    LATE0 = 1;
#endif
}


void SetDipSW()
{
    if (_DIPSW3 && _DIPSW4)         LocalNumber = 0;
    else if (!_DIPSW3 && _DIPSW4)   LocalNumber = 1;
    else if (_DIPSW3 && !_DIPSW4)   LocalNumber = 2;
    else if (!_DIPSW3 && !_DIPSW4)  LocalNumber = 3;

    IdPt = (LocalNumber * HOST_DATA_RECOD) + RCV_DATA;
    RcvBuf[IdPt+S0_FLOOR] = 0x0;

    if (!_DIPSW2)
    {
        bSetCarBtnVoice = TRUE;
    }
    else
    {
        bSetCarBtnVoice = FALSE;
    }
    if (!_DIPSW1)
    {
        
    }
}


unsigned char VoiceBusy(void)
{
    if (SPI_Play_Status_Chk()) // ¿Ωº∫∏‡∆Æ πÊº€¡ﬂ¿Ã∏È OUT1 LEDø° ∫“¿Ã µÈæÓø¬¥Ÿ.
        return TRUE;
    else
        return FALSE;
}


void InitVoice(void)
{
    unsigned char i;

    FloorXCnt = 0;
    for (; FloorXCnt < 10;)
    {
        CLRWDT();
    }
    FloorXCnt = 0;

    WriteSPI_ADDR(ADD00_SYS_CON, 0x08);
    WriteSPI_ADDR(ADD00_SBC_CON, 0x0);
    WriteSPI_ADDR(ADD00_AUDIO_CON, 0x02);
    WriteSPI_ADDR(ADD00_LED_CON, 0xff);
    WriteSPI_ADDR(ADD00_SYS_CON, 0x0);

    _VOICE_ACT = VOICE_OFF;
    SPI_Stop_Play();

    MyAddress = VOICE_ADR;
    RunPgm = 0x55;
    CanDatacnt0 = 2;
    CanDatacnt1 = 2;
    CanCmd = 0;

    for (i = 0; i < 64; i++)
    {
        new_buf[i] = FloorChar[i];
        nBefFlr = 0;
        nBefFlrTime = 0;
    }

    RunPgm = 0x0;
    valid_key = 0;
    active_key = 0;
    BatteryRun = 0;
    Flow_Active = 0;
    CurVoice = 0xff;
    PlaySeq = Default_SEQ;
    bVoicePlaying = 0;
    TmpCurVoice = 0xff;
    HwajaeVoiceCnt = 0;
    OverLoadVoiceCnt = 0;
    bAfterCancel = FALSE;
    bBeepEnab = TRUE;
	bOppositeDoor_Enab = FALSE;
	_BATTERY = BAT_OFF;
	bHajaeMentEn = TRUE;
}

void SetVoice(void)
{
    bSetAfterCancel = TRUE; // ƒ´ƒ› √Îº“Ω√ √˛∏‡∆Æ »ƒ √Îº“ ≥™ø¿∞‘ «“¡ˆ ø©∫Œ 
    bSetSong = FALSE; // ¡÷«‡ ¡ﬂ ¿Ωæ« 
	bSetOppositeDoor = FALSE; // π›¥Î∆Ì πÆ¿Ã ø≠∏≥¥œ¥Ÿ.
}






