

#define     LOCAL_NM        0      // Local number
#define     SENSOR_TYPE     0      // sensor type : 0 = not use,1= reopen, 2= call 
#define     BAGGAGE         0      // button type : 0 = man use,1=call(auto ready),close 2= call(any time),close(화물용)
#define     CALL_OFFSET     0      // call offset
#define     SEARCH4         'Y'
#define     SEARCH5         'S'
#define     NC_END          'E'




#define     BUTTON_CANCLE          	0    // 버튼 취소        	== 1 , 취소 불가  == 0     
#define     BUTTON_EVERY_ON			0    // 장애우 버튼 타입 	== 1 , 일반타입   == 0 
#define     LAMP_4          		0    // USER LAMP4       	== 1 , USER LAMP2 == 0 
#define     HIB_HPI_SET         	0    // HPI					== 1 , HIB        == 0 
#define     SUBDOOR_SET     		0    // 서브  도어          == 1 , 메인 도어  == 0  



#define		SETTING_VALUE			( BUTTON_CANCLE |  (BUTTON_EVERY_ON << 1) |  (LAMP_4 << 2) |  (HIB_HPI_SET << 3) | (SUBDOOR_SET << 4) )

 

extern  void    TopResetting(void);
extern  void    SetupCheck(void);


extern  void    SetupMode(void);
extern  void    LoadSetupValue(void);
extern  void    CompanyWrite(void);

extern	unsigned char new_value[];

                                    
extern  unsigned    char    SrcAddress;
extern  unsigned    char    LocalNumber;
extern  unsigned    char    Model;
extern  unsigned    char    offset_flr;
extern  unsigned    char    Company;
extern  unsigned    char    ButtonType;
extern	unsigned    char    CallOffset;

extern  unsigned    char    UpButtonTime;
extern  unsigned    char    DnButtonTime;
extern  unsigned    char    MainTimer;
extern  unsigned    char    shiftTime;


extern  bit		UpKeyBit;
extern  bit   	DnKeyBit;
extern  bit   	FDsp;
extern  bit   	CmpSetBit;
extern  bit   	MaskSetBit;
extern  bit		bToggleOn;
extern  bit   	bCurOpenKey;
extern  bit   	bUserLamp4;
extern	bit   	ChangeSetup;
extern	bit   	bHibHpiSel;
extern	bit   	bMainSubDoor;
