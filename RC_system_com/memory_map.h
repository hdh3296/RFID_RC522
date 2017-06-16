


//#define		FLOOR_64				1				

#define 	COMPANY          		'A'   


#define     CAN_KEY_DOOR    		0
#define     CAN_KEY_CLEAR   		1
#define     CAN_KEY_READY   		2
#define     CAN_KEY_SET     		3
#define     CAN_KEY_CONFIRM 		4
#define     CAN_PARKING_COMMAND 	0x82
#define     CAN_EARTHQUAKE_COMMAND 	0x83
#define     CAN_FIRE_COMMAND 		0x84
#define     CAN_VIP_COMMAND 		0x88

#define     ALL_KEY_CLEAR   		0xfd
#define     CAN_NO_KEY_SET  		0xfe
#define     CAN_INVALID_CMD 		0xff
  

#define     PC_COMMAND      		0x80
#define     CMD_PARKING     		0x01
#define     CMD_EVEN_ODD    		0x02
#define     CMD_MORNING     		0x03
#define     CMD_FIRE        		0x04
#define     CMD_FAMILY_SEV  		0x05
#define     CMD_CALL_SEV			0x06
#define     CMD_EXT_CALL_SEV		0x07
#define     CMD_VIP					0x08
#define     CMD_SECOND_FIRE			0x09
#define     CMD_FAN_LIGHT_ETC		0x0a


#define 	UP_READY          		0x80
#define 	DN_READY          		0x40
#define 	UPDN_READY        		0xc0




#ifdef		FLOOR_64
	#define     IAM_BASE_FLR     	0
	#define 	ONLY_FLR          	0x3f
#else
	#define 	CAR_READY         	0x20
	#define 	UP_CAR_READY      	0xA0
	#define 	DN_CAR_READY      	0x60
	#define 	UPDN_CAR_READY    	0xe0
	#define 	ONLY_FLR          	0x1f
#endif
		



#define  	S1_OPEN           		0x01
#define  	S1_CLOSE          		0x02
#define  	S1_OVERLOAD       		0x04
#define  	S1_EMG            		0x08
#define  	S1_BAT            		0x10
#define  	S1_AUTO           		0x20
#define  	S1_MANUAL         		0x40
#define  	S1_UP             		0x80

#define  	S2_DN             		0x01
#define  	S2_CAR_MOVE       		0x02
#define  	S2_EXT_BUT_CLR    		0x04
#define  	S2_FLOW           		0x08
#define  	S2_FIRE           		0x10
#define  	S2_CAR_BUT_CLR    		0x20
#define  	S2_OPEN_LAMP_ON   		0x40    
#define  	S2_LAMP_USER      		0x80

#define  	S3_VIP            		0x01
#define  	S3_PARKING        		0x02
#define  	S3_SHIFT          		0x04
#define  	S3_UPDN_VO        		0x08
#define  	S3_OPEN_SUB       		0x10
#define  	S3_FULL           		0x20
#define  	S3_CURKEY         		0x40    
#define  	S3_STOP           		0x80

#define  	S4_CAR_MOST_SERVICE    	0x01
#define  	S4_FAMILY_SERVICE		0x02
#define  	S4_SECOND_FIRE			0x04
#define  	S4_WAIT_FLR_SERVICE		0x08
#define  	S4_PARKING_READY		0x10
#define  	S4_HALLLANTERN_UP		0x20
#define  	S4_HALLLANTERN_DN		0x40
#define  	S4_NEW_LAW_SYSTEM		0x80





#define  	S0_FLOOR       		0                       //StatusBuf + 0
#define  	S1_STATE        	1                       //StatusBuf + 1 
#define  	S2_STATE        	2                       //StatusBuf + 2 
#define  	S3_STATE        	3                       //StatusBuf + 3 
#define  	S4_STATE        	4                       //StatusBuf + 4 
#define  	DEST_FLR        	5                       //StatusBuf + 5
#define  	DSP1            	6                       //StatusBuf + 15 - 19
#define  	DSP2            	7                       //StatusBuf + 15 - 19
#define  	FLR_ON_OFF0     	8                       //StatusBuf + 15 - 19
#define  	FLR_ON_OFF1     	9                       //StatusBuf + 15 - 19
#define  	FLR_ON_OFF2     	10                      //StatusBuf + 15 - 19
#define  	FLR_ON_OFF3     	11                      //StatusBuf + 15 - 19
#define  	mEqualFloor     	12
#define  	mHighFloor      	13
#define  	mLowFloor       	14
#define  	mDoor           	15
#define  	mCarOpCl        	16
#define  	mCarKey1        	17
#define  	mCarKey9        	18
#define  	mCarKey17       	19
#define  	mCarKey25       	20
#define  	mSysStatus      	21
#define  	O_OP            	22
#define  	O_U_W           	23
#define  	O_Y_0           	24
#define  	I_EMG           	25
#define  	I_SU1           	26
#define  	I_GR            	27
#define     I_FIRE          	28
#define  	mMostLongDst    	29
#define  	mStartFloor     	30
#define  	mTopFlr         	31
#define  	mMpmL      			32
#define  	mMpmH           	33
#define  	mExtIN0         	34
#define  	I_X_0           	35
#define  	I_FS0           	36
#define  	S5_STATE_37      	37
#define  	mCrtExtMoveFlr  	38	
#define  	mCallMe         	39												
#define  	FLR_ON_OFF4       	40                       
#define  	FLR_ON_OFF5       	41                       
#define  	FLR_ON_OFF6       	42                      
#define  	FLR_ON_OFF7       	43                     
#define  	mCarKey33         	44
#define  	mCarKey41         	45
#define  	mCarKey49         	46
#define  	mCarKey57         	47

#define  	HibUpCall0        	48                       
#define  	HibUpCall1        	49                       
#define  	HibUpCall2        	50                      
#define  	HibUpCall3        	51                     
#define  	HibUpCall4        	52                       
#define  	HibUpCall5        	53                       
#define  	HibUpCall6        	54                      
#define  	HibUpCall7        	55                     
#define  	HibDnCall0        	56                       
#define  	HibDnCall1        	57                       
#define  	HibDnCall2        	58                      
#define  	HibDnCall3        	59                    
#define  	HibDnCall4        	60                       
#define  	HibDnCall5        	61                       
#define  	HibDnCall6        	62                      
#define  	HibDnCall7        	63               

#define  	TestBuf        		64               

#define  	Year              	80
#define  	Month             	81
#define  	Day               	82
#define  	Hour              	83
#define  	Minuate           	84
#define  	sec               	85

#define  	mcurfloor         	200
#define  	mBefcurfloor      	201
#define  	mReqStopFloor     	202
#define  	mAckStopFloor     	203
#define  	mNewAckStopFloor  	204
#define  	mCarMoveState     	205
#define  	mDoorSeq          	206
#define  	mBefErrCntPt      	207
#define  	LD_INSP           	208
#define  	FHM_SEQ           	209
#define  	SelDoorStatus     	210
#define  	mFireSeq          	211
#define  	mVFlrCnt          	212
#define  	mVFlr1            	213
#define  	mVFlr2            	214
#define  	AUTO_TUNING       	215
#define  	AUTO_TUNING_SEQ   	216
#define  	mCurCarkey        	217
#define  	mLuLdFloor        	218
#define  	mRunFloor    		219
#define  	mInvFloor    		220
#define  	mEV_LastErr   		221
//#define  	mLoader_mSysStatus  222

//#define  	FHM_RUN           	230   // delete  delete

#define  	EndsRam           	254
#define  	END_SRAM          EndsRam+1







#define     SL_mCarOpCl         16
#define     SL_mCarKey1         17
#define     SL_mCarKey9         18
#define     SL_mCarKey17        19
#define     SL_mCarKey25        20
#define     SL_mSysStatus       21   //status nm
#define     SL_OUT_OP       	22   //o_op
#define     SL_OUT_UP       	23   //o_up_ward
#define     SL_OUT_FAN          24   //o_fan
#define     SL_IN_EMG           25   //i_emg
#define     SL_IN_SU1       	26   //i_su1
#define     SL_IN_GR       		27   //i_gr
#define     SL_IN_FIRE       	28   //i_fire
#define     SL_mMostLongDst     29   
#define     SL_mStartFloor      30
#define     SL_mTopFlr          31
#define     SL_mUnKnown21       32  //mpm
#define     SL_mUnKnown22       33  //mpm
#define     SL_mEXT_IN0       	34    
#define     SL_IN_X0       		35  //i_x0 
#define     SL_IN_FS0       	36	//i_fs0
#define     SL_S5_STATE_37      37
#define     SL_mCrtExtMoveFlr   38
#define     SL_mCallMe          39

#ifdef		FLOOR_64
	#define  	SL_FLR_ON_OFF4  40                       
	#define  	SL_FLR_ON_OFF5  41                       
	#define  	SL_FLR_ON_OFF6  42                      
	#define  	SL_FLR_ON_OFF7  43                     
	#define     SL_mCarKey33	44
	#define     SL_mCarKey41    45
	#define     SL_mCarKey49    46
	#define     SL_mCarKey57    47
#endif



#define  LOG_ADR        0   
#define  SLAVE1         1
#define  SLAVE2         2
#define  SLAVE3         3
#define  SLAVE4         4
#define  SLAVE5         5
#define  SLAVE6         6
#define  SLAVE7         7
#define  SLAVE8         8
#define  SLAVE9         9
#define  SLAVE10        10
#define  SLAVE11        11
#define  SLAVE12        12
#define  SLAVE13        13
#define  SLAVE14        14
#define  SLAVE15        15
#define  SLAVE16        16
#define  SLAVE17        17
#define  SLAVE18        18
#define  SLAVE19        19
#define  SLAVE20        20
#define  SLAVE21        21
#define  SLAVE22        22
#define  SLAVE23        23
#define  SLAVE24        24
#define  SLAVE25        25
#define  SLAVE26        26
#define  SLAVE27        27
#define  SLAVE28        28
#define  SLAVE29        29
#define  SLAVE30        30
#define  SLAVE31        31
#define  SLAVE32        32
#define  A_VOI          33
#define  B_VOI          34
#define  C_VOI          35
#define  D_VOI          36
#define  A_CAR1         37
#define  A_CAR2         38
#define  A_CAR3         39
#define  A_CAR4         40
#define  B_CAR1         41
#define  B_CAR2         42
#define  B_CAR3         43
#define  B_CAR4         44
#define  C_CAR1         45
#define  C_CAR2         46
#define  C_CAR3         47
#define  C_CAR4         48
#define  D_CAR1         49
#define  D_CAR2         50
#define  D_CAR3         51
#define  D_CAR4         52
#define  A_XO           53   
#define  A_X1           54
#define  B_XO           55
#define  B_X1           56
#define  C_XO           57
#define  C_X1           58
#define  D_XO           59
#define  D_X1           60   
#define  MAS_A          61
#define  B_M1           62
#define  B_M2           63
#define  C_M1           64
#define  C_M2           65
#define  D_M1           66
#define  D_M2           67
#define  A_LD           68
#define  B_LD           69
#define  C_LD           70
#define  D_LD           71
#define  A_PC           72
#define  B_PC           73
#define  C_PC           74
#define  D_PC           75




#define  	SET_ULS_POSITION    0
#define  	REAL_ULS_POSITION   1
#define  	SET_SUS_POSITION    2
#define  	REAL_SUS_POSITION   3
#define  	SET_SUS1_POSITION   4
#define  	REAL_SUS1_POSITION  5
#define  	SET_X0_POSITION    	6
#define  	REAL_X0_POSITION    7
#define  	SET_DLS_POSITION    8
#define  	REAL_DLS_POSITION   9
#define  	SET_SDS_POSITION    10
#define  	REAL_SDS_POSITION   11
#define  	SET_SDS1_POSITION   12
#define  	REAL_SDS2_POSITION  13
#define  	SET_X1_POSITION    	14
#define  	REAL_X1_POSITION    15
#define  	NC00_POSITION      	16
#define  	NC01_POSITION      	17
#define  	NC02_POSITION      	18
#define  	NC03_POSITION      	19
#define  	POSITION_END      	20



#define  BUT_CANLE_T          220
#define  MAX_CALL_TIME        20
