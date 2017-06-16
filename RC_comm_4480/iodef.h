
//ver5->6
//keysort.c, comdata.c  ,  car11.c ,iodef.h
//hall lantern  ¼öÁ¤
//car11.c --> carlift ¿ë  one button valid



#include        "..\RC_can_4480\you_can.h"
#include        "cpu18f4480.h"



//#define		NEW_CAR						1
//#define		NEW_HIB_HPI					1
//#define		NEW_ES15					1
#define		NEW_VOICE					1
//#define		NEW_CAN_IO_32_BCD			1
//#define		NEW_CAN_IO_32_FLR_ONOFF		1
//#define		NEW_CAN_IO_8				1
//#define		NEW_CAN_O_8					1
//#define		POWER_CONTROL				1
//#define		NEW_CAN_IO_HOST				1


//#define		OLD_CAR						1
//#define		OLD_HIB_HPI					1
//#define		OLD_ES15					1
//#define		OLD_VOICE					1
//#define		OLD_CAN_IO_32_FLR_ONOFF		1


#if  defined(NEW_CAR)   ||  defined(NEW_HIB_HPI) || defined(NEW_CAN_IO_32_BCD) || defined(NEW_CAN_IO_32_FLR_ONOFF) || defined(NEW_CAN_IO_HOST)
	#define		 	U_COMPILER		1
	#define		 	CPU65K80		1

	#if  defined(NEW_CAR)
		#define __TYPE_CAR			1      
	#elif   defined(NEW_HIB_HPI)
		#define __TYPE_HIB_HPI		1      
	#endif
#elif   defined(NEW_ES15) || defined(NEW_VOICE)  || defined(NEW_CAN_IO_8) || defined(POWER_CONTROL) ||	defined(NEW_CAN_O_8)
	#define		 	U_COMPILER		1
	#define			CPU45K80		1

	#if  defined(NEW_ES15)
		#define __TYPE_ES15			1   
	#endif   

#elif   defined(OLD_CAR)
	#define __TYPE_CAR      
#elif   defined(OLD_HIB_HPI)
	#define __TYPE_HIB_HPI      
#elif   defined(OLD_ES15)
	#define __TYPE_ES15      
#endif


#if   defined(NEW_VOICE) 
	#define __TYPE_ES15			1   
#endif



#if	defined(NEW_CAN_IO_8) || defined(NEW_CAN_O_8) || defined(NEW_CAN_IO_32_BCD) || defined(NEW_CAN_IO_32_FLR_ONOFF) || defined(OLD_CAN_IO_32_FLR_ONOFF) || defined(NEW_CAN_IO_HOST) || defined(POWER_CONTROL)
	#define CAN_IO			1   
#endif



//#define __TYPE_DIRECT




#define     EXT_VERSION     	0x08

#define     SUB_DOOR_BIT    	0x01
#define		HIB_PARKING_BIT 	0x02 
#define     HALL_SENSOR     	0x04
#define     IAM_SINGLE_MODE     0x08

#define     OTHER_CAR_CLEAR     0x40
#define     IAM_FLR32     		0x80


//#define     YOUNGJIN_ESCORT     1
//#define     ARRIVE_BELL     	1


//#define     CARD_KEY     1


//#define	CAN_IO				1
//#define	I_AM_FLOOR_ONOFF	1
//#define	I_AM_EXT_IO_ONOFF	1

/*//////////////////////////////////////////////////////////////////////
////// if 	I_AM_FLOOR_ONOFF  ////////////////////////////////////////	
	#define		=	OLD_ES15		
	cpu			=	18f4480
	compiler	=	HI_TECH PICC-18 Toolsuit  	
//////////////////////////////////////////////////////////////////////

////// if 	I_AM_EXT_IO_ONOFF  ////////////////////////////////////////	
	#define		=	NEW_ES15		
	cpu			=	18f45k80
	compiler	=	HI_TECH universal Toolsuit  	
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////*/

/*
#ifdef	CAN_IO 
	#define		 	U_COMPILER		1
	#define		 	CPU65K80		1

	#if  defined(I_AM_EXT_IO_ONOFF)   ||  defined(I_AM_FLOOR_ONOFF)
		#if    defined(I_AM_EXT_IO_ONOFF)   &&  defined(I_AM_FLOOR_ONOFF) 
		    #error  "Multiple Define Error(iodef.h)" 
		    #error  "Multiple Define Error(iodef.h)" 
		#endif
	#else
	    #error  "CAN_IO Define Error(iodef.h)" 
	    #error  "CAN_IO Define Error(iodef.h)" 
	#endif
#endif
*/




/*
#if     defined(__TYPE_ES15)    &&  defined(__TYPE_HIB_HPI) && defined(__TYPE_CAR)
    #error  "Multiple Define Error(iodef.h)" 
    #error  "Multiple Define Error(iodef.h)" 
    #error  "Multiple Define Error(iodef.h)" 
    #error  "Multiple Define Error(iodef.h)" 
#elif   defined(__TYPE_ES15)    &&  defined(__TYPE_HIB_HPI)
    #error  "Multiple Define Error(iodef.h)" 
    #error  "Multiple Define Error(iodef.h)" 
    #error  "Multiple Define Error(iodef.h)" 
    #error  "Multiple Define Error(iodef.h)" 
#elif   defined(__TYPE_ES15)    &&  defined(__TYPE_CAR)
    #error  "Multiple Define Error(iodef.h)" 
    #error  "Multiple Define Error(iodef.h)" 
    #error  "Multiple Define Error(iodef.h)" 
    #error  "Multiple Define Error(iodef.h)" 
#elif   defined(__TYPE_HIB_HPI) &&  defined(__TYPE_CAR)
    #error  "Multiple Define Error(iodef.h)" 
    #error  "Multiple Define Error(iodef.h)" 
    #error  "Multiple Define Error(iodef.h)" 
    #error  "Multiple Define Error(iodef.h)" 
#endif
*/




#if     defined(__TYPE_ES15)
	#if  defined(__TYPE_HIB_HPI) || defined(__TYPE_CAR)  || defined( CAN_IO)
	    #error  "Multiple Define Error(iodef.h)" 
	    #error  "Multiple Define Error(iodef.h)" 
	    #error  "Multiple Define Error(iodef.h)" 
	    #error  "Multiple Define Error(iodef.h)"
	#endif 
#elif   defined(__TYPE_HIB_HPI)
	#if  defined(__TYPE_ES15) || defined(__TYPE_CAR)  || defined( CAN_IO)
	    #error  "Multiple Define Error(iodef.h)" 
	    #error  "Multiple Define Error(iodef.h)" 
	    #error  "Multiple Define Error(iodef.h)" 
	    #error  "Multiple Define Error(iodef.h)"
	#endif 
#elif   defined(__TYPE_CAR)
	#if  defined(__TYPE_ES15) || defined(__TYPE_HIB_HPI)   || defined( CAN_IO)
	    #error  "Multiple Define Error(iodef.h)" 
	    #error  "Multiple Define Error(iodef.h)" 
	    #error  "Multiple Define Error(iodef.h)" 
	    #error  "Multiple Define Error(iodef.h)"
	#endif 
#elif   defined(CAN_IO)
	#if  defined(__TYPE_ES15) || defined(__TYPE_HIB_HPI)   || defined(__TYPE_CAR)
	    #error  "Multiple Define Error(iodef.h)" 
	    #error  "Multiple Define Error(iodef.h)" 
	    #error  "Multiple Define Error(iodef.h)" 
	    #error  "Multiple Define Error(iodef.h)"
	#endif 
#else
    #error  "No Define Error(iodef.h)" 
    #error  "No Define Error(iodef.h)" 
    #error  "No Define Error(iodef.h)" 
    #error  "No Define Error(iodef.h)"
#endif




#if defined(__TYPE_HIB_HPI)

    #define  __DSP_DOT
   
	#ifdef	CPU65K80

    	#define  HIB_PARKING_KEY    PORTBbits.RB0

    	#define  UP_KEY         RE0
	    #define  DN_KEY         RE1
	    #define  UP_KEY_SUB     RE2
	       
	    #define  HALL_LAMP_UP   LATE4
	    #define  HALL_LAMP_DN   LATE5
	    #define  UP_KEY_LAMP    LATE6
	    #define  DN_KEY_LAMP    LATE7
	
	    #define  MANUAL_LAMP    LATG1		//user lamp1
	    #define  AUTO_LAMP    	LATG0		//user lamp2
	    #define  USER_LAMP3     LATG2
	    #define  USER_LAMP4     LATG3
	           
	
	    #define  P0             LATD
	    #define  P1             LATC
	    #define  P2             LATF


	    #define  P10             LATC0
	    #define  P11             LATC1
	    #define  P12             LATC2
	    #define  P13             LATC3
	    #define  P14             LATC4
	    #define  P15             LATC5
	    #define  P16             LATC6
	    #define  P17             LATC7
	
	    #define  P20             LATF0
	    #define  P21             LATF1
	    #define  P22             LATF2
	    #define  P23             LATF3
	    #define  P24             LATF4
	    #define  P25             LATF5
	    #define  P26             LATF6
	    #define  P27             LATF7
	#else

	    #define  UP_KEY         RD0
	    #define  DN_KEY         RD1
	    #define  UP_KEY_SUB     RD7   
	    
	    #define  WR_CLK         LATE2    
	    #define  IN_ACT         LATB1
	    
	    #define  UP_KEY_LAMP    LATB4
	    #define  DN_KEY_LAMP    LATB5
	    #define  AUTO_LAMP      LATB6
	    #define  MANUAL_LAMP    LATB7            
	    #define  HALL_LAMP_DN   LATB0
	
	
	    #define  P0             LATD
	    #define  P1             LATA
	    #define  P2             LATC
	
	    #define  P10             LATA0
	    #define  P11             LATA1
	    #define  P12             LATA2
	    #define  P13             LATA3
	    #define  P14             LATA4
	    #define  P15             LATA5
	    #define  P16             LATE0
	    #define  P17             LATE1
	
	    #define  P20             LATC0
	    #define  P21             LATC1
	    #define  P22             LATC2
	    #define  P23             LATC3
	    #define  P24             LATC4
	    #define  P25             LATC5
	    #define  P26             LATC6
	    #define  P27             LATC7
	#endif;

#elif defined(__TYPE_ES15)
	#if defined(__TYPE_DIRECT)
	    #define  UP_KEY         RB0  
	    #define  DN_KEY         RB0
  	    
	    #define  UP_KEY_LAMP    LATC6  
	    #define  DN_KEY_LAMP    LATC6  	    
	    #define  HALL_LAMP_UP   LATC6
	    #define  HALL_LAMP_DN   LATC6
	    #define  AUTO_LAMP      LATC6


	    #define  BCD1_LAMP      LATE0  //Y0
	    #define  BCD2_LAMP      LATE1  //Y1
	    #define  BCD3_LAMP      LATE2  //Y2
	    #define  BCD4_LAMP      LATC0  //Y3
	    #define  UP_LAMP        LATC1  //Y4 
	    #define  DN_LAMP        LATC2  //Y5
	    #define  DOOR_OPEN_LAMP LATC3  //Y6
	    #define  FULL_LAMP      LATD0  //Y7
	    #define  PARKING_LAMP   LATD1  //Y8
	    #define  MANUAL_LAMP    LATD2  //Y9
	    #define  EMG_LAMP_B   	LATD3  //Y10
	    #define  EMG_LAMP   	LATC4  //Y11
	#else
	    #define  NOT_KEY        RA5  
	    #define  NOT_KEY1       RE2  //RC5
	    #define  UP_KEY         RE1  //RC7
	    #define  DN_KEY         RE0  //RC6
	    
	    #define  UP_KEY_LAMP    LATC7  //LATE1
	    #define  DN_KEY_LAMP    LATC6  //LATE0
	    #define  UP_LAMP        LATC5  //LATE2
	    
	    #define  HALL_LAMP_UP   LATB0
	    #define  HALL_LAMP_DN   LATB1
	    #define  DN_LAMP        LATB4
	    #define  AUTO_LAMP      LATB5
	    #define  MANUAL_LAMP    LATB6
	    #define  FULL_LAMP      LATB7
	    
	    #define  P0             LATD  // 0000 0000 
	    #define  P2             LATC
	
	
	    #define  SEG_A          LATD0 
	    #define  SEG_B          LATD1 
	    #define  SEG_C          LATD2 
	    #define  SEG_D        	LATD3 
	    #define  SEG_E        	LATD4 
	    #define  SEG_F       	LATD5 
	    #define  SEG_G1       	LATD6 
	    #define  SEG_G2         LATD7
 

	    #define  SEG_adg        LATC0 
	    #define  SEG_b_S        LATC1 
	    #define  SEG_c_S        LATC2 
	    #define  SEG_e_S        LATC3 
	    #define  SEG_n12        LATC4 

	#endif

#elif defined(__TYPE_CAR)

    #define  __DSP_DOT

	#ifdef	CPU65K80
	    #define     DATA_PORT     LATD
	    #define     IN_DATA_PORT  PORTD
	    
	    #define     SEL0          LATB0
	    #define     SEL1          LATB1
	    #define     SEL2          LATB4
	    #define     SEL_ACT       LATB5
	    #define  	MANUAL_LAMP   LATB6
	    #define  	AUTO_LAMP     LATB7
	            
	
	    #define     TOGGLE        RA0
	    #define     DIPS2         RA2
	    #define     DIPS3         RA1
	    #define     MULTI_ADR0    RA3
	    #define     MULTI_ADR1    RA5
	
	    
	    #define     OPEN_KEY       RG0
	    #define     OPEN_LAMP     LATG1
	    #define     CLOSE_KEY      RG2
	    #define     CLOSE_LAMP    LATG3
	    #define     BELL          LATG4
                    
	#else
	    #define     DATA_PORT     LATD
	    #define     IN_DATA_PORT  PORTD
	    #define     P1            LATC
	    
	    #define     SEL0          LATC0
	    #define     SEL1          LATC1
	    #define     SEL2          LATC2
	    #define     SEL_ACT       LATC3
	
	    #define     BELL          LATC4  
	    #define     DIPS2         RC5
	    #define     DIPS3         RC6
	    #define     LIVE_LAMP     LATC7  
	            
	    #define     MULTI_ADR0    RB0
	    #define     MULTI_ADR1    RB1
	    
	    #define     NO_USE1       RB4
	    #define     TOGGLE        RB5
	    #define     OPEN_LAMP     RB6   //LATC5
	    #define     CLOSE_LAMP    RB7   //LATC6
	                    
	    
	    #define     DSP_OFF        LATA4
	    #define     WR_CLK         LATA5
	    
	    #define     OPEN_KEY       RE0
	    #define     CLOSE_KEY      RE1
	    
	    #define     P2             LATA
	
	
	    #define  UP_KEY_LAMP        RE2     //not use
	    #define  DN_KEY_LAMP        RE2
	    #define  AUTO_LAMP          RE2
	    #define  MANUAL_LAMP        RE2
	    #define  HALL_LAMP_UP       RE2
	    #define  HALL_LAMP_DN       RE2
	#endif

#elif defined(CAN_IO)

	#if  defined(NEW_CAN_IO_32_BCD) || defined(NEW_CAN_IO_32_FLR_ONOFF) || defined(NEW_CAN_IO_HOST)
		#define  IO_00    	RC0
		#define  IO_01    	RC1
		#define  IO_02   	RC2
		#define  IO_03     	RC3
		#define  IO_04    	RC4
		#define  IO_05     	RC5
		#define  IO_06     	RC6
		#define  IO_07     	RC7
			
		#define  IO_08    	RF0
		#define  IO_09     	RF1
		#define  IO_10     	RF2
		#define  IO_11     	RF3
		#define  IO_12     	RF4
		#define  IO_13     	RF5
		#define  IO_14		RF6         
		#define  IO_15		RF7
		
		#define  IO_16   	RA0
		#define  IO_17     	RA1
		#define  IO_18     	RA2
		#define  IO_19    	RA3
		#define  IO_20      RA5
		#define  IO_21      RE0
		#define  IO_22      RE1
		#define  IO_23    	RE2
		
		#define  IO_24     	RE4
		#define  IO_25      RE5
		#define  IO_26     	RE6
		#define  IO_27      RE7
		#define  IO_28     	RG0
		#define  IO_29      RG1
		#define  IO_30      RG2
		#define  IO_31      RG3

		#define  IO_32     	RD0
		#define  IO_33      RD1
		#define  IO_34     	RD2
		#define  IO_35      RD3
		#define  IO_36     	RD4
		#define  IO_37      RD5
		#define  IO_38      RD6
		#define  IO_39      RD7

		#define  O_32     	LATD0
		#define  O_33      	LATD1
		#define  O_34     	LATD2
		#define  O_35      	LATD3
		#define  O_36     	LATD4
		#define  O_37      	LATD5
		#define  O_38      	LATD6
		#define  O_39      	LATD7

		#define	RUN_LED		LATB0

	#elif	defined(NEW_CAN_IO_8) || defined(NEW_CAN_O_8) || defined(POWER_CONTROL)
		#define  	IO_00         RD0
		#define  	IO_01         RD1
		#define  	IO_02         RD2
		#define  	IO_03         RD3
		#define  	IO_04         RD4
		#define  	IO_05         RD5
		#define  	IO_06         RD6
		#define  	IO_07         RD7
			
		#define  	IO_08         RC0
		#define  	IO_09         RC1
		#define  	IO_10         RC2
		#define  	IO_11         RC3
		#define  	IO_12         RC4
		#define  	IO_13         RC5
		#define  	IO_14         RC6
		#define  	IO_15         RC7

		#define		RUN_LED		  LATB0
	
	#elif  defined(OLD_CAN_IO_32_FLR_ONOFF)
		#define  	IO_00         RD0
		#define  	IO_01         RD1
		#define  	IO_02         RD2
		#define  	IO_03         RD3
		#define  	IO_04         RD4
		#define  	IO_05         RD5
		#define  	IO_06         RD6
		#define  	IO_07         RD7
								
		#define  	IO_08         RC0
		#define  	IO_09         RC1
		#define  	IO_10         RC2
		#define  	IO_11         RC3
		#define  	IO_12         RC4
		#define  	IO_13         RB0     
		#define  	IO_14         RB1

		#define  	IO_15         RB4		
		#define  	IO_16         RA0
		#define  	IO_17         RA1
		#define  	IO_18         RA2
		#define  	IO_19         RA3
		#define  	IO_20         RA4
		#define  	IO_21         RA5
		#define  	IO_22         RE0
		#define  	IO_23         RE1
		
		#define  	IO_24         RE2
		#define  	IO_25         RC5
		#define  	IO_26         RC6
		#define  	IO_27         RC7
		#define  	IO_28         RB6
		#define  	IO_29         RB7
		#define  	IO_30         RA7
		#define  	IO_31         RA6
		#define		RUN_LED		  LATB5
	#else
    	#error   "Type Not Define"
	#endif
#else
    #error   "Type Not Define"
#endif












//#define     VIP_TEMP   1

