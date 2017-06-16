

#include    <pic18.h>



#include        "RC_system_com\memory_map.h"
#include        "RC_comm_4480\iodef.h"
#include        "RC_comm_4480\fontout.h"
#include        "RC_comm_4480\comdata.h"
#include        "RC_comm_4480\keysort.h"
#include        "RC_comm_4480\setup.h"
#include        "RC_comm_4480\esdots.h"



#define		RUN_LED		  LATC7


#define  	IO_00         LATD0
#define  	IO_01         LATD1
#define  	IO_02         LATD2
#define  	IO_03         LATD3
#define  	IO_04         LATB0
#define  	IO_05         LATB1
#define  	IO_06         LATB4
#define  	IO_07         LATB5


	
#define  	IO_08         RA0
#define  	IO_09         RA1
#define  	IO_10         RA2
#define  	IO_11         RA3
#define  	IO_12         RE1
#define  	IO_13         RE2
#define  	IO_14         RC0
#define  	IO_15         RC2



#define	CHARTER_TIME	20	// 20 msec

unsigned char	charter_IO_08;
unsigned char	charter_IO_09;
unsigned char	charter_IO_10;
unsigned char	charter_IO_11;
unsigned char	charter_IO_12;
unsigned char	charter_IO_13;
unsigned char	charter_IO_14;
unsigned char	charter_IO_15;


unsigned 	char	IN_BUF=0;
unsigned	int 	Flr_OnOff_TxTmer;
unsigned 	int		Ext_IO_8_Sec=0;


bit	Bef_IO_08=0;	
bit	Cur_IO_08=0;	
bit	Bef_IO_09=0;	
bit	Cur_IO_09=0;	
bit	Bef_IO_10=0;	
bit	Cur_IO_10=0;	
bit	Bef_IO_11=0;	
bit	Cur_IO_11=0;	
bit	Bef_IO_12=0;	
bit	Cur_IO_12=0;	
bit	Bef_IO_13=0;	
bit	Cur_IO_13=0;	
bit	Bef_IO_14=0;	
bit	Cur_IO_14=0;	
bit	Bef_IO_15=0;	
bit	Cur_IO_15=0;	





void  MyAdrRead_Ext_IO_8(void)
{
   	MyAddress=0;
   	MyAddress=0x8b;
	CallMeAdr=MyAddress;
	Flr_OnOff_TxTmer=0;
}





void    Ext_IO_8_Init(void)
{

	TRISD0=0x00;	// Y0
	TRISD1=0x00;    // Y1
	TRISD2=0x00;    // Y2
	TRISD3=0x00;    // Y3
	TRISB0=0x00;    // Y4
	TRISB1=0x00;    // Y5
	TRISB4=0x00;    // Y6
	TRISB5=0x00;    // Y7

	TRISC7=0x00;    // Runled


	TRISA0=0x1;		// X0
	TRISA1=0x1;    	// X1
	TRISA2=0x1;    	// X2
	TRISA3=0x1;    	// X3
	TRISE1=0x1;    	// X4
	TRISE2=0x1;    	// X5
	TRISC0=0x1;    	// X6
	TRISC2=0x1;    	// X7

	

	LATA0=1;
	LATA1=1;
	LATA2=1;
	LATA3=1;
	LATE1=1;
	LATE2=1;
	LATC0=1;
	LATC1=1;





/*
  	TRISE2=1;	    // port E data direction...(output)
  	TRISE1=1;
  	TRISE0=1;

  	TRISA=0xff;
  	TRISB=0xff;
  	TRISC=0xff;
  	TRISD=0x00;
	

	PORTA=0xff;
	PORTB=0xff;
	PORTC=0xff;
	PORTD=0x00;
	
	PORTE=0xff;

	TRISB0=0;		//RUN LED 
	TRISB3=1;   	//can rx 
	TRISB2=0;   	//can tx
	
	LATB0=0;		//RUN LED
	LATB3=1;		//can rx
	LATB2=0;		//can tx
*/

	
	MyAdrRead_Ext_IO_8();
}



void	Input_Chk(void)
{
    if(Bef_IO_08 == IO_08){
        if(charter_IO_08 > CHARTER_TIME ){
            Cur_IO_08 = IO_08;
        }
    }
    else{
        Bef_IO_08 = IO_08;
        charter_IO_08 = 0;
    }    


    if(Bef_IO_09 == IO_09){
        if(charter_IO_09 > CHARTER_TIME ){
            Cur_IO_09 = IO_09;
        }
    }
    else{
        Bef_IO_09 = IO_09;
        charter_IO_09 = 0;
    }    



    if(Bef_IO_10 == IO_10){
        if(charter_IO_10 > CHARTER_TIME ){
            Cur_IO_10 = IO_10;
        }
    }
    else{
        Bef_IO_10 = IO_10;
        charter_IO_10 = 0;
    }    



    if(Bef_IO_11 == IO_11){
        if(charter_IO_11 > CHARTER_TIME ){
            Cur_IO_11 = IO_11;
        }
    }
    else{
        Bef_IO_11 = IO_11;
        charter_IO_11 = 0;
    }    


    if(Bef_IO_12 == IO_12){
        if(charter_IO_12 > CHARTER_TIME ){
            Cur_IO_12 = IO_12;
        }
    }
    else{
        Bef_IO_12 = IO_12;
        charter_IO_12 = 0;
    }    


    if(Bef_IO_13 == IO_13){
        if(charter_IO_13 > CHARTER_TIME ){
            Cur_IO_13 = IO_13;
        }
    }
    else{
        Bef_IO_13 = IO_13;
        charter_IO_13 = 0;
    }    



    if(Bef_IO_14 == IO_14){
        if(charter_IO_14 > CHARTER_TIME ){
            Cur_IO_14 = IO_14;
        }
    }
    else{
        Bef_IO_14 = IO_14;
        charter_IO_14 = 0;
    }    



    if(Bef_IO_15 == IO_15){
        if(charter_IO_15 > CHARTER_TIME ){
            Cur_IO_15 = IO_15;
        }
    }
    else{
        Bef_IO_15 = IO_15;
        charter_IO_15 = 0;
    }    


	IN_BUF=0;
	if(Cur_IO_08)	IN_BUF = (IN_BUF | 0x01);	
	if(Cur_IO_09)	IN_BUF = (IN_BUF | 0x02);	
	if(Cur_IO_10)	IN_BUF = (IN_BUF | 0x04);	
	if(Cur_IO_11)	IN_BUF = (IN_BUF | 0x08);	
	if(Cur_IO_12)	IN_BUF = (IN_BUF | 0x10);	
	if(Cur_IO_13)	IN_BUF = (IN_BUF | 0x20);	
	if(Cur_IO_14)	IN_BUF = (IN_BUF | 0x40);	
	if(Cur_IO_15)	IN_BUF = (IN_BUF | 0x80);	

}



unsigned int  Ext_IO_8_Func(void)
{
    unsigned 	int    IdPt;


    IdPt=(LocalNumber * HOST_DATA_RECOD) + RCV_DATA;


	if(RcvBuf[IdPt + SL_S5_STATE_37] & 0x01)	IO_00=1;
	else										IO_00=0;

	if(RcvBuf[IdPt + SL_S5_STATE_37] & 0x02)	IO_01=1;
	else										IO_01=0;
	
	if(RcvBuf[IdPt + SL_S5_STATE_37] & 0x04)	IO_02=1;
	else										IO_02=0;

	if(RcvBuf[IdPt + SL_S5_STATE_37] & 0x08)	IO_03=1;
	else										IO_03=0;

	if(RcvBuf[IdPt + SL_S5_STATE_37] & 0x10)	IO_04=1;
	else										IO_04=0;

	if(RcvBuf[IdPt + SL_S5_STATE_37] & 0x20)	IO_05=1;
	else										IO_05=0;
	
	if(RcvBuf[IdPt + SL_S5_STATE_37] & 0x40)	IO_06=1;
	else										IO_06=0;

	if(RcvBuf[IdPt + SL_S5_STATE_37] & 0x80)	IO_07=1;
	else										IO_07=0;



	Input_Chk();

	if(RcvBuf[IdPt + SL_mEXT_IN0]  != IN_BUF){
		Flr_OnOff_TxTmer=1600;
	}


	if(Flr_OnOff_TxTmer >=1500){
        if(CanTxAct){
			CanDatacnt1=8;
            CanCmd=IN_BUF;
            CanKeyValue[1]=0x0;
            CanKeyValue[2]=0x0;
            CanKeyValue[3]=0x0;
            CanKeyValue[4]=0x0;
            CanKeyValue[5]=0x0;
            CanKeyValue[6]=0x0;
			CanKeyValue[7]=0x0;
            Tx1ConfirmCnt=1;

            CanTx1();

			if(Tx1ConfirmCnt==0){
				Flr_OnOff_TxTmer = 0;
			}	
		}		
	}

	return(1);
}


void	Ext_IO_8_Timer_1msec(void)
{

	if(Flr_OnOff_TxTmer < 5000)	Flr_OnOff_TxTmer++;

    if(charter_IO_08 < 250)	charter_IO_08++;
    if(charter_IO_09 < 250)	charter_IO_09++;
    if(charter_IO_10 < 250)	charter_IO_10++;
    if(charter_IO_11 < 250)	charter_IO_11++;
    if(charter_IO_12 < 250)	charter_IO_12++;
    if(charter_IO_13 < 250)	charter_IO_13++;
    if(charter_IO_14 < 250)	charter_IO_14++;
    if(charter_IO_15 < 250)	charter_IO_15++;

	Ext_IO_8_Sec++;
	if(Ext_IO_8_Sec > 1000){
		Ext_IO_8_Sec=0;
		RUN_LED = !RUN_LED;
	}

}


