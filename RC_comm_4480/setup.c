

#include    <pic18.h>

#include        "..\RC_system_com\memory_map.h"
#include        "setup.h"
#include        "fontout.h"
#include        "iodef.h"
#include        "keysort.h"


extern void    SetupDisplayES15(unsigned char mdata,unsigned char mode);


//information   local number,hibhpi,subdoor,Top flr,nc  
//volatile const unsigned char information[12]={LOCAL_NM,HIB_HPI,SUBDOOR_SEL,SENSOR_TYPE,BAGGAGE,COMPANY,H_K_C,CALL_OFFSET,SEARCH4,SEARCH5,NC_END};	/*this is the variable in FLASH where the old text resides*/
volatile const unsigned char information[12]={LOCAL_NM,0,0,SENSOR_TYPE,BAGGAGE,COMPANY,SETTING_VALUE,CALL_OFFSET,SEARCH4,SEARCH5,NC_END};	/*this is the variable in FLASH where the old text resides*/
unsigned char new_value[]={0,0,0,0,0,0,0,0,0,0,0};	                                /*unlike the old_text this is not a CONST -> stored in data RAM */
far unsigned char * source_ptr = (far unsigned char *)new_value;                    /*pointers to data*/
far unsigned char * dest_ptr = (far unsigned char *)information;
unsigned char size = 11;						



unsigned    char    SrcAddress;
unsigned    char    LocalNumber=0;
//unsigned    char    SubkDoor=0;
unsigned    char    offset_flr=0;
unsigned    char    ButtonType=0;
unsigned    char    CallOffset=0;

bit   ChangeSetup=0;
bit   UpKeyOn=0;
bit   UpKeyLong=0;
bit   UpKeyOneTouch=0;
bit   DnKeyOn=0;
bit   DnKeyLong=0;
bit   DnKeyOneTouch=0;
bit   CmpSetBit=0;
bit   MaskSetBit=0;
//bit   bCloseReady=0;                    

bit   bToggleOn=0;
bit   bCurOpenKey=0;
bit   bUserLamp4=0;

bit   bHibHpiSel=0;
bit   bMainSubDoor=0;


void    ReadFlashSetup(void)
{
    unsigned char i;

    for(i=0;i<size;i++){
        new_value[i]=information[i];
    }
}


void    LoadSetupValue(void)
{
	ReadFlashSetup();

    LocalNumber=information[0];

//    DspModel=information[1];
//    if(DspModel > 2)   DspModel = ES00;     

//    SubkDoor=information[2];
//    if(SubkDoor > 1)   SubkDoor = 0;     

    offset_flr=information[3];
    if(offset_flr > 3)   offset_flr = 0;     

    ButtonType=information[4];

    Company=information[5];


	bToggleOn=0;
    if(information[6] & 0x01)	bToggleOn=1;

	bCurOpenKey=0;
    if(information[6] & 0x02)	bCurOpenKey=1;

	bUserLamp4=0;
    if(information[6] & 0x04)	bUserLamp4=1;


	bHibHpiSel=0;
    if(information[6] & 0x08)	bHibHpiSel=1;

	bMainSubDoor=0;
    if(information[6] & 0x10)	bMainSubDoor=1;

	CallOffset=information[7];

}



/*
void    TopResetting(void)
{

    unsigned int i,j,k;

    for(i=0;i<MAX_ELEV;i++){
        k=(i * HOST_DATA_RECOD) + RCV_DATA;
        
        if(RcvBuf[(HOST_DATA_RECOD * i) + RCV_DATA] > TopFloor){
            for(j=0;j<size;j++){
                new_value[i]=information[i];
            }
            TopFloor=RcvBuf[(HOST_DATA_RECOD * i) + RCV_DATA];
            new_value[3]=TopFloor;
            flash_write(source_ptr,size,dest_ptr);
        }
    }

}
*/


/*
void    UpDnKeyCheck(void)
{
    
    if(UpKeyBit){
        if(UpButtonTime > 100){
            UpKeyLong=1;
            UpKeyOn=0;
        }
        else if(UpButtonTime > 0){
            UpKeyOn=1;
            UpKeyLong=0;
        }
        else{
            UpKeyOn=0;
            UpKeyLong=0;
        }
    }
    else{
        UpButtonTime=0;
        if(UpKeyOn){
            UpKeyOn=0;
            UpKeyOneTouch=1;
        }
        UpKeyLong=0;
    }

    if(DnKeyBit){
        if(DnButtonTime > 100){
            DnKeyLong=1;
            DnKeyOn=0;
        }
        else if(DnButtonTime > 0){
            DnKeyOn=1;
            DnKeyLong=0;
        }
        else{
            DnKeyOn=0;
            DnKeyLong=0;
        }
    }
    else{
        DnButtonTime=0;
        if(DnKeyOn){
            DnKeyOn=0;
            DnKeyOneTouch=1;
        }
        DnKeyLong=0;
    }
}
*/

/*
void  SetupDisplay(unsigned char ft,unsigned char sd)
{
   unsigned char i;

   if(FDsp){   
      for(i=0;i<=15;i++){
         DspBuf[i]=0x0;
      }   

      if(ft<0xff)    string_load_ram(0,ft);            
      if(sd<0xff)    EngDisplay(11,sd);    
           
      for(i=0;i<=15;i++){
         shiftDataBuf[i]=DspBuf[i];
      }
   }
   else{
      for(i=0;i<=15;i++){
         shiftDataBuf[i]=0x0;
      }   
   }                    
}
*/


void    Display(unsigned char i,unsigned char j,unsigned char k)
{

    if(MainTimer>5){
        MainTimer=0;
        FDsp=!FDsp;
#if defined(__DSP_DOT)
        SetupDisplay(i,j);
#else     
        SetupDisplayES15(i,k);
#endif     
    }         
}


void    CompanyWrite(void) //new
{
    flash_write(source_ptr,size,dest_ptr);
    asm("reset");
}



/*
void    SetupMode(void)
{

    unsigned char j,k,mode,topchar;

    mode=0;

    do{
        CLRWDT();

        #ifdef  __TYPE_ES15
            if(mode==1)         mode=2;
        #endif     

        switch(mode){
            case    0:
                j=28;   //l local number
                topchar=MAX_ELEV;
                break;
            case    1:
                j=24;   //H hib hpi
                topchar=2;
                break;
            case    2:  //s subdoor
                j=35;
                topchar=2;
                break;

            case    3:  //F offset floor
                j=22;
                topchar=3;
                break;
            case    4:  //B offset floor
                j=18;
                topchar=3;
                break;
   
        }        
        Display(new_value[mode],j,mode);
        UpDnKeyCheck();
        if(UpKeyOneTouch){
            mode=(mode+1)%5;
            UpKeyOneTouch=0;
        }
        else if(DnKeyOneTouch){
            k=new_value[mode];
            k=(k+1)%topchar;
            new_value[mode]=k;
            DnKeyOneTouch=0;
            ChangeSetup=1;    
       }
    }while(!UpKeyLong && !DnKeyLong);



//    if(ChangeSetup){
//		ChangeSetup=0;
//		CompanyWrite();
//    }

}
*/




/*
void SetupCheck(void)
{

    unsigned char i;

    i=1;

    ChangeSetup=0;
    UpButtonTime = 0;
    DnButtonTime = 0;


    shiftTime=0;
    while(shiftTime==0);


    do{
        UpDnKeyCheck();
        if(UpKeyLong){
            i=0;
            break;        
        }                
        CLRWDT();    
    }while(UpKeyBit);

    if(i==0){
        for(i=0;i<size;i++){
            new_value[i]=information[i];
        }

        do{
            i=0;
            Display(new_value[i],28,0);
            UpDnKeyCheck();
            CLRWDT();            
        }while(UpKeyLong || DnKeyLong);

        SetupMode();
    }    

    if(ChangeSetup){
		flash_write(source_ptr,size,dest_ptr);
        ChangeSetup=0;
    }

    LoadSetupValue();

}
*/

void	SetClearBit(unsigned char *TargetBuf,unsigned char value,unsigned char bitnm)
{
	unsigned char bitvalue;
	bitvalue=0x01;
	bitvalue=(bitvalue << bitnm);

	if( *TargetBuf & bitvalue){
		if(value==1){
			*TargetBuf=( *TargetBuf & ~bitvalue);	
			ChangeSetup=1;
		}				
	}
	else{
		if(value>1){
			*TargetBuf=( *TargetBuf | bitvalue);	
			ChangeSetup=1;
		}				
	}
}



void	SetClearByte(unsigned char *TargetBuf,unsigned char value)
{

	if(value > 0){
		value=(value - 1 );
		if(value != *TargetBuf){
			*TargetBuf=value;
			ChangeSetup=1;
		}
	}

}


void	CmdSetupChk(unsigned char id)
{		

    unsigned char	tmpb;
    unsigned int    IdPt;
   
	IdPt=IsBufferPt(id);

    if((RcvBuf[IdPt+DSP1] == '9') && (RcvBuf[IdPt+DSP2] == '1') && !(RcvBuf[IdPt+1] & S1_AUTO) && ((RcvBuf[IdPt+SL_mCarOpCl] & 0xc0)== 0xc0) ){
		tmpb= (RcvBuf[IdPt+SL_mCarOpCl] & 0x03);
		SetClearBit(&new_value[6],tmpb,0);

		tmpb= (RcvBuf[IdPt+SL_mCarOpCl] & 0x0c);
		tmpb=(tmpb >> 2);
		SetClearBit(&new_value[6],tmpb,1);

		tmpb= (RcvBuf[IdPt+SL_mCarOpCl] & 0x30);
		tmpb=(tmpb >> 4);
		SetClearBit(&new_value[6],tmpb,2);


		tmpb= (RcvBuf[IdPt+SL_mCarKey25] & 0x03);
		SetClearBit(&new_value[6],tmpb,3);

		tmpb= (RcvBuf[IdPt+SL_mCarKey25] & 0x0c);
		tmpb=(tmpb >> 2);
		SetClearBit(&new_value[6],tmpb,4);


		SetClearByte( &new_value[7],RcvBuf[IdPt+SL_mCarKey1]);
		SetClearByte( &new_value[4],RcvBuf[IdPt+SL_mCarKey9]);
		SetClearByte( &new_value[3],RcvBuf[IdPt+SL_mCarKey17]);
		SetClearByte( &new_value[0],(id+1));
	}
 }


