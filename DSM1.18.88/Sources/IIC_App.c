#include "includes.h"


void DelayUS(word xus)
{ 
	int i,j;
	
	for(j=0;j<xus;j++)
	  {
	    i=15;                  /////////1us
        while(i>0)i=i-1;		/*1us*/
    } 
}




void StartI2C()
{
	DIR_OUT;
  	SDA_1;
  	DelayUS(5);
  	SCL_1;
  	DelayUS(5);
  	SDA_0;
  	DelayUS(5);
  	SCL_0;
  	DelayUS(5);
}

void StopI2C(void)
{
  	DIR_OUT;
  	SDA_0;
  	DelayUS(5);
  	SCL_1;
  	DelayUS(5);
  	SDA_1;
  	DelayUS(5);
  	SCL_0;
  	DelayUS(5);
}

void WriteAByte(byte data)
{
  	word i;
  	
  	DIR_OUT;
  	for(i=0;i<8;i++)
  	{
    	SCL_0;
    	DelayUS(5);
    	if((data>>7) & 1)
    	    SDA_1;
    	else 
    	    SDA_0;
    	DelayUS(5);
    	SCL_1;
    	data<<=1;
    	DelayUS(5);
  	}
   	SCL_0;
  	DelayUS(5);

}



word ReadAByte(void)
{
  	word i,bit=0,data=0;

  	DIR_IN;                                                     
    SCL_0;
  	DelayUS(5);
  	for(i=0;i<8;i++)
  	{
    	SCL_1;
    	DelayUS(5);
    	if(READ_SDA)bit=1;
	    else bit=0;
	    data=(data<<1)|bit;
	    SCL_0;
    	DelayUS(5);
	}
  	return(data);
}

word ReceiveI2CAck(void)
{
  	word i=1000;

   	SDA_1;
  	DIR_IN;
  	SCL_1;
  	DelayUS(5);
  	while(READ_SDA)
  	{
    	if(!(--i))
    	{
  			SCL_0;
    		return(FALSE);
    	}
  	}
  	SCL_0;
  	DelayUS(5);
  	return(TRUE);
}
void SendI2CAck(void)
{
  	SCL_0;
  	DelayUS(5);
  	DIR_OUT;
  	SDA_0;
  	SCL_1;
  	DelayUS(5);
  	SCL_0;
  	DelayUS(5);
}
void SendI2CNAck(void)
{
  	SCL_0;
  	DelayUS(5);
  	DIR_OUT;
  	SDA_1;
  	SCL_1;
  	DelayUS(5);
  	SCL_0;
  	DelayUS(5);
}

void WriteBytes(word *buffer,word n)
{
	word i;

    for(i=0;i<n;i++)
    {
        WriteAByte(*buffer++);
        ReceiveI2CAck();
    }
    StopI2C();
}


void SetCurrentAddress(word slave,word address)
{
	StartI2C();
	WriteAByte(slave);
	ReceiveI2CAck();

	WriteAByte(address);
	ReceiveI2CAck();
}

void ReadBytes(word slave, word *buffer, word n)
{
	word i;
	
	StartI2C();
	WriteAByte(slave);
	ReceiveI2CAck();
	for(i=0;i<n;i++)
	{
		*buffer=ReadAByte();
		buffer++;
		if(i==(n-1))SendI2CNAck();
		else SendI2CAck();
	}
	StopI2C();
}


void InitRtc()
{    
    word flag = 1;
    DDRJ_DDRJ7=1;   //SCL output
    
    SetCurrentAddress(0x00A2,0x0000); 	//Config  register 00H
    WriteAByte(0x0000);                 //Write data 0 to register 00H; Normal Mode
    flag = ReceiveI2CAck();
    StopI2C();
    
    SetCurrentAddress(0x00A2,0x0001); 	//Config  register 01H
    WriteAByte(0x0015);                 //Write data 15h to register 01H; 
    flag = ReceiveI2CAck();     				//INT pulses active, Timer Flag active and Timer interrupt Enable
    StopI2C();
    
    
    SetCurrentAddress(0x00A2,0x000E); 	//Config  register 0EH
    WriteAByte(0x0082);                 //Write data 82h to register 0EH;
    flag = ReceiveI2CAck(); 						//Timer Enable; Choose INT pulses as 1Hz	
    StopI2C();
    
    SetCurrentAddress(0x00A2,0x000F); 	//Config  register 0FH
    WriteAByte(0x0001);                 //Write data 1 to register 0FH; The count value is 1
    flag = ReceiveI2CAck();
    StopI2C();
    
    /*
    RtcClock.Year=10;
    RtcClock.Month=11;
    RtcClock.Day=30;
    RtcClock.Hour=11;
    RtcClock.Minute=19;
    RtcClock.Second=0; 
    WriteRtc();
    */    
    
    
    //RTC_INT  PH6    
    PIEH_PIEH6=0;   //disable intrrupt
    PPSH_PPSH6=0;   //falling edge
    PIFH_PIFH6 = 1;                          //Clear flag 
    PIEH_PIEH6 = 1;                      // Enable interrupt 
 /*   */
}

void ReadRtc(void)
{
    
    word i,*ptr=&RtcClock;
    DisableInterrupts;       //disable interrupt

    SetCurrentAddress(0x00a2,0x0002);         //Read Clock Register
    ReadBytes(0x00a3,&RtcClock.Second,7);

    for(i=0;i<7;i++)
    {
		switch(i)
		{
		case 0:
		case 1:
			*ptr=((*ptr)&0x000f) + (((*ptr)>>4) & 0x7)*10;
			break;
		case 2:
		case 3:
			*ptr=((*ptr)&0x000f) + (((*ptr)>>4) & 0x3)*10;
			break;
		case 4:
			*ptr=((*ptr)&0x0007);
			break;
		case 5:
			*ptr=((*ptr)&0x000f) + (((*ptr)>>4) & 0x1)*10;
			break;
		case 6:
		  	*ptr=((*ptr)&0x000f) + (((*ptr)>>4) & 0xF)*10;
			break;
		}        
        ptr++;
    } 
    
    if(RtcClock.Year>99)RtcClock.Year=0;
	if((RtcClock.Month>12)||(RtcClock.Month==0))RtcClock.Month=1;
	if(RtcClock.Weekday>6)RtcClock.Weekday=0;
	if((RtcClock.Day>31)||(RtcClock.Day==0))RtcClock.Day=1;
	if(RtcClock.Hour>23)RtcClock.Hour=0;
	if(RtcClock.Minute>59)RtcClock.Minute=0;
	if(RtcClock.Second>59)RtcClock.Second=0;
    EnableInterrupts; 
}

void WriteRtc()
{
    word i,*ptr=&RtcClock.Second;
    DisableInterrupts;
    for(i=0;i<7;i++)
    {
        *ptr=(((*ptr)/10)<<4) + (*ptr)%10;
        ptr++;
    }

    SetCurrentAddress(0x00A2,0x0002);   //Write Clock Register
    WriteBytes(&RtcClock.Second,7);
    EnableInterrupts;

}

void CacuSecondTime(dword *TimeSec)
{
	dword DayDelay=0;
	dword TimeDelay=0;

	word Flagleapyear=0;	//flag leapyear 
	word Year=1970;
	word Month=1;
	word Day=1;
	word Hour=0;
	word Minute=0;
	word Second=0;		
	while (1)
    {
		if(Year<(RtcClock.Year+2000))
		{
			if (Year % 4 == 0 &&Year % 100 != 0 ||Year % 400 == 0)  //leap year
	            DayDelay += 366;
	        else
	            DayDelay += 365;
			Year++;
		}
		else
		{
			if(Month<RtcClock.Month)
			{
				if (Month == 1 || Month == 3 || Month == 5 || Month == 7 || Month == 8 || Month == 10 || Month == 12)
                {
					DayDelay+=31;
                }
                else if (Month == 2)
                {
					if (Year % 4 == 0 &&Year % 100 != 0 ||Year % 400 == 0)  //leap year
			            DayDelay += 29;
			        else
			            DayDelay += 28;
                }
				else
				{
					DayDelay+=30;
				}

				Month++;
			}
	        else
			{
		    	TimeDelay = (((DayDelay +RtcClock.Day-1)* 24 + RtcClock.Hour) * 60 + RtcClock.Minute) * 60 + RtcClock.Second;
                break;
			}
		}
    }

    *TimeSec=TimeDelay;
}

void CacuClockTime(dword TimeSec)
{
	
    dword YearDelay = 0;
    dword MonthDelay = 0;
    dword DayDelay = 24.0 * 60.0 * 60.0;
    dword HourDelay = 60.0 * 60.0;
    word MiniteDelay = 60;

	RtcClock.Year=1970;
	RtcClock.Month=1;
	RtcClock.Day=1;
	RtcClock.Hour=0;
	RtcClock.Minute=0;
	RtcClock.Second=0;	


	while (1)
    {
        if (RtcClock.Year % 4 == 0 && RtcClock.Year % 100 != 0 ||RtcClock.Year % 400 == 0)  //leap year
            YearDelay=366.0*24.0*60.0*60.0;
        else
            YearDelay=365.0*24.0*60.0*60.0;
        if (TimeSec >=YearDelay)
        {
            RtcClock.Year++;
			TimeSec-=YearDelay;
        }
        else
        {
	        if (RtcClock.Month == 1 || RtcClock.Month == 3 || RtcClock.Month == 5 || RtcClock.Month == 7 || RtcClock.Month == 8 || RtcClock.Month == 10 || RtcClock.Month == 12)	       
	           MonthDelay=31.0*24.0*60.0*60.0;	       
	        else if (RtcClock.Month == 2)
	        {
	            if (RtcClock.Year % 4 == 0 &&RtcClock.Year % 100 != 0 ||RtcClock.Year % 400 == 0)  //leap year	           
	                MonthDelay=29.0*24.0*60.0*60.0;	           
	            else	         
 					MonthDelay=28.0*24.0*60.0*60.0;	          
	        }
	        else	       
	            MonthDelay=30.0*24.0*60.0*60.0;
	        
	        if(TimeSec>=MonthDelay)
	        {
				RtcClock.Month++;
				TimeSec-=MonthDelay;
	        }	
			else
			{
				if(TimeSec>=DayDelay)
				{
					RtcClock.Day++;
					TimeSec-=DayDelay;
				}
				else
				{
					if(TimeSec>=HourDelay)
					{
						RtcClock.Hour++;
						TimeSec-=HourDelay;
					}
					else
					{
						if(TimeSec>=MiniteDelay)
						{
							RtcClock.Minute++;
							TimeSec-=MiniteDelay;
						}
						else
						{
							RtcClock.Second=TimeSec;
							break;
						}
					}
				}

			}	        
                   
        }

    }
	RtcClock.Year %=100;
}

__interrupt  void RTC_ISR(void)
{ 
    DisableInterrupts;
    PIFH_PIFH6=1;           //clear the flag 
    if(SysTime>=0xFFFF)
        SysTime=0;
    else  
        SysTime++;    
    BatTimerCounter++; 
    if(3==EEP_MODE && 0!=eepromDat.CRP1.ParTimeBro && 0==SysTime%eepromDat.CRP1.ParTimeBro)
       TimeBroState=1;     
     
    EnableInterrupts
}