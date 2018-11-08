#include "includes.h"

void TimerInit(void)       //time out period =(PITMTLD+1)*(PITLD+1)/fbus     10ms
{ 
  /* Enable PIT 0 to provide 100us trigger to ATDs */
    PITCFLMT = 0x80;                 // Enable PIT
    PITCE = 0x01;                    // Enable channel 0
    PITMUX = 0x00;                   // Use microtimer 0 for all channels
    PITINTE = 0x00;                  // Disable interrupts    
    PITMTLD0 = 255;                   // Microtimer count     
    PITLD0 = 59;                     // Channel 0 counter ,1ms 
   // PITLD0 = 299;                     // Channel 0 counter ,5ms   
    
    
    //定时器初始化，4 ms        
    //PITLD0=64000;             //down counter load value    //4ms     
    //PITCFLMT =0x80;          //enable PIT
    //PITTF_PTF0 = 1;          //Reset interrupt request flag
    //PITINTE_PINTE0= 1;       //Enable interrupt 
            
}





