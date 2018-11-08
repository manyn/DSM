#include "includes.h"

void PLL_Init (void)       //fpll=30,fbus=15
{  
    CLKSEL = 0x00;                                              /* Meaning for CLKSEL:                                  */
                                                                /* Bit 7: PLLSEL = 0 OSCCLK selected as current clk src */
                                                                /* Bit 6: PSTP   = 0 Disable Psuedo Stop Mode           */
                                                                /* Bit 5: SYSWAI = 0 In wait mode system clocks stop.   */
                                                                /* Bit 4: ROAWAI = 0 Osc ampl not reduced in wait mode  */
                                                                /* Bit 3: PLLWAI = 0 Do not turn off PLL in wait mode   */
                                                                /* Bit 2: CWAI   = 0 Do not stop the core in wait mode  */
                                                                /* Bit 1: RTIWAI = 0 Do not stop the RTI in wait mode   */
                                                                /* Bit 0: COPWAI = 0 Do not stop the COP in wait mode   */
                                                                

    PLLCTL &= 0xBF; //LH,PLLON=0,close pll  
 
    
    SYNR   = 14              |                                 /* Set multiplier of 4                                 */
             SYNR_VCOFRQ0_MASK ;                                /* VCORANGE: 48MHz < Fvco < 80MHz                      */ 
                 
                          
    REFDV  = 15               |                                /* Set divider of 1                                     */
             REFDV_REFFRQ1_MASK;                                /* Set REFFRQ = 2: 6MHz  < Fref(osc) < 12MHz             */     
        

    
    
    POSTDIV=0x00;   //LH
    
    PLLCTL = 0xC0;                                              /* Meaning for PLLCTL:
                                                                /* Bit 7: CME   = 1; Clock monitor enabled              */
                                                                /* Bit 6: PLLON = 1; PLL On bit                         */
                                                                /* Bit 5: AUTO  = 0; No auto. control of PLL bandwidth  */
                                                                /* Bit 4: ACQ   = 1; High bandwidth filter acquisition  */
                                                                /* Bit 3:            Reserved                           */
                                                                /* Bit 2: PRE   = 0; RTI stops during Pseudo Stop Mode  */
                                                                /* Bit 1: PCE   = 0; COP diabled in Pseudo STOP mode    */
                                                                /* Bit 0: SCME  = 1; Use Self Clock mode if OSC fails   */                                                  
  
    while((CRGFLG & 0x08) == 0) { 	                            /* Wait for PLLCLK to stabilize.                        */
        ;				                                        /* If the PLL never stabilizes, this will hang forever  */
    }  
  
    CLKSEL |= CLKSEL_PLLSEL_MASK;                               /* Switch to PLL clock                                  */
}



void  GPIO_Init(void)
{
    //define Port A B C D E F J K L M R AD0 AD1 as input  default 
    
    DDRA=0;
    DDRB=0;
    DDRC=0;
    DDRD=0;
    DDRE=0;
    DDRF=0;
    DDRJ=0;
    DDRK=0;
    DDRL=0;
    DDRM=0;    
    DDRR=0;
    DDR0AD0=0;
    DDR0AD1=0;
    DDR1AD0=0;
    DDR1AD1=0;

    //reduice drive of Port k,e,d,c,b,a
    RDRIV=0xFF;
    //reduice drive of Port h
    RDRH=0xFF;
    //reduice drive of Port j
    RDRJ=0xFF;
    //reduice drive of Port AD0
    RDR0AD0=0xFF;
    //reduice drive of Port AD0
    RDR1AD0=0xFF;     
    //reduice drive of Port AD1
    RDR0AD1=0xFF;
    //reduice drive of Port AD1
    RDR1AD1=0xFF;
    //reduice drive of Port R
    RDRR=0xFF;
    //reduice drive of Port L
    RDRL=0xFF;
    //reduice drive of Port F
    RDRF=0xFF;
    
}