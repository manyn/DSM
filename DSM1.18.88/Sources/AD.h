#ifndef _AD_H
#define _AD_H

  /* Including shared modules, which are used in the whole project */
  
#pragma CODE_SEG DEFAULT

void AD_Init(void);

float ReadVoltage();    //voltage
float ReadCurrent();    //Current

float ReadInnerPre();
float ReadannPre();

#pragma CODE_SEG DEFAULT

#endif