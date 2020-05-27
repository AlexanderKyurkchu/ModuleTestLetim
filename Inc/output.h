/**
  ******************************************************************************
  * @file          : output.h
  * @brief         : Header for output.c file.
  * @author        : Kyurkchu  A.               
  ******************************************************************************
  * 
  *       
  ******************************************************************************
  */
	
#ifndef _OUTPUT_H
#define _OUTPUT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "port.h"
#include "input.h"


 extern  _Floatdata OUTPUT_VDDA_DAC;
 extern  _Floatdata OUTPUT_VDDA_TRANS;
 extern  _Floatdata OUTPUT_VDDA_PIX;
 extern  _Floatdata OUTPUT_VMMerge_DAC;
 extern  _Floatdata OUTPUT_PRM5;
 extern  _Floatdata OUTPUT_PRM6;
 extern  _Floatdata OUTPUT_PRM7;
 extern  _Floatdata OUTPUT_PRM8;
 extern _Floatdata OUTPUT_CURRENT_ALGORITHM;
 
 extern SHORT  InputHoldingBuf[REG_HOLDING_NREGS];
 
 void modbus_data_output_process();
 
 
#ifdef __cplusplus
}
#endif

#endif

