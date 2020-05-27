/**
  ******************************************************************************
  * @file           : output.c
  * @brief          : This file provides code for the write output data
  * @author         : Kyurkchu  A.                       
  ******************************************************************************
  * 
  *       
  ******************************************************************************
  */
	
/* Includes ------------------------------------------------------------------*/
#include "output.h"
#include "input.h"

 _Floatdata OUTPUT_VDDA_DAC;
 _Floatdata OUTPUT_VDDA_TRANS;
 _Floatdata OUTPUT_VDDA_PIX;
 _Floatdata OUTPUT_VMMerge_DAC;
 _Floatdata OUTPUT_PRM5;
 _Floatdata OUTPUT_PRM6;
 _Floatdata OUTPUT_PRM7;
 _Floatdata OUTPUT_PRM8;
 
 
 _Floatdata OUTPUT_CURRENT_ALGORITHM;
 

  /**
  * @brief  modbus data read process from input holding registers 
  * @retval void
  */
 
 void modbus_data_output_process()
	 
  {
		
	/*#############  section  for float data ######## */	
		
    OUTPUT_VDDA_DAC.data[0] = InputHoldingBuf[0];
    OUTPUT_VDDA_DAC.data[1] = InputHoldingBuf[1];
	 
	  OUTPUT_VDDA_TRANS.data[0] = InputHoldingBuf[2];
    OUTPUT_VDDA_TRANS.data[1] = InputHoldingBuf[3];
	 
	  OUTPUT_VDDA_PIX.data[0] = InputHoldingBuf[4];
    OUTPUT_VDDA_PIX.data[1] = InputHoldingBuf[5];
	 
	  OUTPUT_VMMerge_DAC.data[0] = InputHoldingBuf[6];
    OUTPUT_VMMerge_DAC.data[1] = InputHoldingBuf[7];
	 
	 // OUTPUT_PRM5.data[0] = InputHoldingBuf[8];
   // OUTPUT_PRM5.data[1] = InputHoldingBuf[9];
	 
	 //	OUTPUT_PRM6.data[0] = InputHoldingBuf[10];
   // OUTPUT_PRM6.data[1] = InputHoldingBuf[11];
	 
	 //	OUTPUT_PRM7.data[0] = InputHoldingBuf[12];
   // OUTPUT_PRM7.data[1] = InputHoldingBuf[13];
	 
	 //	OUTPUT_PRM8.data[0] = InputHoldingBuf[14];
   // OUTPUT_PRM8.data[1] = InputHoldingBuf[15]; 
		
		
	/*#############  section  for algorithms ######## */		
		
		OUTPUT_CURRENT_ALGORITHM.data[0] = InputHoldingBuf[30];
		OUTPUT_CURRENT_ALGORITHM.data[1] = InputHoldingBuf[31];
		
	
			
 }