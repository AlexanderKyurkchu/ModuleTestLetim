  /**
	******************************************************************************
  * \file   : adc_input.c
  * \brief  : this file provides code for the adc input data
  * \author : Kyurkchu  A.      
  ******************************************************************************
  * @attention
  *       
  ******************************************************************************
  */
	/* Includes ------------------------------------------------------------------*/
#include "adc_input.h"
	
 float adc_average_value1;
 float adc_average_value2;
 float adc_average_value3;


 float adc_moving_average_value1;
 float adc_moving_average_value2;
 float adc_moving_average_value3;

 volatile  short  ADC_11;
 volatile  short  ADC_12;
 volatile  short  ADC_13;
 
 short  buf_11[128];
 short  buf_12[128];
 short  buf_13[128];

 FILTER_REG filter_reg1;
 FILTER_REG filter_reg2;
 FILTER_REG filter_reg3;

 int32_t filter_value11; 
 int32_t filter_value12; 
 int32_t filter_value13; 

 uint8_t flag_adc_dma;

  /**
  * @brief  ADC average calculation
  * @retval float
  */
 
float InputADC_ValueTo_Average_Float(short *InputArray , uint16_t Size)
 {
	  int32_t int_value ;
	  float float_value ;
    int32_t tempValue;
	  int_value = 0;
	 
	  for(int i = 0; i < Size; i++ )
	      {
	        int_value += InputArray[i]&(0x0000FFFF);	
		  	}
			
			tempValue = (int32_t)(int_value/Size);
			
	    float_value  = (float)((tempValue *3.3)/65535);
			
		return float_value;
 }
 
 
  /**
  * @brief  ADC filter moving average
  * @retval int32_t
  */
 
int32_t filter_moving_average(short  ADC_val, short* buf, FILTER_REG* filter_reg)
 {
     if (filter_reg->Reg.Flag){
        filter_reg->Reg.Filter_sum-=buf[filter_reg->Reg.Index];
        filter_reg->Reg.Filter_sum+=ADC_val;
        buf[filter_reg->Reg.Index]=ADC_val;
        if (filter_reg->Reg.Index>=COUNT_FILTER-1){
            filter_reg->Reg.Index=0;
        }
        else{
            filter_reg->Reg.Index++;
        }
    }
    else{
        filter_reg->Reg.Filter_sum+=ADC_val;
        buf[filter_reg->Reg.Index]=ADC_val;
        if (filter_reg->Reg.Index>=COUNT_FILTER-1){
            filter_reg->Reg.Index=0;
            filter_reg->Reg.Flag=1;
        }
        else{
            filter_reg->Reg.Index++;
        }
    }
    return (int32_t)(filter_reg->Reg.Filter_sum/COUNT_FILTER);

 }
 
  /**
  * @brief  convert average value to float value
  * @retval float
  */
 
 float moving_average_value_to_float(unsigned short int Average_value)
 {
   float float_value ;
	 
	 float_value  = (float)(((unsigned short int)Average_value *3.3)/65535);
	 
	 return float_value ;
 }
 