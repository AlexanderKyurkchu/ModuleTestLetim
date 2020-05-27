 /**
 ******************************************************************************
  * @file          : adc_input.h
  * @brief         : Header for  adc_input.c
  * @author        : Kyurkchu  A.      
  ******************************************************************************
  * 
  *       
  ******************************************************************************
  */
	
#ifndef _ADC_INPUT_H
#define _ADC_INPUT_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "adc_input.h"

 
//structur of filter moving average

typedef union
{
    unsigned int Val;
    struct
    {
        unsigned Flag:1;        // buffer fill flag
        unsigned Index:8;       // index of value buffer
        unsigned Filter_sum:23; //
    } Reg;
} FILTER_REG;


extern float adc_average_value1;
extern float adc_average_value2;
extern float adc_average_value3;


extern float adc_moving_average_value1;
extern float adc_moving_average_value2;
extern float adc_moving_average_value3;

extern  volatile short  ADC_11;
extern  volatile short  ADC_12;
extern  volatile short  ADC_13;

extern  short  buf_11[128];
extern  short  buf_12[128];
extern  short  buf_13[128];

extern FILTER_REG filter_reg1;
extern FILTER_REG filter_reg2;
extern FILTER_REG filter_reg3;


extern int32_t filter_value11; 
extern int32_t filter_value12; 
extern int32_t filter_value13; 

extern uint8_t flag_adc_dma;


#define COUNT_FILTER    128
 

// ADC filter moving average
int32_t filter_moving_average(short  ADC_val, short* buf, FILTER_REG* filter_reg);

// moving average value to flaot value 
float  moving_average_value_to_float(unsigned short int Average_value);


// ADC average calculation
float  InputADC_ValueTo_Average_Float(short *InputArray, uint16_t Size);


#ifdef __cplusplus
}
#endif

#endif
