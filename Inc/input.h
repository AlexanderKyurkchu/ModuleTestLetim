/**
  ******************************************************************************
  * @file          : input.h
  * @brief         : Header for  input.c
  * @author        : Kyurkchu  A.      
  ******************************************************************************
  * 
  *       
  ******************************************************************************
  */


#ifndef _INPUT_H
#define _INPUT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "port.h"


//**********for discrete INPUTS***********
#define INPUT_1    GPIO_PIN_4        
#define INPUT_2    GPIO_PIN_5  
#define INPUT_3
#define INPUT_4
#define INPUT_5
#define INPUT_6
#define INPUT_7
#define INPUT_8

#define INPUT_PORT  GPIOG

uint8_t  read_input_1(void);
uint8_t  read_input_2(void);
uint8_t  read_input_3(void);
uint8_t  read_input_4(void);
uint8_t  read_input_5(void);
uint8_t  read_input_6(void);
uint8_t  read_input_7(void);
uint8_t  read_input_8(void);
 
 
//************ for ModBus RTU************ 
#define REG_INPUT_START 20  // not used
#define REG_INPUT_NREGS 10  // not used

#define REG_HOLDING_START 130
#define REG_HOLDING_NREGS 32


#define REG_COILS_START   200
#define REG_COILS_SIZE    16


#define INPUT_STATUS_START 100
#define INPUT_INPUT_NREGS  8


typedef union 
  {
	  float  fl;
	  SHORT data[2];
		uint8_t int_data;
  }_Floatdata;

extern 	_Floatdata  VDDD_U;
extern  _Floatdata  VDDD_I;
	 
	
extern  _Floatdata  VDDA_U;
extern  _Floatdata  VDDA_I;
	
extern _Floatdata  VDDA_U;
extern  _Floatdata  VDDA_I;
	
extern  _Floatdata  V2_5VDD_U;
extern  _Floatdata  V2_5VDD_I;
	
extern  _Floatdata  VDDA_DAC_U;
extern  _Floatdata  VDDA_DAC_I;
	
extern  _Floatdata  VDD_PIX_U;
extern  _Floatdata  VDD_PIX_I;	

extern  _Floatdata  VDDA_TRANS_U;
extern  _Floatdata  VDDA_TRANS_I;	
	
extern  _Floatdata  VCCD_U;
extern  _Floatdata  VCCD_I;

extern  _Floatdata  VDDU_U;
extern  _Floatdata  VDDU_I;

extern _Floatdata  VMMerge_DAC_U;
extern _Floatdata  VMMerge_DAC_I;

extern SHORT    OutRegHoldingBuf[REG_HOLDING_NREGS];

void modbus_data_input_process(void);


	 
#ifdef __cplusplus
}
#endif

#endif
