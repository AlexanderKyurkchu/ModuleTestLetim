/**
  ******************************************************************************

  * @file     : input.c
  * @brief    : This file provides code for the read  INPUT DATA (ADC:analog PINS; date from INA 219,discrete PINS)
  * @author   : Kyurkchu  A.
  ******************************************************************************
  * 
  *       
  ******************************************************************************
  */
	
/* Includes ------------------------------------------------------------------*/
#include "input.h"
#include "adc_input.h"


 _Floatdata  VDDD_U;
 _Floatdata  VDDD_I;

 _Floatdata  VDDA_U;
 _Floatdata  VDDA_I;
	
 _Floatdata  V2_5VDD_U;
 _Floatdata  V2_5VDD_I;
	
 _Floatdata  VDDA_DAC_U;
 _Floatdata  VDDA_DAC_I;
	
 _Floatdata  VDD_PIX_U;
 _Floatdata  VDD_PIX_I;	

 _Floatdata  VDDA_TRANS_U;
 _Floatdata  VDDA_TRANS_I;	
	
 _Floatdata  VCCD_U;
 _Floatdata  VCCD_I;

 _Floatdata  VDDU_U;
 _Floatdata  VDDU_I;

 _Floatdata  VMMerge_DAC_U;
 _Floatdata  VMMerge_DAC_I;


  /**
  * @brief  modbus data write process to output holding  registers (ModBus RTU)
  * @retval void
  */

void modbus_data_input_process()
 {
	 
	
	VDDD_U.fl =  adc_average_value1 ; 
  VDDA_U.fl =  adc_average_value2 ;
	 
  //VDDD_U.fl = 0.00 ;
	//VDDD_I.fl = -85.25;
	
	//VDDA_U.fl = 3.57  ;
	//VDDA_I.fl = 70.23 ;
	
	V2_5VDD_U.fl = 2.49;
  //V2_5VDD_I.fl = 50.12;
	
	VDDA_DAC_U.fl = 3.55;
	//VDDA_DAC_I.fl = 34.23;
	
	VDD_PIX_U.fl = 2.51;
	VDD_PIX_I.fl = 3.32;
	
	VDDA_TRANS_U.fl  = 2.61;
	VDDA_TRANS_I.fl  = 20.45;
	
	VCCD_U.fl = 2.43;
  VCCD_I.fl = 21.34;

  VDDU_U.fl = 1.32;
  VDDU_I.fl = 4.66;
	
	VMMerge_DAC_U.fl = 1.23;
	
	VMMerge_DAC_I.fl = 1.43;
	 
	 
	OutRegHoldingBuf[0]  = VDDD_U.data[0];
	OutRegHoldingBuf[1]  = VDDD_U.data[1];;
	OutRegHoldingBuf[2]  = VDDD_I.data[0];
	OutRegHoldingBuf[3]  = VDDD_I.data[1];
	
	OutRegHoldingBuf[4]  = VDDA_U.data[0];
	OutRegHoldingBuf[5]  = VDDA_U.data[1];
	OutRegHoldingBuf[6]  = VDDA_I.data[0];
	OutRegHoldingBuf[7]  = VDDA_I.data[1];
	
  OutRegHoldingBuf[8]  = V2_5VDD_U.data[0];
	OutRegHoldingBuf[9]  = V2_5VDD_U.data[1];
	OutRegHoldingBuf[10] = VMMerge_DAC_U.data[0];       //VMMerge_DAC_U
	OutRegHoldingBuf[11] = VMMerge_DAC_U.data[1];       //VMMerge_DAC_U
	
  OutRegHoldingBuf[12]  = VDDA_DAC_U.data[0];
	OutRegHoldingBuf[13]  = VDDA_DAC_U.data[1]; 
	OutRegHoldingBuf[14]  = VMMerge_DAC_I.data[0];      //VMMerge_DAC_I
	OutRegHoldingBuf[15]  = VMMerge_DAC_I.data[1];      //VMMerge_DAC_I
	
	OutRegHoldingBuf[16]  = VDD_PIX_U.data[0];
	OutRegHoldingBuf[17]  = VDD_PIX_U.data[1];
	OutRegHoldingBuf[18]  = VDD_PIX_I.data[0];
	OutRegHoldingBuf[19]  = VDD_PIX_I.data[1];
	
	OutRegHoldingBuf[20]  = VDDA_TRANS_U.data[0];
	OutRegHoldingBuf[21]  = VDDA_TRANS_U.data[1];
	//OutRegHoldingBuf[22]  = VDDA_TRANS_I.data[0];
	//OutRegHoldingBuf[23]  = VDDA_TRANS_I.data[1];
	
  OutRegHoldingBuf[24]  = VCCD_U.data[0];
	OutRegHoldingBuf[25]  = VCCD_U.data[1];
	//OutRegHoldingBuf[26]  = VCCD_I.data[0];
	//OutRegHoldingBuf[27]  = VCCD_I.data[1];
	
	OutRegHoldingBuf[28]  = VDDU_U.data[0];
	OutRegHoldingBuf[29]  = VDDU_U.data[1];
	
	
	
/*#############  section  for algorithms ######## */		
	
	//OutRegHoldingBuf[30]  = VDDU_I.data[0];
	//OutRegHoldingBuf[31]  = VDDU_I.data[1];
	 
 }

  /**
  * @brief  read state discrete pin
  * @retval uint8_t
  */
 
uint8_t read_input_1(void)
{
    if(HAL_GPIO_ReadPin( INPUT_PORT,INPUT_1) == GPIO_PIN_SET)
		{
		    return 1 ;
		}
		else 
		{
			  return 0;
		}
}

 /**
  * @brief  read state discrete pin
  * @retval uint8_t
  */
uint8_t  read_input_2(void)
{
	 if(HAL_GPIO_ReadPin( INPUT_PORT,INPUT_2) == GPIO_PIN_SET)
		{
		  return 1 ;
		}
		else 
		{
			return 0;
		}
}
 /**
  * @brief  read state discrete pin
  * @retval uint8_t
  */
uint8_t  read_input_3(void)
{
 	return 0;
}
 /**
  * @brief  read state discrete pin
  * @retval uint8_t
  */
uint8_t read_input_4(void)

{
	return 0;
}

 /**
  * @brief  read state discrete pin
  * @retval uint8_t
  */
uint8_t  read_input_5(void)
	
{
	return 0;
}
 /**
  * @brief  read state discrete pin
  * @retval uint8_t
  */
uint8_t  read_input_6(void)
{
	return 0;
}
 /**
  * @brief  read state discrete pin
  * @retval uint8_t
  */
uint8_t  read_input_7(void)
{
	return 0;
}
 /**
  * @brief  read state discrete pin
  * @retval uint8_t
  */
uint8_t  read_input_8(void)
{
	return 0;
}

