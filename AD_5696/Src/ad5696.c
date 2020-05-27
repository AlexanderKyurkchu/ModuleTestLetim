/**
  ******************************************************************************
  * @file           : ad5696.c
  * @brief          : This file provides code for the 16-DAC ad5696 with I2C Interface
  * @author         : Kyurkchu  A.               
  ******************************************************************************
  *
  *       
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
 #include "ad5696.h"
 
 /**
 Transfer function:
 
************************************
  Uout = VREF * Gain * (D/65535);
	
************************************	
	
	1. VREF is the value of the external reference. VREF = 2.048 V
	
  2. Gain is the gain of the output amplifier, Gain = 2.
    
  3. D is the decimal equivalent of the binary code that is loaded to

     and 0 to 65,535 for the 16-bit AD5696.
		

Convert function:

************************************

	D = (Uout /(VREF * Gain)) * 65535;
	
************************************	
	
	1. VREF is the value of the external reference.
	
  2. Gain is the gain of the output amplifier , Gain = 2.
    
  3. Uout is the float value of channels (A,B,C,D).

    
 */
 
extern I2C_HandleTypeDef hi2c2;
uint8_t TXbuffer[3];
uint8_t RXbuffer[3];

 //*********** INIT AD5696 AND SET INIT DATA **********************/
 /**
  * @brief  AD5696 init and set data
  * @retval void
  */

void AD5696_init(void){
	
	if(HAL_I2C_IsDeviceReady(&hi2c2,AD5696_ADDRESSE, 2, 10)==HAL_OK)
	  {
	     set_value_to_channel_AD5696(COMM_WRITE_AND_UPDATE,ADDR_DAC_A,ADC_VALUE_A);       // for VDDA_DAC
	     set_value_to_channel_AD5696(COMM_WRITE_AND_UPDATE,ADDR_DAC_B,ADC_VALUE_B_MIN);   // for VDDA_TRANS
	     set_value_to_channel_AD5696(COMM_WRITE_AND_UPDATE,ADDR_DAC_C,ADC_VALUE_C_MIN);   // for VDDA_PIX
	     set_value_to_channel_AD5696(COMM_WRITE_AND_UPDATE,ADDR_DAC_D,ADC_VALUE_D);       // for VMMerge_DAC
	  }
	else
    {
	
  	}
}
 //*********** SET VALUE TO SELECT CHANNEL of AD5696 ***********/
void set_value_to_channel_AD5696(uint8_t command, uint8_t dac_channel, uint16_t data){
		
	    TXbuffer[0] =  (command <<4)| dac_channel;
		  TXbuffer[1] =  (data>>8);
		  TXbuffer[2] =  data & 0xff;
		
		HAL_I2C_Master_Transmit(&hi2c2,AD5696_ADDRESSE, TXbuffer, 3,100);
		
  }

//*********** WRITE AND UPDATE DATA FOR  SELECTED CHANNEL AD5696 */	
	
	/**
  * @brief  write and update data for selected channel of AD5696
  * @retval void
  */

void write_and_update_DAC_AD5696(uint16_t data, uint8_t dac_channel)
  {
       TXbuffer[0] = (COMM_WRITE_AND_UPDATE <<4)| dac_channel;
		   TXbuffer[1] = (data>>8);
		   TXbuffer[2] = data & 0xff;
		
		HAL_I2C_Master_Transmit(&hi2c2,AD5696_ADDRESSE, TXbuffer, 3,100);

  }
 
//*********** CONVERT FLOAT DATA TO DIGITAL DATA AD5696 ****/
	 /**
  * @brief  convert float data to digital data AD5696 
  * @retval void
  */
	
 uint16_t convert_float_to_data_DAC_AD5696(float FloatInputData)
 {
    uint16_t Dout = 0x00;
	 
	  Dout = (FloatInputData /( U_REF * GAIN )) * 65535;	
	 
	  return  Dout;
 }
	