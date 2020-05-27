/**
  ******************************************************************************
  * @file          : ina219.c
  * @brief         : This file provides code for the  INA 219
  * @author        : Kyurkchu  A.      
  ******************************************************************************
  * 
  *       
  ******************************************************************************
  */
		
/* Includes ------------------------------------------------------------------*/
 #include "ina219.h"
 #include "input.h"
 
 extern  I2C_HandleTypeDef hi2c2;
 
 static  float temp_value;
 
 uint8_t TX_buffer[3];
 uint8_t RX_buffer[2];
 
 uint16_t INA219_VDDA_DAC_U;    
 uint16_t INA219_VDDA_TRANS_U;
 
 uint16_t INA219_VDD_PIX_U;
 uint16_t INA219_VDD_PIX_I;
 
 uint16_t INA219_VMMerge_DAC_U;
 uint16_t INA219_VMMerge_DAC_I;
 
 uint16_t INA219_VDDD_U;
 uint16_t INA219_VDDD_I;
 
 uint16_t INA219_VDDA_U;
 uint16_t INA219_VDDA_I;
 
 uint16_t INA219_VCCD_U;
 uint16_t INA219_VDDU_U;
 

 /**
 
 Current_LSB function:
 *******************************************************************************
 * Current_LSB  = Maximum_Expected_Current/2^15; 
	
 * for INA219:2^15 = 32768;
 *******************************************************************************


Calibration function:
 *******************************************************************************
 * Calibration  Register Value =  0.04096/(Current_LSB * R_shunt);
 
 *******************************************************************************
  1. 0.04096 is an internal fixed value used to ensure scaling is maintained properly
	
  2. R_shunt = shunt (Ohm)	
	
	
 Current function:
 *******************************************************************************
    Current = Current_Register * Current_LSB;
 *******************************************************************************
  
 Voltage function:
 *******************************************************************************
    Bus Voltage =  Bus_Voltage_Register * Bus_Voltage_LSB;
			
		Bus_Voltage_LSB = 4mV;
 
 *******************************************************************************
	
*/
 


/**

********************Calibration value calculation:****************************** 
	
	VDDD_CALIBR_VALUE     =   0.04096/(0.00000259*0.33)    
	VDDA_CALIBR_VALUE     =   0.04096/(0.00000701*0.18)
  VDDU_CALIBR_VALUE     =   0.04096/(0.00000701*0.18)
	VDD_PIX_CALIBR_VALUE  =   0.04096/(0.00000335*0.33)
	VCCD_CALIBR_VALUE     =   0.04096/(0.00000732*0.18)
	
*/

/**
**********************************************************************************************************
15  * 14 * 13   *  12  * 11  * 10   * 9    * 8     * 7     * 6    * 5    * 4    * 3    *  2    * 1    * 0
RST * —  * BRNG *  PG1 * PG0 * BADC * BADC * BADC  * BADC  * SADC * SADC * SADC * SADC *  MODE * MODE * MODE
    *    *      *      *     *      *      *       *       *      *      *      *      *       *      *
**********************************************************************************************************
//  0000  0001 1001 1111 = 0x019F

 RST[15]    = 0;
 BRNG[13]   = 0;     0 = 16V FSR;
 PG[12..11] = 00;    gain=1,  range 40 mV 
 BADC[10..7]= 0011;  adc resolution 12 but
 SADC[6..3] = 0011;  adc resolution 12 bit
 MODE[2..0] = 111;   shunt and bus continuos

*/
 const uint16_t CONFIG_REG_VALUE     = 0x019F;

 const uint16_t VDDD_CALIBR_VALUE    = 0xC303;    // 49923;  
 const uint16_t VDDA_CALIBR_VALUE    = 0x7ECD;    // 32461;
 const uint16_t VDDU_CALIBR_VALUE    = 0x7ECD;    // 32461;
 const uint16_t VDD_PIX_CALIBR_VALUE = 0x90BB;    // 37051;
 const uint16_t VCCD_CALIBR_VALUE    = 0x796E;    // 31086;

/**
 ************************************
 Calibration array []  = {CALIBRATION_REG_ADDR, H_CALIBRATION_BYTE, L_CALIBRATION_BYTE};
 ************************************
*/
 
 uint8_t calibration_array_VDDD[]    = {CALIBRATION_REG_ADDR,(VDDD_CALIBR_VALUE >>8),(VDDD_CALIBR_VALUE & 0xff)};
 uint8_t calibration_array_VDDA[]    = {CALIBRATION_REG_ADDR,(VDDA_CALIBR_VALUE >>8),(VDDA_CALIBR_VALUE  &  0xff)};
 uint8_t calibration_array_VDDU[]    = {CALIBRATION_REG_ADDR,(VDDU_CALIBR_VALUE >>8),(VDDU_CALIBR_VALUE & 0xff)};
 uint8_t calibration_array_VDD_PIX[] = {CALIBRATION_REG_ADDR,(VDD_PIX_CALIBR_VALUE >>8),(VDD_PIX_CALIBR_VALUE & 0xff)};
 uint8_t calibration_array_VCCD[]    = {CALIBRATION_REG_ADDR,(VCCD_CALIBR_VALUE >>8),(VCCD_CALIBR_VALUE & 0xff)};
  
 
 /**
 ************************************
 Configuration array []  = {CONFIGURATION_REG_ADDR, H_CONFIGURATION_BYTE, L_CONFIGURATION_BYTE};
 ************************************
*/
 
 uint8_t configuration_array[] = {CONFIG_REG_ADDR,(CONFIG_REG_VALUE >>8),(CONFIG_REG_VALUE & 0xff)};
 
  
//*************INIT for all INA 219*******************/
  /**
  * @brief  init all INA 219
  * @retval void
  */
 void INA219_init (void){
	 set_configuration_INA219();
	 set_calibration_INA219();
 }
 
 
//************* SET CONFIGURATION REGISTERS for all INA 219***/
 /**
  * @brief  set configuration registers for all INA 219
  * @retval void
  */
 
 void set_configuration_INA219(void){
		I2C_WriteBuffer_INA219(SLAVE_ADDR_VDDPIX,configuration_array,TX_BUF_SIZE); 
	  I2C_WriteBuffer_INA219(SLAVE_ADDR_VDDD,configuration_array,TX_BUF_SIZE); 
	  I2C_WriteBuffer_INA219(SLAVE_ADDR_VDDA,configuration_array,TX_BUF_SIZE); 
	  I2C_WriteBuffer_INA219(SLAVE_ADDR_VCCD,configuration_array,TX_BUF_SIZE); 
	  I2C_WriteBuffer_INA219(SLAVE_ADDR_VDDU,configuration_array,TX_BUF_SIZE); 
	  I2C_WriteBuffer_INA219(SLAVE_ADDR_VDDA_TRANS,configuration_array,TX_BUF_SIZE); 
	  I2C_WriteBuffer_INA219(SLAVE_ADDR_VDDA_DAC,configuration_array,TX_BUF_SIZE); 
	  I2C_WriteBuffer_INA219(SLAVE_ADDR_VMM_DAC ,configuration_array,TX_BUF_SIZE); 
	}
 
	
 //************ SET CALIBRATION REGISTERS for all INA 219***/
	 /**
  * @brief  set calibration registers for all INA 219
  * @retval void
  */
	
void set_calibration_INA219(void){
	  I2C_WriteBuffer_INA219(SLAVE_ADDR_VDDPIX,calibration_array_VDD_PIX,TX_BUF_SIZE);
	  I2C_WriteBuffer_INA219(SLAVE_ADDR_VDDD,calibration_array_VDDD,TX_BUF_SIZE);
	  I2C_WriteBuffer_INA219(SLAVE_ADDR_VDDA,calibration_array_VDDA,TX_BUF_SIZE);
	  I2C_WriteBuffer_INA219(SLAVE_ADDR_VCCD,calibration_array_VCCD,TX_BUF_SIZE);
	  I2C_WriteBuffer_INA219(SLAVE_ADDR_VDDU,calibration_array_VDDU,TX_BUF_SIZE); 	
	}
 
 //************* WRITE BUFFER  ********************/
	
  /**
  * @brief  write buffer INA 219
  * @retval void
  */
	
 void I2C_WriteBuffer_INA219(uint8_t I2C_ADDRESS,uint8_t *pTXBUFFER, uint8_t TXBUFFERSIZE){
	 
	  while (HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)I2C_ADDRESS,(uint8_t *)pTXBUFFER,(uint16_t)TXBUFFERSIZE, 100) != HAL_OK){
		 
			if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF)
			 	{
	    	 // _Error_Handler(__FILE__,pTXBUFER[0] );
		   	}
		}
    
		while(HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY){};
 }

 //************ READ BUFFER ***********************/
  /**
  * @brief  read buffer INA 219
  * @retval void
  */
 
 void I2C_ReadBuffer_INA219(uint8_t I2C_ADDRESS,uint8_t Reg_ADDRESS, uint8_t *pRXBUFFER, uint8_t RXBUFFERSIZE){
	 
	   I2C_WriteBuffer_INA219(I2C_ADDRESS, &Reg_ADDRESS, 1);
	 

	   while (HAL_I2C_Master_Receive(&hi2c2, (uint16_t)I2C_ADDRESS,(uint8_t *)pRXBUFFER,(uint16_t)RXBUFFERSIZE, 100) != HAL_OK)
			 {
		
		      if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF)
						{
			      // _Error_Handler(__FILE__,pTXBUFER[0] );
			      }
	   	}
    
		while(HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY){};
			 
 }
  
 
 //**************** READ REGISTER ***************************/
  /**
  * @brief  read register INA 219
  * @retval uint16_t 
  */
 
 uint16_t ReadRegister_INA219(uint8_t I2C_ADDRESS, uint8_t Receive_registr){
    
		  uint16_t Value_registr;
		 
      HAL_I2C_Master_Transmit (&hi2c2,I2C_ADDRESS, &Receive_registr , 1, 100); 
      HAL_I2C_Master_Receive  (&hi2c2,I2C_ADDRESS, RX_buffer,RX_BUF_SIZE, 100); 
	    Value_registr = (RX_buffer[1]<<8|RX_buffer[0]); 
	 
		  return Value_registr;
 }
 
 //*********** WRITE REGISTER ********************************/

  /**
  * @brief  write register INA 219
  * @retval void
  */
 
void WriteRegister_INA219(uint8_t I2C_ADDRESS,uint8_t* TX_data){
 
     HAL_I2C_Master_Transmit(&hi2c2,I2C_ADDRESS, TX_data,TX_BUF_SIZE, 100); 
}


//********* CONVERT INT DATA TO FLOAT**************************/

  /**
  * @brief  convet int data to float
  * @retval float
  */
 float ConvertIntDataToFloat_INA2191(unsigned short int Value, float LSB_Value){
	 
	temp_value  = (float)((unsigned short int)Value *LSB_Value);
	 
	 return temp_value;	 
}

 //*********** READ ALL REGISTERS*****************************/

  /**
  * @brief  read registers all INA 219
  * @retval void
  */

void Read_all_INA219(void){
	
	INA219_VDDA_DAC_U =  ReadRegister_INA219(SLAVE_ADDR_VDDA_DAC,BUS_VOLTAGE_REG_ADDR);
	
  INA219_VDDA_TRANS_U = ReadRegister_INA219(SLAVE_ADDR_VDDA_TRANS,BUS_VOLTAGE_REG_ADDR );
 
  INA219_VDD_PIX_U = ReadRegister_INA219(SLAVE_ADDR_VDDPIX,BUS_VOLTAGE_REG_ADDR );
  INA219_VDD_PIX_I = ReadRegister_INA219(SLAVE_ADDR_VDDPIX,SHUNT_VOLTAGE_REG_ADDR );
 
  INA219_VMMerge_DAC_U = ReadRegister_INA219(SLAVE_ADDR_VMM_DAC,BUS_VOLTAGE_REG_ADDR );
  INA219_VMMerge_DAC_I = ReadRegister_INA219(SLAVE_ADDR_VMM_DAC,SHUNT_VOLTAGE_REG_ADDR );
 
  INA219_VDDD_U = ReadRegister_INA219(SLAVE_ADDR_VDDD,BUS_VOLTAGE_REG_ADDR );
  INA219_VDDD_I = ReadRegister_INA219(SLAVE_ADDR_VDDD,SHUNT_VOLTAGE_REG_ADDR );
 
  INA219_VDDA_U = ReadRegister_INA219(SLAVE_ADDR_VDDA,BUS_VOLTAGE_REG_ADDR );
  INA219_VDDA_I = ReadRegister_INA219(SLAVE_ADDR_VDDA,SHUNT_VOLTAGE_REG_ADDR );
 
  INA219_VCCD_U = ReadRegister_INA219(SLAVE_ADDR_VCCD,BUS_VOLTAGE_REG_ADDR );
  INA219_VDDU_U = ReadRegister_INA219(SLAVE_ADDR_VCCD,BUS_VOLTAGE_REG_ADDR );
}


//*********** CONVERT ALL DATA IN219*****************************/

  /**
  * @brief  convert data all INA 219
  * @retval void
  */

void Convert_all_data_IN219(void){
	
 VDDA_DAC_U.fl   = ConvertIntDataToFloat_INA2191((INA219_VDDA_DAC_U >>3),VOLTAGE_LSB );
 VDDA_TRANS_U.fl = ConvertIntDataToFloat_INA2191( (INA219_VDDA_TRANS_U >>3),VOLTAGE_LSB );
 VDD_PIX_U.fl =    ConvertIntDataToFloat_INA2191((INA219_VDD_PIX_U >>3),VOLTAGE_LSB );
 VDD_PIX_I.fl =    ConvertIntDataToFloat_INA2191(INA219_VDD_PIX_I,VDD_PIX_CUREENT_LSB );
	
 VMMerge_DAC_U.fl = ConvertIntDataToFloat_INA2191((INA219_VMMerge_DAC_U >>3),VOLTAGE_LSB );
	
 VDDD_U.fl        = ConvertIntDataToFloat_INA2191((INA219_VDDD_U >>3),VOLTAGE_LSB );
 VDDD_I.fl        = ConvertIntDataToFloat_INA2191(INA219_VDDD_I,VDDD_CURRENT_LSB );
	
 VDDA_U.fl        = ConvertIntDataToFloat_INA2191((INA219_VDDA_U >>3),VOLTAGE_LSB );
 VDDA_I.fl        = ConvertIntDataToFloat_INA2191(INA219_VDDA_I,VDDA_CURRENT_LSB );
	
 VCCD_U.fl        = ConvertIntDataToFloat_INA2191((INA219_VCCD_U >>3),VOLTAGE_LSB );
	
 VDDU_U.fl        = ConvertIntDataToFloat_INA2191((INA219_VDDU_U >>3),VOLTAGE_LSB );
	
}