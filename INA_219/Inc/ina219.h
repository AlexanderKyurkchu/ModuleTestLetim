/**
  ******************************************************************************
  * @file          : ina219.h
  * @brief         : Header for ina219.c file.
  * @author        : Kyurkchu  A.             
  ******************************************************************************
  * 
  *       
  ******************************************************************************
  */

#ifndef _INA219_H
#define _INA219_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#define TX_BUF_SIZE                  3
#define RX_BUF_SIZE                  2

#define CONFIG_REG_ADDR             0x00
#define SHUNT_VOLTAGE_REG_ADDR      0x01
#define BUS_VOLTAGE_REG_ADDR        0x02
#define POWER_REG_ADDR              0x03
#define CURRENT_REG_ADDR            0x04
#define CALIBRATION_REG_ADDR        0x05

#define SLAVE_ADDR_VDDPIX           0x82
#define SLAVE_ADDR_VDDD             0x84
#define SLAVE_ADDR_VDDA             0x86
#define SLAVE_ADDR_VCCD             0x88
#define SLAVE_ADDR_VDDU             0x8A
#define SLAVE_ADDR_VDDA_TRANS       0x8C
#define SLAVE_ADDR_VDDA_DAC         0x8E
#define SLAVE_ADDR_VMM_DAC          0x96

#define VOLTAGE_LSB                 0.004

#define VDDD_CURRENT_LSB            0.00000259      
#define VDDA_CURRENT_LSB            0.00000701
#define VDDU_CURRENT_LSB            0.00000701
#define VCCD_CURRENT_LSB            0.00000732
#define VDD_PIX_CUREENT_LSB         0.00000335


#define VDDD_R_SHUNT                0.33
#define VDDA_R_SHUNT                0.18
#define VDDU_R_SHUNT                0.18
#define VDD_PIX_R_SHUNT             0.33
#define VCCD_R_SHUNT                0.18

#define CURRENT_VDDD                0.85
#define CURRENT_VDD_PIX             0.11
#define CURRENT_VDDU                0.2
#define CURRENT_VDDA                0.23
#define CURRENT_VCCD                0.24
             
extern  uint8_t TX_buffer[3];
extern  uint8_t RX_buffer[2];

void INA219_init(void);
void set_configuration_INA219(void);
void set_calibration_INA219(void);

void I2C_WriteBuffer_INA219(uint8_t I2C_ADDRESS,uint8_t *pTXBUFER, uint8_t TXBUFFERSIZE);
void I2C_ReadBuffer_INA219(uint8_t I2C_ADDRESS,uint8_t Reg_ADDRESS, uint8_t *pRXBUFER, uint8_t RXBUFFERSIZE);
uint16_t ReadRegister_INA219(uint8_t I2C_ADDRESS, uint8_t Receive_registr);
void WriteRegisater_INA219(uint8_t I2C_ADDRESS,uint8_t* TX_data);
void Read_all_INA219(void);

 float ConvertIntDataToFloat_INA2191(unsigned short int Value, float LSB_Value);

void  Convert_all_data_IN219(void);

#ifdef __cplusplus
}
#endif

#endif
