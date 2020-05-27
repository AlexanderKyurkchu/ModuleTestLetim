/**
  ******************************************************************************
  * @file           : ad5696.h
  * @brief          : Header for ad5696.c file.
  * @author         : Kyurkchu  A.               
  ******************************************************************************
  * 
  *       
  ******************************************************************************
  */

#ifndef _AD5696_H
#define _AD5696_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#define AD5696_ADDRESSE        0x18

#define ADDR_DAC_A             0x01
#define ADDR_DAC_B             0x02
#define ADDR_DAC_C             0x04
#define ADDR_DAC_D             0x08
#define ADDR_DAC_ALL           0x0f


#define COMM_N0                0x00
#define COMM_WRITE             0x01 
#define COMM_UPDATE            0x02
#define COMM_WRITE_AND_UPDATE  0x03
#define COMM_POWR_DOWN__UP     0x04
#define COMM_SOFTWARE_RESET    0x06



#define ADC_VALUE_A            0xE0FF      // D = 57599, for 3.6 V         
#define ADC_VALUE_B_MIN        0x9C3F      // D = 39999, for 2.5 V
#define ADC_VALUE_B_MAX        0xCE3F      // D = 52799, for 3.3 V
#define ADC_VALUE_C_MIN        0x7CFF      // D = 31999, for 2.0 V
#define ADC_VALUE_C_MAX        0xBB7F      // D = 47999, for 3.0 V
#define ADC_VALUE_D            0x00FF      

#define U_REF   2.048                     // Uref= 2.048 
#define GAIN    2                         // Gain = 2


typedef struct _command{
 uint8_t addr_byte;
 uint8_t comm_byte;
 uint8_t data_high_byte;
 uint8_t data_low_byte;
} command;


extern uint8_t TXbuffer[3];
extern uint8_t RXbuffer[3];

void AD5696_init(void);
void set_value_to_channel_AD5696(uint8_t command, uint8_t dac_channel, uint16_t data);
void send_command_AD5696(uint8_t command);
void send_data_AD5696(uint8_t data);
void write_and_update_DAC_AD5696(uint16_t dac_data, uint8_t dac_channel);
uint16_t convert_float_to_data_DAC_AD5696(float FloatData);
void init_send_buffer_AD5696(uint8_t *tx_buffer);

#ifdef __cplusplus
}
#endif

#endif


extern uint8_t TXbuffer[3];
extern uint8_t RXbuffer[3];

void AD5696_init(void);
void set_value_to_channel_AD5696(uint8_t command, uint8_t dac_channel, uint16_t data);
void send_command_AD5696(uint8_t command);
void send_data_AD5696(uint8_t data);
void write_and_update_DAC_AD5696(uint16_t dac_data, uint8_t dac_channel);
uint16_t convert_float_to_data_DAC_AD5696(float FloatData);
void init_send_buffer_AD5696(uint8_t *tx_buffer);

#ifdef __cplusplus
}
#endif

