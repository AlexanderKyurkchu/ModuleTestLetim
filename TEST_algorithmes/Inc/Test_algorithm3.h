/**
  ******************************************************************************

  * @file     : Test_algorithm3.h
  * @brief    : header file for Test_algorithm1.c
  * @author   : Kyurkchu  A.
  ******************************************************************************
  * 
  *       
  ******************************************************************************
  */
	/* Includes ------------------------------------------------------------------*/


#ifndef _TEST_ALGORITHM3_H
#define _TEST_ALGORITHM3_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stdbool.h"


enum STATE_ALGORITHM3
	{
    SET_OUT1_3_6V = 1,
		DELAY_OUT1,
		MEASUREMENT_U7,
		DELAY_END_OUT1,
		
		SET_OUT2_2_5V,
		DELAY_OUT2_1,
		MEASUREMENT_U11_1,
		DELAY_END_OUT2,
		
		SET_OUT2_3_3V,
		DELAY_OUT2_2,
		MEASUREMENT_U11_2,
		DELAY_END_OUT2_2,
		
		SET_OUT3_2_0V,
		DELAY_OUT3_1,
		MEASUREMENT_U6_1,
		DELAY_END_OUT3,
		
		SET_OUT3_3_0V,
		DELAY_OUT3_2,
		MEASUREMENT_U6_2,
		DELAY_END_OUT3_2,
		
		SET_OUT4,
    DELAY_OUT4,
		MEASUREMENT_TP10,	
    DELAY_END_OUT4	
} ;


void test_algorithm3(void);
                
#ifdef __cplusplus
}
#endif

#endif
