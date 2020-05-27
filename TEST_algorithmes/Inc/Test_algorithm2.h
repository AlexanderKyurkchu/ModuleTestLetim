/**
  ******************************************************************************

  * @file     : Test_algorithm2.h
  * @brief    : header file for Test_algorithm2.c
  * @author   : Kyurkchu  A.
  ******************************************************************************
  * 
  *       
  ******************************************************************************
  */
	/* Includes ------------------------------------------------------------------*/


#ifndef _TEST_ALGORITHM2_H
#define _TEST_ALGORITHM2_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stdbool.h"

enum STATE_ALGORITHM2
	{

   ENABLE_U3 = 1,
	 DELAY_BEGIN_U3,
   MEASUREMENT_U3,
	 DELAY_END_U3,
	
	 ENABLE_U4,
	 DELAY_BEGIN_U4,
   MEASUREMENT_U4,
	 DELAY_END_U4,
	
	 ENABLE_U5,
	 DELAY_BEGIN_U5,
   MEASUREMENT_U5,
	 DELAY_END_U5,
	
	 DELAY_BEGIN_U8,
	 MEASUREMENT_U8,
	 DELAY_END_U8,
	
	 DELAY_BEGIN_U9,
	 MEASUREMENT_U9,
	 DELAY_END_U9,
	 
	 DISABLE_ALL	
} ;



                      
void  test_algorithm2(void);
		
float measuring_output_U3(void);

float measuring_output_U4(void);

float measuring_output_U5(void);

float measuring_output_U8(void);

float measuring_output_U9(void);

void U3_enable(void);

void U4_enable(void);

void U5_enable(void);

void U3_disable(void);

void U4_disable(void);

void U5_disable(void);

#ifdef __cplusplus
}
#endif

#endif
