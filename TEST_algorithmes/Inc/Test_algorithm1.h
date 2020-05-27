/**
  ******************************************************************************

  * @file     : Test_algorithm1.h
  * @brief    : header file for Test_algorithm1.c
  * @author   : Kyurkchu  A.
  ******************************************************************************
  * 
  *       
  ******************************************************************************
  */
	/* Includes ------------------------------------------------------------------*/


#ifndef _TEST_ALGORITHM1_H
#define _TEST_ALGORITHM1_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stdbool.h"


enum STATE_ALGORITHM1
	{

   ENABLE_POWER = 1,
	 DELAY_BEGIN_U25,
   MEASUREMENT_U25,
	 DELAY_END_U25,
	
	 DELAY_BEGIN_U26,
   MEASUREMENT_U26,
	 DELAY_END_U26,
	
	
	 DELAY_BEGIN_U27,
   MEASUREMENT_U27,
	 DELAY_END_U27,
	
	 DELAY_BEGIN_U28,
	 MEASUREMENT_U28,
	 DELAY_END_U28,
	
	 DELAY_BEGIN_U29,
	 MEASUREMENT_U29,
	 DELAY_END_U29,
	
	 DISABLE_POWER	
} ;


void  test_algorithm1(void);
                      


#ifdef __cplusplus
}
#endif

#endif
