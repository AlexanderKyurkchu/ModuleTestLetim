/**
  ******************************************************************************

  * @file     : Test_algorithm1.c
  * @brief    : This file provides code for the testing U25-U29
  * @author   : Kyurkchu  A.
  ******************************************************************************
  * 
  *       
  ******************************************************************************
  */
	/* Includes ------------------------------------------------------------------*/
	
#include "Test_algorithm1.h"
#include "Timer.h"
#include "main.h"


void  test_algorithm1()
	{
	
  static uint8_t testing_state = ENABLE_POWER;
	
  switch(testing_state)
		
	 {
		case ENABLE_POWER:
			break;
		
/**************  MEASUREMENT_U25 ********************/	
		case DELAY_BEGIN_U25:
			
		  Timer_process_delay(&testing_state, MEASUREMENT_U25,100);
			break;
		
		case MEASUREMENT_U25:
			break;
		
		case DELAY_END_U25:
			
		 Timer_process_delay(&testing_state, DELAY_BEGIN_U26,100);
		
			break;
		
/**************  MEASUREMENT_U26 ********************/	
		case DELAY_BEGIN_U26:
			
			Timer_process_delay(&testing_state, MEASUREMENT_U26,100);
			break;
		
		case MEASUREMENT_U26:
			break;
		
		case DELAY_END_U26:
			
		 Timer_process_delay(&testing_state, DELAY_BEGIN_U27,100);
			break;
/**************  MEASUREMENT_U27 ********************/		
		case DELAY_BEGIN_U27:
			
			Timer_process_delay(&testing_state, MEASUREMENT_U27,100);
			break;
		case MEASUREMENT_U27:
			break;
		
		case DELAY_END_U27:
		Timer_process_delay(&testing_state, DELAY_BEGIN_U28,100);
			break;
/**************  MEASUREMENT_U28 *******************/		
		case DELAY_BEGIN_U28:
			
			Timer_process_delay(&testing_state, MEASUREMENT_U28,100);
			break;
		
		case MEASUREMENT_U28:
			break;
		
		case  DELAY_END_U28:
		Timer_process_delay(&testing_state, DELAY_BEGIN_U27,100);
			break;
	
/**************  MEASUREMENT_U29 ********************/			
		case DELAY_BEGIN_U29:
			
			Timer_process_delay(&testing_state, MEASUREMENT_U29,100);
			break;
		
		case MEASUREMENT_U29:
			break;
		
		case DELAY_END_U29:
		Timer_process_delay(&testing_state,DISABLE_POWER,100);
			break;
	
		case DISABLE_POWER:
		break;
		 	 
	 }
 }
	