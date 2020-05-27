/**
  ******************************************************************************
  * @file     : Test_algorithm3.c
  * @brief    : This file provides code for the testing U7,U11,U6, TP10
  * @author   : Kyurkchu  A.
  ******************************************************************************
  *  U7   (Vout1 DAC U1)
	*  U11  (Vout2 DAC U1)
	*  U6   (Vout3 DAC U1)
	*	TP10  (Vout4 DAC U1)
  *       
  ******************************************************************************
  */
	/* Includes ------------------------------------------------------------------*/

#include "Test_algorithm3.h"
#include "Timer.h"
#include "main.h"


void  test_algorithm3()
	{
	
  static uint8_t testing_state = 	SET_OUT1_3_6V;
	
  switch(testing_state)
	 {
   /**************  MEASUREMENT_U7 ********************/			  
		case SET_OUT1_3_6V:
			
		  testing_state = DELAY_OUT1;
			break;
		case DELAY_OUT1:
					
		  Timer_process_delay(&testing_state, MEASUREMENT_U7,100);
			break;
		case MEASUREMENT_U7:
			
		  //MEASUREMENT_U7 action
		  testing_state = DELAY_END_OUT1;
			break;
		
		case	DELAY_END_OUT1:
			
			Timer_process_delay(&testing_state, SET_OUT2_2_5V,100);
			break;
		
/**************  MEASUREMENT_U11 2.5V *****************/		
		
		case SET_OUT2_2_5V:
			
			testing_state = DELAY_OUT2_1;
			break;
		
		case DELAY_OUT2_1:
		
			Timer_process_delay(&testing_state, MEASUREMENT_U11_1,100);
			break;
		
		case MEASUREMENT_U11_1:
			//MEASUREMENT_U11_1 action
			testing_state = DELAY_END_OUT2;
			break;
		
		case DELAY_END_OUT2:
			
			Timer_process_delay(&testing_state,SET_OUT2_3_3V,100);
			break;
		
/**************  MEASUREMENT_U11 3.3V ******************/				
		case SET_OUT2_3_3V:
			
		  testing_state = DELAY_OUT2_2;
			break;
		case DELAY_OUT2_2:
			
				Timer_process_delay(&testing_state, MEASUREMENT_U11_2,100);
			break;
		
		case MEASUREMENT_U11_2:
			
		 //MEASUREMENT_U11_2 action
		
			break;
		case DELAY_END_OUT2_2:
			break;
/**************  MEASUREMENT_U6 2.0V ******************/			
		case SET_OUT3_2_0V:
			
			 testing_state = DELAY_OUT3_1;
			 break;
		
		case DELAY_OUT3_1:
			
			Timer_process_delay(&testing_state, MEASUREMENT_U6_1,100);
			
			break;
		case 	MEASUREMENT_U6_1:
			
		 //MEASUREMENT_U6_1 action
		
			testing_state = DELAY_END_OUT3;
			break;
		
		case DELAY_END_OUT3:
			
			Timer_process_delay(&testing_state, SET_OUT3_3_0V,100);	
			break;
/**************  MEASUREMENT_U6 3.3V ******************/			
		case SET_OUT3_3_0V:
			
			testing_state =  DELAY_OUT3_2;
			break;
		
		case DELAY_OUT3_2:
			
			Timer_process_delay(&testing_state, MEASUREMENT_U6_2,100);
			break;
		case 	MEASUREMENT_U6_2:
			
			//MEASUREMENT_U6_2 action
		
		  testing_state = DELAY_END_OUT3_2;
			break;
		
		case DELAY_END_OUT3_2:
			
			Timer_process_delay(&testing_state, SET_OUT4,100);
			break;
		
/**************  MEASUREMENT_TP10 ******************/			
		case SET_OUT4:
			
		   testing_state =  DELAY_OUT4;
		
			break;
		
    case DELAY_OUT4:
			
				Timer_process_delay(&testing_state, MEASUREMENT_TP10,100);
			break;
		
		case MEASUREMENT_TP10:
			
			//MEASUREMENT_TP10  action
		
		  testing_state = DELAY_END_OUT4;
		
    break;	

    case DELAY_END_OUT4:
			
			break;
		
		
	 }

 }
	