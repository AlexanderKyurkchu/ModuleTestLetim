/**
  ******************************************************************************

  * @file     : Test_algorithm2.c
  * @brief    : This file provides code for the testing U1,U3,U4,U5,U8,U9
  * @author   : Kyurkchu  A.
  ******************************************************************************
  * 
  *       
  ******************************************************************************
  */
	/* Includes ------------------------------------------------------------------*/

#include "Test_algorithm2.h"
#include "Timer.h"
#include "main.h"


void  test_algorithm2()
	{
	
     static uint8_t testing_state = ENABLE_U3;
	
   switch(testing_state)
	  {
	/**************  MEASUREMENT_U3 ********************/
		 
		case ENABLE_U3:
			
			   HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
		     testing_state = DELAY_BEGIN_U3;
		
				break;		
		case DELAY_BEGIN_U3:
			
			   Timer_process_delay(&testing_state, MEASUREMENT_U3,100);
		
		     break;	
		case MEASUREMENT_U3:                                    // action 	MEASUREMENT_U3
			
		      measuring_output_U3();
		
			    HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);  
		      testing_state = DELAY_END_U3;		
			break;
		
		case DELAY_END_U3:
			
		  Timer_process_delay(&testing_state, ENABLE_U4,100);
			break;
		
	/************END MEASUREMENT_U3************/	
		
			
	/************MEASUREMENT_U4****************/	
		case ENABLE_U4:
			
		  HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
		  testing_state = DELAY_BEGIN_U4;	
			break;
		
		case DELAY_BEGIN_U4:
			
	       Timer_process_delay(&testing_state, MEASUREMENT_U4,100);
	  	break;
		
	  case 	MEASUREMENT_U4:                                 // action 	MEASUREMENT_U4
			
		    measuring_output_U4();
		    HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);  
		    testing_state = DELAY_END_U4;
	   	break;
			
		case DELAY_END_U4:
			
		Timer_process_delay(&testing_state, ENABLE_U3,100);
			
		break;
	/***********END MEASUREMENT_U4************/	
		
		
	/************MEASUREMENT_U5***************/	
	  case  ENABLE_U5:
			
		   break;
		
		case DELAY_BEGIN_U5:
			
		 
	  	break;
	
    case  MEASUREMENT_U5:
			
		  measuring_output_U5();
		
	   	break;
		
		case DELAY_END_U5:
			
		  
	  	break;
			
	/***********END MEASUREMENT_U5************/	
		
	/************MEASUREMENT_U8***************/	
		case DELAY_BEGIN_U8:
				
		break;
		
	  case  MEASUREMENT_U8:
			
		 measuring_output_U8();
		
	  break;
		
		case DELAY_END_U8:
				
		break;
/***********END MEASUREMENT_U8************/		
		
/************MEASUREMENT_U9***************/	
		case DELAY_BEGIN_U9:
				
		  break;
		
	  case  MEASUREMENT_U9:
			
		   measuring_output_U9();	
		 break;
		case DELAY_END_U9:
				
		break;
		
/***********END MEASUREMENT_U9************/		

/***********DISABLE ALL MEASUREMENTS******/		
		
	  case  DISABLE_ALL	:
		  break;
	
	  default:
		   break;
	}
} 


float measuring_output_U3(){

	 float tmp_value = 0.0;
	
	return tmp_value;

}

float measuring_output_U4(){

	 float tmp_value = 0.0;
	
	return tmp_value;

}


float measuring_output_U5(){
	
  float tmp_value = 0.0;
	
	return tmp_value;

}

float measuring_output_U8(){

  float tmp_value = 0.0;
	
	return tmp_value;
}

float measuring_output_U9(){
	
  float tmp_value = 0.0;
	
	return tmp_value;

}


void U3_enable()
{
	

}

void U4_enable()
{

}

void U5_enable()
{

}


void U3_disable()
{

}

void U4_disable()
{

}

void U5_disable()
{

}
