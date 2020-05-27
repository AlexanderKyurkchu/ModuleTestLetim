/**
  ******************************************************************************

  * @file     : Timer.c
  * @brief    : This file provides code for the Timer for no block delay
  * @author   : Kyurkchu  A.
  ******************************************************************************
  * 
  *       
  ******************************************************************************
  */
	/* Includes ------------------------------------------------------------------*/

#include "Timer.h"


__IO uint32_t Timer_Tick;

/* Get Tick 1 ms */

 uint32_t Timer_GetTick(void)
  {
    return Timer_Tick;
  }
	
/* no blocking delay  */
void Timer_process_delay(uint8_t *FSM_current_state, uint8_t FSM_next_state,uint32_t Delay_time){
	
 static uint8_t timer_state = IDLE;
 static uint32_t begin_time = 0;	
	
	switch(timer_state)	
	{
		case IDLE:
			timer_state = START_PROCESS;
			break;
		
    case START_PROCESS:
			begin_time = (uint32_t)Timer_GetTick();
		  timer_state = TIMER_PROCESS;
			break;
		
    case TIMER_PROCESS:
			
		if(Timer_GetTick() -begin_time > Delay_time)
		  {
				*FSM_current_state = FSM_next_state;
			  timer_state = IDLE;
		  }
		
			break;
			 
		default:
      break;		
	}
		
}
	