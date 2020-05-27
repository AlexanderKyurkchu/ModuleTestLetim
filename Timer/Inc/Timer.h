/**
  ******************************************************************************
  * @file          : Timer.h
  * @brief         : Header for Timer.c file.
  * @author        : Kyurkchu  A.             
  ******************************************************************************
  * 
  *       
  ******************************************************************************
  */


#ifndef _TIMER_H
#define _TIMER_H

#ifdef __cplusplus
extern "C" {
#endif
	
/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"	
	
	
enum {	
  IDLE = 1,
  START_PROCESS,
  TIMER_PROCESS,
	END_PROCESS
};
	
	
extern __IO uint32_t Timer_Tick;
	
/* Get Tick 1 ms */

uint32_t Timer_GetTick(void);

void Timer_process_delay(uint8_t *FSM_current_state, uint8_t FSM_next_state,uint32_t Delay_time);
	
	
#ifdef __cplusplus
}
#endif

#endif
	