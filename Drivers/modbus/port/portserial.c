/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );

extern UART_HandleTypeDef huart4;

void  USER_UART_IRQ_Handler(void);
/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
	if (xRxEnable) 
		{
		__HAL_UART_ENABLE_IT(&huart4,  UART_IT_RXNE);
  	} 
	else
	{
		__HAL_UART_DISABLE_IT(&huart4,  UART_IT_RXNE);
	}

	if (xTxEnable) 
		{
		__HAL_UART_ENABLE_IT(&huart4, UART_IT_TXE);
	  } 
	else 
		{
		__HAL_UART_DISABLE_IT(&huart4, UART_IT_TXE);
	}	
	
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
	
	switch (ucPORT)
  {
	case 0:
          huart4.Instance = UART4;
          break;
 	case 1:
          huart4.Instance = USART2;
          break;
	case 2:
          huart4.Instance = USART3;
          break;
        default:
          return FALSE;
  }
	
	
	huart4.Init.BaudRate = ulBaudRate;
	
	
	 switch (ucDataBits)
  {
        case 8:
                huart4.Init.WordLength = UART_WORDLENGTH_8B;
                break;
        default:
                return FALSE;
  }

	  switch (eParity)
  {
    case MB_PAR_NONE:
            huart4.Init.Parity = UART_PARITY_NONE;
            break;
    case MB_PAR_EVEN:
            huart4.Init.Parity = UART_PARITY_EVEN;
            break;
    case MB_PAR_ODD:
            huart4.Init.Parity = UART_PARITY_ODD;
            break;
    default:
            return FALSE;
  }
	
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16; 
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    //Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    //Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    //Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
   // Error_Handler();
  }

    return TRUE;
}



BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
    * called. */

	 huart4.Instance->TDR =  ucByte;
   return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
	  *pucByte = huart4.Instance->RDR;
    return TRUE;
   
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
static void prvvUARTTxReadyISR( void )
{

    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void prvvUARTRxISR( void )
{
	
    pxMBFrameCBByteReceived(  );
}


 void USER_UART_IRQ_Handler(void) {
	
  uint32_t tmp_flag = 0, tmp_it_source = 0;
	 
	 
  tmp_flag = __HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(&huart4, UART_IT_RXNE);
  /* UART in mode Receiver ---------------------------------------------------*/
  if((tmp_flag != RESET))
  { 
    prvvUARTRxISR(  ); 
  }
  
  tmp_flag = __HAL_UART_GET_FLAG(&huart4, UART_FLAG_TXE);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(&huart4, UART_IT_TXE);

  /* UART in mode Transmitter ------------------------------------------------*/
  if((tmp_flag != RESET))
  {
    prvvUARTTxReadyISR(  );
  } 

	
}
