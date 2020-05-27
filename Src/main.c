/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "eth.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* ----------------------- Modbus includes ----------------------------------*/

#include "mb.h"
#include "mbport.h"
#include "input.h"
#include "output.h"

/* -----------------------ad5696 includes----------------------------------*/
#include "ad5696.h"

/* -----------------------ina219 includes----------------------------------*/
#include "ina219.h"

/* -----------------------adc_input includes--------------------------------*/
#include "adc_input.h"

/* -----------------------algorithm includes--------------------------------*/


#include "Timer.h"
#include "Test_algorithm1.h"
#include "Test_algorithm2.h"
#include "Test_algorithm3.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADC_CONVERTED_DATA_BUFFER_SIZE   ((uint8_t)  2)   /* Size of array aADCxConvertedData[] */

 extern  uint16_t timeout_Timer6 ;
 extern volatile uint16_t counter_Timer6;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//**********************************************************/
//********INPUT and OUTPUT buffers for ModBusRTU************/
//**********************************************************/

//**********DEFINES FOR INPUTS REGISTERS*******************/	

 static USHORT   usRegInputStart = REG_INPUT_START;
 static USHORT   usRegInputBuf[REG_INPUT_NREGS];

//**********DEFINES FOR HOLDING REGISTERS*******************/
 static USHORT   usRegHoldingStart = REG_HOLDING_START;
  SHORT    OutRegHoldingBuf[REG_HOLDING_NREGS];
  SHORT    InputHoldingBuf[REG_HOLDING_NREGS];
	
//**********DEFINES FOR COILS REGISTERS*******************/	
 unsigned char ucRegCoilsBuf[REG_COILS_SIZE];
	
//**********DEFINES FOR DISCRETE REGISTERS*******************/
 static USHORT   usInputStatusStart = INPUT_STATUS_START;
 static USHORT   usInputStatusBuf[INPUT_INPUT_NREGS];

//extern uint8_t TXbuffer[3];


// ****I********Input ADC *******

//ALIGN_32BYTES (static short aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]);

//ALIGN_32BYTES (static short aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE]);

volatile uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];

volatile uint16_t aADCxConvertedData2[ADC_CONVERTED_DATA_BUFFER_SIZE];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//extern uint8_t TXbuffer[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void ModbusRTU_polling(void);
void ADC_data_read (void);
void Algorithm_process(void);

uint8_t Get_current_algorithm(void);

static uint8_t current_algorithm ;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ETH_Init();
  MX_I2C2_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
	
  /* USER CODE BEGIN 2 */
		
/* ######### ModbusRTU init and enable ####### */
	 eMBErrorCode    eStatus;
   eStatus =  eMBInit( MB_RTU, 0x05, 0, 115200, MB_PAR_NONE );
	 eStatus =  eMBEnable();
	 	 
/*########## ad5696 init  and set init value ###### */

     AD5696_init();

/* ######## init all INA_219 ########### */
		
//	 INA219_init ();
		
					
//******** ADC init ********	

 /* ###  Start calibration ############################################ */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
	}
			
 /* ### Start conversion in DMA mode ################################# */
	if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)aADCxConvertedData, ADC_CONVERTED_DATA_BUFFER_SIZE) != HAL_OK) 
		
   {
		 Error_Handler();		
   }	
  	 
	/* #######  Init start for TIMER&, used  no blocking delay ########### */  
	 
   HAL_TIM_Base_Start_IT(&htim7);
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
   /*####### polling ModbusRTU ######## */
		
	  ModbusRTU_polling();
		
   /*####### reading ADC data ######## */	
		ADC_data_read ();
		
   /*####### algorithm process ####### */
		
		Algorithm_process();
		
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
 
 //**** Read date from sensors INA 219********//	

 //  Read_all_INA219();
 //  Convert_all_data_IN219();
		
		
// **************OUTPUT SET**********************//
	if((ucRegCoilsBuf[0])&&(0x01) == 1){
		
		  //  HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);
	    }
	else{
	    //  HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
	  }
	
		if((ucRegCoilsBuf[0]>>1)&&0x01 == 1){
		
		  //  HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
	    }
	else{
	    //  HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
	  }
	
		
	//	test_algorithm2();		
  }



  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 18;
  PeriphClkInitStruct.PLL2.PLL2P = 15;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 6144;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable USB Voltage detector 
  */
  HAL_PWREx_EnableUSBVoltageDetector();
}

/* USER CODE BEGIN 4 */

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START  + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );

            *pucRegBuffer++ = ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}


//**************************eMBRegHoldingCB************************/
eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs,
                 eMBRegisterMode eMode )
{
       eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
            /* Pass current register values to the protocol stack. */
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( unsigned char )( OutRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( OutRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

            /* Update current register values with new values from the
             * protocol stack. */
        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                 InputHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                 InputHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                 iRegIndex++;
                usNRegs--;
            }
        }
				
				
				//	 current_algorithm = Get_current_algorithm();
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

//********************eMBRegCoilsCB********************************/

eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
               eMBRegisterMode eMode )
{
	  eMBErrorCode    eStatus = MB_ENOERR;
    short           iNCoils = ( short )usNCoils;
    unsigned short  usBitOffset;

    /* Check if we have registers mapped at this block. */
    if( ( usAddress >= REG_COILS_START ) &&
        ( usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE ) )
    {
        usBitOffset = ( unsigned short )( usAddress - REG_COILS_START );
        switch ( eMode )
        {
                /* Read current values and pass to protocol stack. */
            case MB_REG_READ:
                while( iNCoils > 0 )
                {
                    *pucRegBuffer++ =
                        xMBUtilGetBits( ucRegCoilsBuf, usBitOffset,
                                        ( unsigned char )( iNCoils >
                                                           8 ? 8 :
                                                           iNCoils ) );
                    iNCoils -= 8;
                    usBitOffset += 8;
                }
                break;

								
								
                /* Update current register values. */
            case MB_REG_WRITE:
                while( iNCoils > 0 )
                {
                    xMBUtilSetBits( ucRegCoilsBuf, usBitOffset,
                                    ( unsigned char )( iNCoils > 8 ? 8 : iNCoils ),
                                    *pucRegBuffer++ );
                    iNCoils -= 8;
                }
                break;
        }

    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

 //********************eMBRegDiscreteCB********************************

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
   eMBErrorCode    eStatus = MB_ENOERR;
	 uint8_t tmpInputdata =  0x00;
	
    if( ( usAddress >= INPUT_STATUS_START )
        && ( usAddress + usNDiscrete <= INPUT_STATUS_START +  INPUT_INPUT_NREGS  ) )
    {
			       // for discrete inputs
			        if(read_input_1())
							{
						    tmpInputdata |= (1 << 0);	
							}
			        if(read_input_2())
							{
							   tmpInputdata |= (1 << 1);	
							}   
							if(read_input_3())
							{
						     tmpInputdata |= (1 << 2);	
							}
			        if(read_input_4())
							{
							   tmpInputdata |= (1 << 3);	
							}

			
		        	*pucRegBuffer = tmpInputdata;
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

//******************************************************************

static uint32_t lock_nesting_count = 0;

void __critical_enter(void)
{
    __disable_irq();
	
	  // HAL_NVIC_DisableIRQ(UART4_IRQn);
	  // HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
   // ++lock_nesting_count;
}
void __critical_exit(void)
{
    /* Unlock interrupts only when we are exiting the outermost nested call. */
  //  --lock_nesting_count;
  //  if (lock_nesting_count == 0) {
			
	//	  HAL_NVIC_EnableIRQ(UART4_IRQn);
	//		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
       __enable_irq();
   // }
}


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	
  /* Invalidate Data Cache to get the updated content of the SRAM on the first half of the ADC converted data buffer: 32 bytes */
    SCB_InvalidateDCache_by_Addr((uint32_t *) &aADCxConvertedData[0], ADC_CONVERTED_DATA_BUFFER_SIZE);	 
		
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
		  
	 	  SCB_InvalidateDCache_by_Addr((uint32_t *) &aADCxConvertedData[1], ADC_CONVERTED_DATA_BUFFER_SIZE);
	    HAL_ADC_Stop_DMA(&hadc1); 
	    flag_adc_dma = 1;
	
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /*
	  * timer6 for ModBus RTU
	*/
	   if(htim->Instance == TIM6) //check if the interrupt comes from TIM6
        {
	
          if((++counter_Timer6)>=timeout_Timer6)
             {
		           counter_Timer6 = 0;
   
		           ( void )pxMBPortCBTimerExpired(  );
            }
       }	
				
 /* 
	 *		timer7 for no block delay, timer tick 1 ms
 */				 
			if(htim->Instance == TIM7) //check if the interrupt comes from TIM7
       {
				Timer_Tick +=(uint32_t)uwTickFreq;
       } 
}

/**
 *  modbus RTU polling
*/

void ModbusRTU_polling()
{
   eMBPoll();
   modbus_data_output_process();
	 modbus_data_input_process();
}

/**
 *  adc data read
*/

void ADC_data_read (){
	
  /*####### filter data from ADC ###### */		
	 if (flag_adc_dma){
		 
        flag_adc_dma = 0;
		 
		    ADC_11=aADCxConvertedData[0];
	  	  ADC_12=aADCxConvertedData[1];
     
       filter_value11 = filter_moving_average(ADC_11,buf_11,&filter_reg1);
	     adc_average_value1 = moving_average_value_to_float(filter_value11);
		 
	     filter_value12 = filter_moving_average(ADC_12,buf_12,&filter_reg2);
	     adc_average_value2 = moving_average_value_to_float(filter_value12);
		 
   /* ### Start conversion in DMA mode ################################# */
	   if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)aADCxConvertedData, ADC_CONVERTED_DATA_BUFFER_SIZE) != HAL_OK) 
		
         {
		      Error_Handler();		
         }	 
    } 
	}

/**
  * algorithm process
*/	


	
void Algorithm_process()
 {
	
	 static uint8_t algorithm_number = ALG_IDLE;
	 
	/*####### get current algorithm ###### */	
	   	  	
  algorithm_number =  (uint8_t)OUTPUT_CURRENT_ALGORITHM.int_data;
	 
	switch(algorithm_number)
		
	    {                             
		     case ALG_IDLE:
				      {  
				 
				      }
				 
			     break;
		     case  ALG_1:
					 
			      test_algorithm1();
				 
			    break;
				 
		     case  ALG_2:
					 
			     test_algorithm2();
				 
			   break;
				 
		     case ALG_3:
					 
			    test_algorithm3();
				 
			  
				 break;
				 
				 default:
					 break;
				 
				 
				 
	    }
	 }

		 
	
uint8_t Get_current_algorithm()
{
    return OUTPUT_CURRENT_ALGORITHM.int_data;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
