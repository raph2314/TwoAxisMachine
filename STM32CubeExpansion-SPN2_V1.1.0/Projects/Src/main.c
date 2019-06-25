/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 09/10/2014 11:13:03
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "xnucleoihm02a1.h"
#include "example.h"
#include "example_usart.h"
#include "stm32f4xx_hal_adc.h"

#define TEST_MOTOR	//!< Comment out this line to test the ADC

/**
  * @defgroup   MotionControl
  * @{
  */

/**
  * @addtogroup BSP
  * @{
  */

/**
  * @}
  */ /* End of BSP */

/**
  * @addtogroup MicrosteppingMotor_Example
  * @{
  */

/**
  * @defgroup   ExampleTypes
  * @{
  */

// #define MICROSTEPPING_MOTOR_EXAMPLE        //!< Uncomment to performe the standalone example
#define MICROSTEPPING_MOTOR_USART_EXAMPLE  //!< Uncomment to performe the USART example
#if ((defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option only!"
#elif ((!defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (!defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
  #error "Please select an option!"
#endif
#if (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE) && (!defined (NUCLEO_USE_USART)))
  #error "Please define "NUCLEO_USE_USART" in "stm32fxxx_x-nucleo-ihm02a1.h"!"
#endif

/**
  * @}
  */ /* End of ExampleTypes */
	
	/* Private Variables ----------------------*/

/* Variable used to get converted value */
__IO uint16_t uhADCxConvertedValue = 0;


/* Private function prototypes -----------------------------------------------*/
//static void SystemClock_Config(void);
static void Error_Handler(void);
void GPIO_CustomInit();
uint16_t Read_ADC(void);

/**
  * @brief The FW main module
  */
int main(void)
{
  /* NUCLEO board initialization */
	/* Init for UART, ADC, GPIO and SPI */
  NUCLEO_Board_Init();
  
  /* X-NUCLEO-IHM02A1 initialization */
  BSP_Init();

  /********************************     POLLING     ****************************/
  /*
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  GPIO_PinState switchOn=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8); 

  USART_Transmit(&huart2, "\n\rTrue\n\r");
  */

  /* Custom GPIO setup */
  GPIO_CustomInit();

  /****************************      Interrupt     ****************************/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /*************************************************************************/

#ifdef NUCLEO_USE_USART
  /* Transmit the initial message to the PC via UART */
  USART_TxWelcomeMessage();
#endif
	
#if defined (MICROSTEPPING_MOTOR_EXAMPLE)
  /* Perform a batch commands for X-NUCLEO-IHM02A1 */
  MicrosteppingMotor_Example_01();
  
  /* Infinite loop */
  while (1);
#elif defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)
  /* Fill the L6470_DaisyChainMnemonic structure */
  Fill_L6470_DaisyChainMnemonic();
	
	/*Initialize the motor parameters */
	Motor_Param_Reg_Init();
  
  // L6470_Run(0, L6470_DIR_FWD_ID, 20000); 
  // L6470_Run(1, L6470_DIR_FWD_ID, 500); 

  // while(1);
  
// #endif
  
  /* Infinite loop */
  while (1)
  {

	
#ifdef TEST_MOTOR		

		/* Check if any Application Command for L6470 has been entered by USART */
    USART_CheckAppCmd();
		
#else
		
		uint16_t myADCVal;
		myADCVal = Read_ADC();
		USART_Transmit(&huart2, " ADC Read: ");
	  USART_Transmit(&huart2, num2hex(myADCVal, WORD_F));
	  USART_Transmit(&huart2, " \n\r");
#endif		
  }
#endif
}

void GPIO_CustomInit (){
  GPIO_InitTypeDef GPIO_InitStruct;

  // //LED output to pin 9
  // GPIO_InitStruct.Pin = GPIO_PIN_9;
  // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // GPIO_InitStruct.Pull = GPIO_PULLUP;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
  // HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); 

  //Setup vertical interrupt for pin B8
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; 
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  //Setup vertical interrupt for pin B9
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; 
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Setup horizontal interrupt for pin A1
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; 
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Setup horizontal interrupt for pin C7
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; 
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 on */
  BSP_LED_On(LED2);
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @brief  This function return the ADC conversion result.
  * @retval The number into the range [0, 4095] as [0, 3.3]V.
  */
uint16_t Read_ADC(void)
{
  HAL_ADC_Start(&HADC);
  HAL_ADC_PollForConversion(&HADC, 100);
  
  return HAL_ADC_GetValue(&HADC);
}

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/**
  * @}
  */ /* End of MotionControl */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
