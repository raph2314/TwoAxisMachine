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

// #define TEST_MOTOR	//!< Comment out this line to test the ADC	

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
// #define MICROSTEPPING_MOTOR_USART_EXAMPLE  //!< Uncomment to performe the USART example
// #define MOTOR_EDGE_RAMPUP 
// #define MOTOR_EDGE_BOTH_MOTOR //!< Uncomment to perform edge case 1 ()
// #define MOTOR_DEMO_1 //!< Uncomment to perform step demo
// #define MOTOR_DEMO_2 //!< HACKY: Uncomment to perform step demo
// #define MOTOR_DEMO_3 //!< Uncomment to perform max speed demo
#define ADC_MOTOR_CONTROL //!< Uncomment for ADC demo and final competition 
// #define SERIAL_USART_EXAMPLE


// #if ((defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
//   #error "Please select an option only!"
// #elif ((!defined (MICROSTEPPING_MOTOR_EXAMPLE)) && (!defined (MICROSTEPPING_MOTOR_USART_EXAMPLE)))
//   #error "Please select an option!"
// #endif
// #if (defined (MICROSTEPPING_MOTOR_USART_EXAMPLE) && (!defined (NUCLEO_USE_USART)))
//   #error "Please define "NUCLEO_USE_USART" in "stm32fxxx_x-nucleo-ihm02a1.h"!"
// #endif

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
uint16_t* Read_ADC(void);

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

  /********************************  FROM POLLING     ****************************/
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

  //Motor Initializations
  /* Fill the L6470_DaisyChainMnemonic structure */
  Fill_L6470_DaisyChainMnemonic();
  
  /*Initialize the motor parameters */
  Motor_Param_Reg_Init();

  L6470_HardStop(1);
  L6470_HardStop(0);

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
  
  /*Initialize the ADC*/
  MX_ADC1_Init(); 

// #endif

  while (1)
  {

    #ifdef TEST_MOTOR		

        /* Check if any Application Command for L6470 has been entered by USART */
        USART_CheckAppCmd();
        
    #else

        //
        uint16_t myADCVal;
        myADCVal = Read_ADC();
        USART_Transmit(&huart2, " ADC Read: ");
        USART_Transmit(&huart2, num2hex(myADCVal, WORD_F));
        USART_Transmit(&huart2, " \n\r");
    #endif		
  }
  #elif defined(MOTOR_EDGE_RAMPUP)
  /*Ensure you mark the starting position, and that there is sufficient
    amount of vertical range */
    L6470_Run(0,L6470_DIR_REV_ID,27500);
    HAL_Delay(2500); //run for 2.5s
    L6470_HardStop(0);
    HAL_Delay(10000);
    L6470_Run(0,L6470_DIR_REV_ID,27500);
    HAL_Delay(5000); //run for 5s
    L6470_HardStop(0);
    HAL_Delay(10000);
    L6470_Run(0,L6470_DIR_REV_ID,27500);
    HAL_Delay(10000); //run for 10s
    L6470_HardStop(0);
    HAL_Delay(10000);
    L6470_Run(0,L6470_DIR_REV_ID,27500);
    HAL_Delay(15000); //run for 15s
    L6470_HardStop(0);
    HAL_Delay(10000);
    L6470_GoHome(0);
    while (1);
  #elif defined(MOTOR_EDGE_BOTH_MOTOR)
  /*May need to change the direction */
    L6470_Run(0,L6470_DIR_REV_ID,27500);
    L6470_Run(1, L6470_DIR_FWD_ID, 27500);
    HAL_Delay(20000);

    L6470_HardStop(1);
    L6470_HardStop(0);
    while (1);
  #elif defined (MOTOR_DEMO_1) //!!! NEED TO CHECK DIRECTION
    L6470_Move(0,L6470_DIR_FWD_ID,1282000); //Should move up vertically by 100mm, determined from motor characterization
    while(1);

  #elif defined (MOTOR_DEMO_2)
    L6470_Move(0,L6470_DIR_REV_ID,1265500); //Should move down vertically by 100mm
    while(1);

  #elif defined (MOTOR_DEMO_3)
    L6470_Run(0,L6470_DIR_FWD_ID,27500); //Should move down vertically by 100mm
    HAL_Delay(24096); //Value determined from motor characterization
    L6470_HardStop(0);
    HAL_Delay(5000);
    L6470_Run(0,L6470_DIR_REV_ID,27500); //Should move up vertically by 100mm
    HAL_Delay(24096);
    L6470_HardStop(0);
    while(1);
  #elif defined (ADC_MOTOR_CONTROL)

      /* Fill the L6470_DaisyChainMnemonic structure */
    Fill_L6470_DaisyChainMnemonic();
    
    /*Initialize the motor parameters */
    Motor_Param_Reg_Init();
    
    /*Initialize the ADC*/
    MX_ADC1_Init(); 

    // Variables to track the "stopped" state of the motor
    uint16_t horizStopped = 0;
    uint16_t verticalStopped = 0;

    // Variables to track the "speed set" state of the motor
    uint16_t horizSpeedSet, verticalSpeedSet;
    
    // ADC values, broken down into horizontal and vertical components
    uint16_t* myADCVal;
    uint16_t horizontalVal, verticalVal;

    // Current speed that the motors are set to
    uint16_t horizSpeed, verticalSpeed;

    // Previous speed that the motors were set to, used for filtering out noise
    uint16_t prevHorizSpeed = 0; 
    uint16_t prevVerticalSpeed = 0; 
    
    // Track direction of both motors
    eL6470_DirId_t horizDir;
    eL6470_DirId_t verticalDir;

    while(1) {
        myADCVal = Read_ADC();
        horizontalVal= myADCVal[0];
        verticalVal=myADCVal[1];

        // Compute different between previous value. Only set speed when the change is greater than 15
        horizSpeedSet = (abs(horizontalVal - prevHorizSpeed) > 32) ? 0 : 1; 
        verticalSpeedSet = (abs(verticalVal - prevVerticalSpeed) > 32) ? 0: 1;

        //////////////////////////////
        // HORIZONTAL MOTOR CONTROL //
        //////////////////////////////

        // Stopped condition
        if (0x0700 < horizontalVal && 0x08F0 > horizontalVal && !horizStopped){
          L6470_HardStop(1); 
          USART_Transmit(&huart2, " HORIZ STOPPED");
          USART_Transmit(&huart2, " \n\r"); 
          horizStopped = 1; 
        }

        // Reverse direction (LEFT)
        else if(horizontalVal < 0x0700 && !horizSpeedSet ) {
          horizSpeed = 15 * (0x0700 - horizontalVal); 
          horizDir = L6470_DIR_REV_ID;
          L6470_Run(1, horizDir, horizSpeed);
          USART_Transmit(&huart2, " HORIZ REV DIR: ");
          USART_Transmit(&huart2, num2hex(horizSpeed, WORD_F));
          USART_Transmit(&huart2, " \n\r");
          prevHorizSpeed = horizontalVal; 
          horizStopped = 0; 
        }
        
        // Forward direction (Right)
        else if(horizontalVal > 0x08F0 && !horizSpeedSet) {
          horizSpeed = 15 * (horizontalVal - 0x08F0); 
          horizDir = L6470_DIR_FWD_ID;
          L6470_Run(1, horizDir, horizSpeed);
          USART_Transmit(&huart2, " HORIZ FWD DIR: ");
          USART_Transmit(&huart2, num2hex(horizSpeed, WORD_F));
          USART_Transmit(&huart2, " \n\r");
          prevHorizSpeed = horizontalVal; 
          horizStopped = 0; 
        }
        //////////////////////////////
        //  VERTICAL MOTOR CONTROL  //
        //////////////////////////////
        
        if(0x0700 < verticalVal && 0x08F0 > verticalVal && !verticalStopped){
          L6470_HardStop(0);
          USART_Transmit(&huart2, " VERTICAL STOPPED");
          USART_Transmit(&huart2, " \n\r");
          verticalStopped = 1; 
        }

        else if(verticalVal < 0x0700 && !verticalSpeedSet) {
          verticalSpeed = 15 * (0x0700 - verticalVal);
          verticalDir = L6470_DIR_REV_ID;
          L6470_Run(0, verticalDir, verticalSpeed);
          USART_Transmit(&huart2, " VERTICAL REV DIR: ");
          USART_Transmit(&huart2, num2hex(verticalSpeed, WORD_F));
          USART_Transmit(&huart2, " \n\r");
          prevVerticalSpeed = verticalVal; 
          verticalStopped = 0; 
        }

        else if(verticalVal > 0x08F0 && !verticalSpeedSet) {
          verticalSpeed = 15 * (verticalVal - 0x08F0);
          verticalDir = L6470_DIR_FWD_ID;
          L6470_Run(0, verticalDir, verticalSpeed);
          USART_Transmit(&huart2, " VERTICAL FWD DIR: ");
          USART_Transmit(&huart2, num2hex(verticalSpeed, WORD_F));
          USART_Transmit(&huart2, " \n\r");
          prevVerticalSpeed = verticalVal; 
          verticalStopped = 0; 
        }


      

    }
  #elif defined (SERIAL_USART_EXAMPLE)
    while(1) {
      USART_Transmit(&huart2, "A");
      HAL_Delay(1000);
    }

#endif
}

void GPIO_CustomInit (){
  GPIO_InitTypeDef GPIO_InitStruct;

  //LED output to pin 9
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
uint16_t* Read_ADC(void)
{

  HAL_ADC_Start(&HADC);
  HAL_ADC_PollForConversion(&HADC, 100);
  uint16_t value1=HAL_ADC_GetValue(&HADC);
  HAL_ADC_PollForConversion(&HADC, 100);
  uint16_t value2=HAL_ADC_GetValue(&HADC);
  HAL_ADC_Stop(&HADC);
  uint16_t values[2]={value1, value2};

  return values;
}

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/**
  * @}
  */ /* End of MotionControl */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
