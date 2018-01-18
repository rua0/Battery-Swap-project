/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim9;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line[9:5] interrupts.
*/
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
* @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
*/
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  //if timer 9 reached the target period
  //say timer 9 is reserved for trigger
  if(htim->Instance==TIM9){
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_RESET);
    HAL_TIM_Base_Stop_IT(&htim9);
  }

}
/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}
extern void decode_msg(uint8_t end_char_index);
extern char CMD_buf[100];
extern int Rx_cb_indx;
extern unsigned char Rx_indx, Rx_data[2], Transfer_cplt;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t i;//max 255, or 511?
  //my_toggleled();
  //do this to indicate that this routine is entered
  //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

  if (huart->Instance == USART2)             //is current&USB uart?
  {
    //if data is not being received
    if (Rx_cb_indx == 0)                       
    {
      //****thinking abt moving this to after decode msg
      //clear_cmd_buf();
      for (i=0;i<30;i++)//clear rx buffer
      {
        CMD_buf[i]=0;                     // clear Rx_buf before receiving new data
      }
    }
    //if received data is different from the end character
    if (Rx_data[0]!='$')                     
    {
      CMD_buf[Rx_cb_indx++]= Rx_data[0];     // store data in buffer

      //HAL_UART_Transmit(&huart2, (uint8_t *)CMD_buf, CMD_buf_SIZE,99999);
    }
    else                                  // if received data = 13
    {
      //at the end, decode received msg
      decode_msg(Rx_cb_indx);
      Rx_cb_indx= 0;//re-init index to zero when an end is detected
      HAL_UART_Transmit(&huart2, (uint8_t *)CMD_buf, 30,99999);
      #ifdef RX_CB_END_DEBUG
      //debug mode: blockingly echo received bytes array 
      //when the end character detected
      HAL_UART_Transmit(&huart2, (uint8_t *)CMD_buf, 30E,99999);
        //buffer size could also be strlen(CMD_buf)
      #endif
      //maybe clear_cmd_buf() here
    }
    // activate receive in background
    HAL_UART_Receive_IT (&huart2, Rx_data, 1);     
  }
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(GPIO_Pin);

  //if the EXTI line is the line for Limit Switch 0
  //Limit Switch 0 is the only EXTI source
  //meaning Limit Switch at 0 position is hit
  // if (GPIO_Pin==LimitSW_0_Pin)
  // {
  //   /* code */
  // }

  //newer version written in switch statement 
  //because there are a couple lines
  switch(GPIO_Pin){
    case LimitSW_0_Pin://EXTI7
    break;
    case LimitSW_1_Pin://EXTI6
    break;
    case B1_Pin://EXTI13
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    break;
    default:
    break;
  }
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
