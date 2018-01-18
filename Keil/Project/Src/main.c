/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//macro definitions
#define STEP_CMD_ID 100
#define CMD_MAX_SIZE 30
#define MSG_END_CHAR '$'
#define CMD_buf_SIZE 30
//global variables
  //static msg's
    char Cmd_cflt_err_msg[]="1, CMD conflict";
    //incompatible command error message
    //conflict with current command
  
  static int Cur_CMD=0;
	static int Step_left=0;
    //uint8_t CMD_TO_EXE=0;
    //Probably not needed

  char CMD_buf[100];//buffer that stores the received msg/cmd
  //used in rx cplt function
    char Rx_buffer[100];
    unsigned char Rx_indx, Rx_data[2], Transfer_cplt;
  
    char Tx_buffer[2];
    int Rx_cb_indx=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM2_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//higher level cmds
  void exe_cmd(void);//you can say that this is a cmd dispatcher
//high level cmds
  void my_MX_init(void);
  void decode_msg(uint8_t end_char_index);
  void my_printf(void);
  bool isShort(int cmd_id);
  void exe_short_cmd(void);
  bool areCompatible(int cur_cmd_id, int new_cmd_id);
  void update_target(int cmd_id);
//low level controls
  void s_step(void);
//lowest level controls
  void trigger(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//the argument GPIO_Pin refers to the EXTI line
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//   /* Prevent unused argument(s) compilation warning */
//   //UNUSED(GPIO_Pin);

//   //if the EXTI line is the line for Limit Switch 0
//   //Limit Switch 0 is the only EXTI source
//   //meaning Limit Switch at 0 position is hit
//   // if (GPIO_Pin==LimitSW_0_Pin)
//   // {
//   //   /* code */
//   // }

//   //newer version written in switch statement 
//   //because there are a couple lines
//   switch(GPIO_Pin){
//     case LimitSW_0_Pin://EXTI7
//     break;
//     case LimitSW_1_Pin://EXTI6
//     break;
//     case B1_Pin://EXTI13
//       HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//     break;
//     default:
//     break;
//   }
// }
#define TRIGGER_CMD_ID 10
void exe_cmd(){
  switch(Cur_CMD){
    case TRIGGER_CMD_ID:
      trigger();
      Cur_CMD=0;//reset the current cmd
    break;
  }
}

void decode_msg(uint8_t end_char_index){
  int new_cmd_id=CMD_buf[0];
  char debug_msg[40];
  sprintf (debug_msg, "decode_msg called %d\n",end_char_index);
  HAL_UART_Transmit(&huart2, (uint8_t *)debug_msg, strlen(debug_msg) ,9999);//30 is buffer size
  //if short cmd such as ask status
  if(isShort(new_cmd_id)){
    exe_short_cmd();
    return;
  }
  //not short cmd

  //if there is an ongoing cmd
  if (Cur_CMD)
  {
    if (!areCompatible(Cur_CMD,new_cmd_id))
    {
      //causing conflict
      //print error msg
      HAL_UART_Transmit(&huart2, (uint8_t *)Cmd_cflt_err_msg, strlen(Cmd_cflt_err_msg),9999);
      //it really should be transmit_IT here to nonblockingly transmit, 
        //but using blocking transmit 
        //and to see this communicatio delay would be interesting
      //return keep executing current cmd
      return;
    }else{
      //is compatible
      Cur_CMD=new_cmd_id;
    }
  }else{
    //if no ongoing cmd
    //??activate CMD_TO_EXE flag?
    //is this needed
    //CMD_TO_EXE=1;
    Cur_CMD=new_cmd_id;
  }
  //if cmd is executed here, then parameter can be updated
  update_target(Cur_CMD);
}

//high level commands
void my_printf(){

}

bool isShort(int cmd_id){
  if (cmd_id==STEP_CMD_ID||cmd_id==TRIGGER_CMD_ID)
  {
    return false;
  }else{
    return true;
  }
  
}
void exe_short_cmd(){

}
bool areCompatible(int cur_cmd_id, int new_cmd_id){

  return false;
}


void update_target(int cmd_id){
  switch(cmd_id){
    case STEP_CMD_ID:
      Step_left=CMD_buf[1];
    break;
  }
}

//lowest level functions
void trigger(){
  //toggle the pin high
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_SET);
  //start timer 9 to toggle the trigger low
  HAL_TIM_Base_Start_IT(&htim9);
}

// void clear_cmd_buf(){
//   for (uint8_t i=0;i<CMD_MAX_SIZE;i++)//clear rx buffer
//   {
//     CMD_buf[i]=0;                     // clear Rx_buf before receiving new data
//   }
// }

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   uint8_t i;//max 255, or 511?
//   //my_toggleled();
//   //do this to indicate that this routine is entered
//   //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

//   if (huart->Instance == USART2)             //is current&USB uart?
//   {
//     //if data is not being received
//     if (Rx_cb_indx == 0)                       
//     {
//       //****thinking abt moving this to after decode msg
//       clear_cmd_buf();
//       // for (i=0;i<CMD_MAX_SIZE;i++)//clear rx buffer
//       // {
//       //  CMD_buf[i]=0;                     // clear Rx_buf before receiving new data
//       // }
//     }
//     //if received data is different from the end character
//     if (Rx_data[0]!=MSG_END_CHAR)                     
//     {
//       CMD_buf[Rx_cb_indx++]= Rx_data[0];     // store data in buffer

//       //HAL_UART_Transmit(&huart2, (uint8_t *)CMD_buf, CMD_buf_SIZE,99999);
//     }
//     else                                  // if received data = 13
//     {
//       //at the end, decode received msg
//       //decode_msg(Rx_cb_indx);
//       Rx_cb_indx= 0;//re-init index to zero when an end is detected
//       HAL_UART_Transmit(&huart2, (uint8_t *)CMD_buf, CMD_buf_SIZE,99999);
//       #ifdef RX_CB_END_DEBUG
//       //debug mode: blockingly echo received bytes array 
//       //when the end character detected
//       HAL_UART_Transmit(&huart2, (uint8_t *)CMD_buf, CMD_buf_SIZE,99999);
//         //buffer size could also be strlen(CMD_buf)
//       #endif
//       //maybe clear_cmd_buf() here
//     }
//     // activate receive in background
//     HAL_UART_Receive_IT (&huart2, Rx_data, 1);     
//   }
// }

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
//   //if timer 9 reached the target period
//   if(htim->Instance==TIM9){
//     HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,0);
//     HAL_TIM_Base_Stop_IT(&htim9);
//   }

// }
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/
  my_MX_init();
  

/* USER CODE BEGIN 2 */
  //needed to make first interrupt detected
  //not sure why, but this works
  HAL_TIM_Base_Start_IT(&htim9);
  HAL_TIM_Base_Stop_IT(&htim9);

  //enable interrutp events for uart2
  __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);//receive data not empty interrupt
  __HAL_UART_ENABLE_IT(&huart2,UART_IT_TC);//transmit complet interrupt
  //start the callback
  HAL_UART_Receive_IT (&huart2, Rx_data, 1); 
  // HAL_UART_Receive_IT (&huart2, Rx_buffer, 1); 
  //start timer2 in encoder interface mode
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
/* USER CODE END 2 */
  //token 1 of i am alive by blink the led
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_SET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,GPIO_PIN_RESET);
  //token 2 of i am alive by send a msg
  char alive_msg[]="Hello, I am alive\n";
  //blockingly send i am alive
  HAL_UART_Transmit(&huart2, (uint8_t *)alive_msg, strlen(alive_msg),99999);
  //non-blockingly send i am alive
  HAL_UART_Transmit_IT(&huart2, (uint8_t *)alive_msg, strlen(alive_msg));
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    if (Cur_CMD)//remember to declare the variable and function underneath
    {
      exe_cmd();//execute cmd based on cmd_buffer
      //is blocking
      //Cur_CMD=0;
    }
  }
  /* USER CODE END 3 */

}

//my version of initializations
//just a compilation of all the init calls
void my_MX_init(){
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
  MX_USART2_UART_Init();
  MX_TIM9_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* TIM1_BRK_TIM9_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967290;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM9 init function */
static void MX_TIM9_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 42000;
  //makes each tick at 84MHz/42k=2khz->.5ms
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
//****tentative here
//timer9 used for trigger reset
  htim9.Init.Period = 1000;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|Step_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Dir_pin_GPIO_Port, Dir_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LimitSW_0_Pin */
  GPIO_InitStruct.Pin = LimitSW_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LimitSW_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Step_pin_Pin */
  GPIO_InitStruct.Pin = Step_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Step_pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LimitSW_1_Pin */
  GPIO_InitStruct.Pin = LimitSW_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LimitSW_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Dir_pin_Pin */
  GPIO_InitStruct.Pin = Dir_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Dir_pin_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
