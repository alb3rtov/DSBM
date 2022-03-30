/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void turn_on_beep(void);
uint32_t read_sensor(void);
void blink_green_pedestrian_sem(void);
void control_semaphores(int r1, int g1, int a1, int r2, int g2);
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Variables */
uint32_t g_delay_change_sem = 3000;
uint32_t g_delay_pedestrian_sem = 15000;
uint32_t g_sensor_delay = 0;
uint32_t current_time, start_time, final_time, echo_width, distance;
static enum {ST_PEDESTRIAN_CROSSING, ST_AMBAR_SEM, ST_VEHICLES_CROSSING, ST_BLINKING_GREEN_PEDESTRIAN} next_state;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_TIM11_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim11);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  next_state = ST_VEHICLES_CROSSING; /* Vehicles crossing by default */ 

  while (1)
  {
    /* USER CODE END WHILE */
    switch (next_state) {
      case ST_PEDESTRIAN_CROSSING:
        control_semaphores(1,0,0,0,1);
        turn_on_beep();
        next_state = ST_BLINKING_GREEN_PEDESTRIAN;
        break;
      case ST_AMBAR_SEM:
        control_semaphores(0,0,1,1,0);
        HAL_Delay(g_delay_change_sem);
        next_state = ST_PEDESTRIAN_CROSSING;
        break;
      case ST_VEHICLES_CROSSING:
        control_semaphores(0,1,0,1,0);
        if (g_sensor_delay == 0) {
          distance = read_sensor();
          if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)) || distance <= 20) {
            g_sensor_delay = 1;
            next_state = ST_AMBAR_SEM;
            HAL_Delay(g_delay_change_sem);
          }
        } else {
          HAL_Delay(5000);
          g_sensor_delay = 0;
        }
         
        break;
      case ST_BLINKING_GREEN_PEDESTRIAN:
        blink_green_pedestrian_sem();
        next_state = ST_VEHICLES_CROSSING;
        break;
    }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* Turn on the beep while pedestrians are crossing */
void turn_on_beep(void) {
  for (int i = 0; i < 15; i++) {
    htim2.Instance->CCR1 = 40;
    HAL_Delay(500);
    htim2.Instance->CCR1 = 0;
    HAL_Delay(500);
  }
}

/* Timer interruption */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim->Instance == TIM11) {
    current_time++;
  }
}

/* Change pin to echo mode */
void change_pin_echo(void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* Change pin to trigger mode */
void change_pin_trigger(void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* Send signal to detect */
void send_trigger(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); 
	HAL_Delay(50);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
}

/* Meaure time of the echo */
void measure_echo(void) {
  while (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)));
  start_time = current_time;
  while (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6));
}

/* Change sensor mode and compute echo width of the pulse and distance */
uint32_t read_sensor(void) {
	current_time = 0;

  send_trigger();
  change_pin_echo();
  measure_echo();
  change_pin_trigger();

  final_time = current_time;
  echo_width = final_time - start_time;

  return (echo_width * 10/58);
}

/* Blink green LED of pedestrian semaphore */
void blink_green_pedestrian_sem(void) {
  for (int i = 0; i < 3; i++) {
    HAL_GPIO_WritePin(GPIOB, LG_PEDESTRIAN_Pin, GPIO_PIN_RESET);
    htim2.Instance->CCR1 = 100;
    HAL_Delay(250);
    htim2.Instance->CCR1 = 0;
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOB, LG_PEDESTRIAN_Pin, GPIO_PIN_SET);
    htim2.Instance->CCR1 = 100;
    HAL_Delay(250);
    htim2.Instance->CCR1 = 0;
    HAL_Delay(250);
  }
  HAL_GPIO_WritePin(GPIOB, LG_PEDESTRIAN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LR_VEHICLE_Pin, GPIO_PIN_RESET);
}

/* Set and reset LEDs */
void control_semaphores(int r1, int g1, int a1, int r2, int g2) {
  if (r1)
    HAL_GPIO_WritePin(GPIOB, LR_VEHICLE_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOB, LR_VEHICLE_Pin, GPIO_PIN_RESET);
  if (g1)
    HAL_GPIO_WritePin(GPIOA, LG_VEHICLE_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOA, LG_VEHICLE_Pin, GPIO_PIN_RESET);
  if (a1)
    HAL_GPIO_WritePin(GPIOB, LY_VEHICLE_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOB, LY_VEHICLE_Pin, GPIO_PIN_RESET);
  if (r2)
    HAL_GPIO_WritePin(GPIOA, LR_PEDESTRIAN_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOA, LR_PEDESTRIAN_Pin, GPIO_PIN_RESET);
  if (g2)
    HAL_GPIO_WritePin(GPIOB, LG_PEDESTRIAN_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(GPIOB, LG_PEDESTRIAN_Pin, GPIO_PIN_RESET);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1290;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 254;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 98;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 11;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6|LG_VEHICLE_Pin|LR_PEDESTRIAN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LY_VEHICLE_Pin|LR_VEHICLE_Pin|LG_PEDESTRIAN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA6 LG_VEHICLE_Pin LR_PEDESTRIAN_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6|LG_VEHICLE_Pin|LR_PEDESTRIAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LY_VEHICLE_Pin LR_VEHICLE_Pin LG_PEDESTRIAN_Pin */
  GPIO_InitStruct.Pin = LY_VEHICLE_Pin|LR_VEHICLE_Pin|LG_PEDESTRIAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

