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
#include "lcd_drv.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

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
static void MX_I2C1_Init(void);
static void MX_USART6_UART_Init(void);
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
const char* pedestrian_crossing = "Pedestrian crossing";
const char* ambar_sem = "Ambar vehicle semaphore";
const char* vehicles_crossing = "Vehicles crossing";
const char* blinking_green_pedestrian = "Bliking green pedestrian semaphore";

/* LCD message colors */
static volatile char green[]="xPulse Boton";
static volatile char red[]="xPase";
static volatile char ambar[]= "xEspere verde"; 


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
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim11);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  next_state = ST_VEHICLES_CROSSING; /* Vehicles crossing by default */ 
  begin(16,2,LCD_5x8DOTS);
  clear();
  setCursor(0,0);

  while (1)
  {
    /* USER CODE END WHILE */
    switch (next_state) {
      case ST_PEDESTRIAN_CROSSING:
        control_semaphores(1,0,0,0,1);
        clear();
        setCursor(0,0);
        setColorGreen();
        HAL_I2C_Master_Transmit(&hi2c1,LCD,red,sizeof(red)-1,100);
        HAL_UART_Transmit(&huart6,pedestrian_crossing,sizeof(pedestrian_crossing),100);
        turn_on_beep();
        next_state = ST_BLINKING_GREEN_PEDESTRIAN;
        break;
      case ST_AMBAR_SEM:
        control_semaphores(0,0,1,1,0);
        clear();
        setCursor(0,0);
        setColorBlue();
        HAL_I2C_Master_Transmit(&hi2c1,LCD,ambar,sizeof(ambar)-1,100);
        HAL_UART_Transmit(&huart6,ambar_sem,sizeof(ambar_sem),100);
        HAL_Delay(g_delay_change_sem);
        next_state = ST_PEDESTRIAN_CROSSING;
        break;
      case ST_VEHICLES_CROSSING:
        control_semaphores(0,1,0,1,0);
        clear();
        setCursor(0,0);
        setColorRed();
        HAL_I2C_Master_Transmit(&hi2c1,LCD,green,sizeof(green)-1,100);
        HAL_UART_Transmit(&huart6,vehicles_crossing,sizeof(vehicles_crossing),100);
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
        HAL_UART_Transmit(&huart6,blinking_green_pedestrian,sizeof(blinking_green_pedestrian),100);
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

void i2c_send_byte(unsigned char dta) {
   HAL_I2C_Master_Transmit(&hi2c1,LCD,&dta,sizeof(dta),100);  
}

void i2c_send_byteS(unsigned char *dta, unsigned char len) {
  for(int i=0; i<len; i++) {
    HAL_I2C_Master_Transmit(&hi2c1,LCD,&dta[i],sizeof(dta),100); 
  }
}

inline void command(uint8_t value) {
  unsigned char dta[2] = {0x80, value};
  i2c_send_byteS(dta, 2);
}

void begin(uint8_t cols, uint8_t lines, uint8_t dotsize) 
{
    if (lines > 1) {
        _displayfunction |= LCD_2LINE;
    }
    _numlines = lines;
    _currline = 0;

    if ((dotsize != 0) && (lines == 1)) {
        _displayfunction |= LCD_5x10DOTS;
    }

    HAL_Delay(50);

    command(LCD_FUNCTIONSET | _displayfunction);
    HAL_Delay(5);
    command(LCD_FUNCTIONSET | _displayfunction);
    HAL_Delay(1);
    command(LCD_FUNCTIONSET | _displayfunction);

    command(LCD_FUNCTIONSET | _displayfunction);

    _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    display();
    clear();

    _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    command(LCD_ENTRYMODESET | _displaymode);
    
    setReg(REG_MODE1, 0);
    setReg(REG_OUTPUT, 0xFF);
    setReg(REG_MODE2, 0x20);
}

void clear() {
    command(LCD_CLEARDISPLAY);
    HAL_Delay(2);
}

void home() {
    command(LCD_RETURNHOME);
    HAL_Delay(2);
}

void setCursor(uint8_t col, uint8_t row) {
    col = (row == 0 ? col|0x80 : col|0xc0);
    unsigned char dta[2] = {0x80, col};
    i2c_send_byteS(dta, 2);
}

void noDisplay() {
    _displaycontrol &= ~LCD_DISPLAYON;
    command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void display() {
    _displaycontrol |= LCD_DISPLAYON;
    command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void noCursor() {
    _displaycontrol &= ~LCD_CURSORON;
    command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void cursor() {
    _displaycontrol |= LCD_CURSORON;
    command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void noBlink() {
    _displaycontrol &= ~LCD_BLINKON;
    command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void blink() {
    _displaycontrol |= LCD_BLINKON;
    command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void scrollDisplayLeft(void) {
    command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void scrollDisplayRight(void) {
    command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void leftToRight(void) {
    _displaymode |= LCD_ENTRYLEFT;
    command(LCD_ENTRYMODESET | _displaymode);
}

void rightToLeft(void) {
    _displaymode &= ~LCD_ENTRYLEFT;
    command(LCD_ENTRYMODESET | _displaymode);
}

void autoscroll(void) {
    _displaymode |= LCD_ENTRYSHIFTINCREMENT;
    command(LCD_ENTRYMODESET | _displaymode);
}

void noAutoscroll(void) {
    _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
    command(LCD_ENTRYMODESET | _displaymode);
}

void createChar(uint8_t location, uint8_t charmap[]) {

    location &= 0x7;
    command(LCD_SETCGRAMADDR | (location << 3));
    unsigned char dta[9];
    dta[0] = 0x40;

    for(int i=0; i < 8; i++) {
        dta[i+1] = charmap[i];
    }

    i2c_send_byteS(dta, 9);
}

inline size_t write(char  value[])
{
  int i;
  for(i = 0; i < sizeof(value) ; i++){
    unsigned char dta[2] = {0x40, value[i]};
    i2c_send_byteS(dta, 2);
  }
  return 1; // assume sucess
}

void setReg(unsigned char addr, unsigned char dta) {
  uint8_t datos[2]={addr,dta};
  HAL_I2C_Master_Transmit(&hi2c1,RGB,datos,sizeof(datos),100); 
}

void setRGB(unsigned char r, unsigned char g, unsigned char b) {
    setReg(REG_RED, r);
    setReg(REG_GREEN, g);
    setReg(REG_BLUE, b);
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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

