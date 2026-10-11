/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Dual Joystick & Triple DC Motor Control for STM32F103
  * @pins           : M1  -> IN1: PB5,  IN2: PB3 (EN1 connected to 5V)
  *                   M2  -> IN3: PA12, IN4: PA9 (EN2 connected to 5V)
  *                   M3  -> IN5: PA11, IN6: PB8 (EN3 connected to 5V)
  *                   J1  -> RX: PA3 (ADC1_IN3), RY: PA5 (ADC1_IN5)
  *                   J2  -> RX: PB1 (ADC1_IN9), RY: PB0 (ADC1_IN8)
  *                   LED -> PC13 (Built-in LED, Active LOW)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
/* ADC Raw Readings */
uint32_t adc_j1_rx = 2048; // PA3 (M3 Control)
uint32_t adc_j2_rx = 2048; // PB1 (J2 Steering - Left/Right)
uint32_t adc_j2_ry = 2048; // PB0 (J2 Throttle - Forward/Backward)

/* Heartbeat timer counter */
uint16_t heartbeat_counter = 0;

/* Control & ADC Constants */
#define STICK_CENTER 2048    // 12-bit ADC mid-point (~1.65V)
#define DEADZONE     250     // Range around center to suppress drift

/* Function Declarations */
static uint32_t ADC_ReadChannel(uint32_t channel);
static void Set_Motor_State(GPIO_TypeDef *port1, uint16_t pin1, GPIO_TypeDef *port2, uint16_t pin2, int32_t speed);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, SET);


  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* --- 0. Heartbeat LED Toggle (PC13) --- */
    heartbeat_counter++;
    if (heartbeat_counter >= 25) // 25 * 10ms = 250ms toggle rate
    {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      heartbeat_counter = 0;
    }

    /* --- 1. Read Joystick ADC Values --- */
    adc_j1_rx = ADC_ReadChannel(ADC_CHANNEL_3); // PA3 (J1 RX -> Motor 3)
    adc_j2_rx = ADC_ReadChannel(ADC_CHANNEL_9); // PB1 (J2 RX -> Differential Steering)
    adc_j2_ry = ADC_ReadChannel(ADC_CHANNEL_8); // PB0 (J2 RY -> Forward / Backward)

    /* --- 2. Differential Drive Logic (J2 -> M1 & M2) --- */
    int32_t throttle = (int32_t)adc_j2_ry - STICK_CENTER; // Y-axis
    int32_t steering = (int32_t)adc_j2_rx - STICK_CENTER; // X-axis

    // Apply deadzone filtering
    if (throttle > -DEADZONE && throttle < DEADZONE) throttle = 0;
    if (steering > -DEADZONE && steering < DEADZONE) steering = 0;

    // Mixing for differential drive (Tank drive style)
    int32_t m1_speed = throttle + steering; // Motor 1 (Left/Right blend)
    int32_t m2_speed = throttle - steering; // Motor 2 (Opposite direction on spin)

    /* --- 3. Single Motor Logic (J1 RX -> M3) --- */
    int32_t m3_speed = (int32_t)adc_j1_rx - STICK_CENTER;
    if (m3_speed > -DEADZONE && m3_speed < DEADZONE) m3_speed = 0;

    /* --- 4. Apply Drive States to Pins --- */
    // M1: IN1 (PB5), IN2 (PB3)
    Set_Motor_State(GPIOB, GPIO_PIN_5, GPIOB, GPIO_PIN_3, m1_speed);

    // M2: IN3 (PA12), IN4 (PA9)
    Set_Motor_State(GPIOA, GPIO_PIN_12, GPIOA, GPIO_PIN_9, - m2_speed);

    // M3: IN5 (PA11), IN6 (PB8)
    Set_Motor_State(GPIOB, GPIO_PIN_15, GPIOB, GPIO_PIN_13, m3_speed);

    /* 10ms loop time */
    HAL_Delay(10);
  }
}

/**
  * @brief  Applies direction commands to direct GPIO motor output pins.
  */
static void Set_Motor_State(GPIO_TypeDef *port1, uint16_t pin1, GPIO_TypeDef *port2, uint16_t pin2, int32_t speed)
{
  if (speed > 0)
  {
    // Forward direction
    HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_RESET);
  }
  else if (speed < 0)
  {
    // Reverse direction
    HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_SET);
  }
  else
  {
    // Stop / Coast
    HAL_GPIO_WritePin(port1, pin1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(port2, pin2, GPIO_PIN_RESET);
  }
}

/**
  * @brief Helper to read dynamic ADC channels sequentially
  */
static uint32_t ADC_ReadChannel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  HAL_ADC_Start(&hadc1);

  uint32_t val = STICK_CENTER;
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  {
    val = HAL_ADC_GetValue(&hadc1);
  }
  HAL_ADC_Stop(&hadc1);

  return val;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_5
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB15 PB3 PB5
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_5
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
#ifdef USE_FULL_ASSERT
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
