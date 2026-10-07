
#include "main.h"

UART_HandleTypeDef huart3;
DMA_HandleTypeDef  hdma_usart3_rx;   // kept so generated msp/it files still link
TIM_HandleTypeDef  htim1;          // <-- add this line
void SystemClock_Config(void);
static void GPIO_Init(void);
static void USART3_Init(void);
static void TIM1_FreeRun_Init(void);

/* ---------------- Stepper pin mapping ---------------- */
typedef struct {
  GPIO_TypeDef* pulPort;
  uint16_t      pulPin;
  GPIO_TypeDef* dirPort;
  uint16_t      dirPin;
} Stepper_t;

static const Stepper_t stp[3] = {
  { GPIOB, GPIO_PIN_15, GPIOB, GPIO_PIN_14 },  // Axis 0: Base
  { GPIOA, GPIO_PIN_5,  GPIOA, GPIO_PIN_6  },  // Axis 1: Shoulder
  { GPIOB, GPIO_PIN_0,  GPIOA, GPIO_PIN_0  }   // Axis 2: Elbow
};

#define PULSE_WIDTH_US   10u     // TB6600 needs >= 2.5 us
#define MIN_SAFE_US      100u    // never step faster than this
#define FAILSAFE_WRAPS   3u      // 3 x 65.5 ms ~= 200 ms without a frame -> stop

static uint16_t interval[3]    = {0, 0, 0};   // us between steps (0 = stop)
static uint16_t lastStep[3]    = {0, 0, 0};
static uint16_t pulseStart[3]  = {0, 0, 0};
static uint8_t  pulseActive[3] = {0, 0, 0};

static inline void LED_Update(void)
{
  // PC13 LED is active low: ON while any motor is commanded to move
  if (interval[0] || interval[1] || interval[2])
    GPIOC->BSRR = (uint32_t)GPIO_PIN_13 << 16;   // LED on
  else
    GPIOC->BSRR = GPIO_PIN_13;                   // LED off
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  GPIO_Init();
  USART3_Init();
  TIM1_FreeRun_Init();

  // Startup blink (3x) so you know the code is running
  for (int i = 0; i < 6; i++)
  {
    GPIOC->ODR ^= GPIO_PIN_13;
    for (volatile uint32_t d = 0; d < 800000; d++);
  }
  GPIOC->BSRR = GPIO_PIN_13;   // LED off

  /* Frame parser state */
  uint8_t state = 0;           // 0: wait AA, 1: wait 55, 2: payload, 3: checksum
  uint8_t buf[9];
  uint8_t idx = 0;
  uint8_t sum = 0;

  uint16_t lastCnt = 0;
  uint32_t wraps   = 0;

  while (1)
  {
    uint16_t now = (uint16_t)TIM1->CNT;

    /* ---- Failsafe timer: count 65.5 ms wraps of the free-running timer ---- */
    if (now < lastCnt)
    {
      if (wraps < FAILSAFE_WRAPS) wraps++;
    }
    lastCnt = now;

    if (wraps >= FAILSAFE_WRAPS)
    {
      if (interval[0] || interval[1] || interval[2])
      {
        interval[0] = interval[1] = interval[2] = 0;
        LED_Update();
      }
    }

    /* ---- Receive bytes (polled) ---- */
    uint32_t sr = USART3->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE))
    {
      uint8_t data = (uint8_t)(USART3->DR & 0xFF);   // reading DR also clears ORE

      if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE))
      {
        state = 0;   // error: resync
      }
      else
      {
        switch (state)
        {
          case 0:
            if (data == 0xAA) state = 1;
            break;

          case 1:
            if (data == 0x55)      { state = 2; idx = 0; sum = 0; }
            else if (data == 0xAA) { state = 1; }
            else                   { state = 0; }
            break;

          case 2:
            buf[idx++] = data;
            sum ^= data;
            if (idx >= 9) state = 3;
            break;

          case 3:
            if (data == sum)
            {
              for (int i = 0; i < 3; i++)
              {
                uint8_t  dir = buf[i * 3];
                uint16_t iv  = ((uint16_t)buf[i * 3 + 1] << 8) | buf[i * 3 + 2];

                if (iv > 0 && iv < MIN_SAFE_US) iv = MIN_SAFE_US;

                if (dir) stp[i].dirPort->BSRR = stp[i].dirPin;
                else     stp[i].dirPort->BSRR = (uint32_t)stp[i].dirPin << 16;

                interval[i] = iv;
              }
              wraps = 0;          // frame OK -> reset failsafe
              LED_Update();
            }
            state = 0;
            break;

          default:
            state = 0;
            break;
        }
      }
    }

    /* ---- Step pulse generation (non-blocking) ---- */
    for (int i = 0; i < 3; i++)
    {
      if (pulseActive[i])
      {
        if ((uint16_t)(now - pulseStart[i]) >= PULSE_WIDTH_US)
        {
          stp[i].pulPort->BSRR = (uint32_t)stp[i].pulPin << 16;   // pulse LOW
          pulseActive[i] = 0;
        }
      }
      else if (interval[i] > 0 && (uint16_t)(now - lastStep[i]) >= interval[i])
      {
        stp[i].pulPort->BSRR = stp[i].pulPin;                      // pulse HIGH
        lastStep[i]    = now;
        pulseStart[i]  = now;
        pulseActive[i] = 1;
      }
    }
  }
}

/* TIM1 free-running 1 MHz counter (1 count = 1 us), no interrupt */
static void TIM1_FreeRun_Init(void)
{
  __HAL_RCC_TIM1_CLK_ENABLE();
  TIM1->PSC = 63;          // 64 MHz / 64 = 1 MHz
  TIM1->ARR = 0xFFFF;
  TIM1->EGR = TIM_EGR_UG;  // load prescaler
  TIM1->CR1 |= TIM_CR1_CEN;
}

static void GPIO_Init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_5 | GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

  g.Pin = GPIO_PIN_13; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &g);

  g.Pin = GPIO_PIN_0 | GPIO_PIN_5 | GPIO_PIN_6; g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &g);

  g.Pin = GPIO_PIN_0 | GPIO_PIN_14 | GPIO_PIN_15; g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &g);
}

static void USART3_Init(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_USART3_CLK_ENABLE();

  g.Pin = GPIO_PIN_10; g.Mode = GPIO_MODE_AF_PP; g.Speed = GPIO_SPEED_FREQ_HIGH;   // TX
  HAL_GPIO_Init(GPIOB, &g);

  g.Pin = GPIO_PIN_11; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;             // RX
  HAL_GPIO_Init(GPIOB, &g);

  huart3.Instance          = USART3;
  huart3.Init.BaudRate     = 115200;
  huart3.Init.WordLength   = UART_WORDLENGTH_8B;
  huart3.Init.StopBits     = UART_STOPBITS_1;
  huart3.Init.Parity       = UART_PARITY_NONE;
  huart3.Init.Mode         = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* Internal HSI -> PLL x16 = 64 MHz (no external crystal needed) */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  osc.HSIState            = RCC_HSI_ON;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.PLL.PLLState        = RCC_PLL_ON;
  osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  osc.PLL.PLLMUL          = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
  {
    Error_Handler();
  }

  clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;   // USART3 on APB1 = 32 MHz
  clk.APB2CLKDivider = RCC_HCLK_DIV1;   // TIM1 on APB2 = 64 MHz
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif

