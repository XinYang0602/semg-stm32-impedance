/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : sEMG acquisition with periodic electrode impedance check
  *
  * Functionality:
  *  - Continuous ADC sampling for sEMG signal streaming (ADC Channel 0)
  *  - Periodic impedance measurement using test current injection (PA4)
  *    and voltage sensing (ADC Channel 5)
  *  - UART output for real-time monitoring
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdio.h>
#include <string.h>

/* ===================== User Configuration ===================== */

/* Known test resistor value (update to match hardware) */
#define R_TEST_OHMS    100000.0f   // 100 kΩ test resistor

/* ADC reference voltage */
#define VREF           3.3f

/* Acceptable electrode impedance range */
#define R_MIN_OK       10000.0f    // 10 kΩ
#define R_MAX_OK       300000.0f   // 300 kΩ

/* UART and impedance timing parameters */
#define UART_DIV       2           // Send 1 UART sample every N ADC samples
#define IMP_PERIOD_MS  200         // Impedance check interval (ms)

/* ===================== Peripheral Handles ===================== */

ADC_HandleTypeDef   hadc1;
UART_HandleTypeDef  huart1;

/* ===================== Global Variables ===================== */

uint32_t adc_value = 0;            // Latest EMG ADC sample
char msg[80];

static uint32_t adc_count = 0;
static uint32_t last_imp_ms = 0;

/* ===================== Function Prototypes ===================== */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);

static void  ADC_SwitchChannel(uint32_t channel);
static float Measure_Electrode_Impedance(void);

/* ===================== Helper Functions ===================== */

/**
 * @brief  Reconfigure ADC1 to sample a different channel.
 * @param  channel: ADC channel number (e.g., ADC_CHANNEL_0, ADC_CHANNEL_5)
 */
static void ADC_SwitchChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    HAL_ADC_Stop(&hadc1);

    sConfig.Channel      = channel;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  Measure electrode–skin impedance using voltage divider method.
 * @return Estimated impedance in ohms.
 *
 * Method:
 *  - Inject a small DC excitation via PA4
 *  - Measure resulting voltage on PA5 (ADC)
 *  - Compute impedance using resistor divider equation
 */
static float Measure_Electrode_Impedance(void)
{
    uint32_t adc_sum = 0;

    /* Switch ADC to impedance sensing channel (PA5) */
    ADC_SwitchChannel(ADC_CHANNEL_5);

    /* Enable test current injection */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_Delay(2);   // Short settling time (avoid blocking EMG stream)

    /* Average multiple ADC samples for noise reduction */
    for (int i = 0; i < 16; i++)
    {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 10);
        adc_sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }

    /* Disable test current to avoid DC bias */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    uint32_t adc_avg = adc_sum / 16;
    float v_meas = (float)adc_avg * VREF / 4095.0f;

    /* Boundary protection */
    if (v_meas >= VREF * 0.99f) return 1e9f;  // Open circuit
    if (v_meas <= VREF * 0.01f) return 0.0f;  // Short circuit

    /* Voltage divider equation */
    return R_TEST_OHMS * (v_meas / (VREF - v_meas));
}

/* ===================== Main Program ===================== */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_ADC1_Init();

    /* Default mode: continuous EMG sampling on ADC Channel 0 */
    ADC_SwitchChannel(ADC_CHANNEL_0);
    HAL_ADC_Start(&hadc1);

    last_imp_ms = HAL_GetTick();

    while (1)
    {
        /* ---------- EMG Streaming ---------- */
        if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
        {
            adc_value = HAL_ADC_GetValue(&hadc1);
            adc_count++;

            /* Downsample UART output to reduce bandwidth */
            if ((adc_count % UART_DIV) == 0)
            {
                int n = snprintf(msg, sizeof(msg), "%lu\r\n",
                                 (unsigned long)adc_value);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, n, 50);
            }
        }

        /* ---------- Periodic Impedance Check ---------- */
        uint32_t now = HAL_GetTick();
        if ((now - last_imp_ms) >= IMP_PERIOD_MS)
        {
            last_imp_ms = now;

            /* Temporarily stop EMG sampling */
            HAL_ADC_Stop(&hadc1);

            float R_skin = Measure_Electrode_Impedance();
            uint32_t R_kohm = (uint32_t)(R_skin / 1000.0f);

            /* Indicate electrode contact quality */
            if (R_skin >= R_MIN_OK && R_skin <= R_MAX_OK)
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                int n = snprintf(msg, sizeof(msg),
                                 "[IMP] R=%lu kOhm, good contact\r\n",
                                 (unsigned long)R_kohm);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, n, 100);
            }
            else
            {
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
                int n = snprintf(msg, sizeof(msg),
                                 "[IMP] R=%lu kOhm, poor contact\r\n",
                                 (unsigned long)R_kohm);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, n, 100);
            }

            /* Restore EMG acquisition */
            ADC_SwitchChannel(ADC_CHANNEL_0);
            HAL_ADC_Start(&hadc1);
        }
    }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_0;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure PA4 as push-pull output for test current injection */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Default low (no injection) */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /* Configure PA5 as analog input for ADC impedance sensing */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
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

